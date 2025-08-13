# Hardware loop: Motor + LEDs + Button + Heart sensor
from gpiozero import LED, Button
from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c
import RPi.GPIO as GPIO
import webbrowser, json, time, os, tempfile, signal, sys, random

# Files shared with app.py
HR_FILE        = "/tmp/hr.json"
SPEED_FILE     = "/tmp/speed.json"
MOTOR_CMD_FILE = "/tmp/motor_cmd.json"

def _atomic_write(path, obj):
    d = os.path.dirname(path) or "."
    with tempfile.NamedTemporaryFile("w", dir=d, delete=False) as tf:
        json.dump(obj, tf); tmp = tf.name
    os.replace(tmp, path)

def write_hr(bpm, status="Normal"):
    try: _atomic_write(HR_FILE, {"bpm": int(bpm), "status": status, "ts": time.time()})
    except Exception: pass

def write_speed(duty, hazard=False):
    try: _atomic_write(SPEED_FILE, {"duty": int(duty), "hazard": bool(hazard), "ts": time.time()})
    except Exception: pass

def read_external_hazard():
    try:
        with open(MOTOR_CMD_FILE, "r") as f: data = json.load(f)
        return bool(data.get("hazard", False))
    except Exception:
        return False

# ---- Motor pins (change if yours differ)
ENA, IN1, IN2 = 12, 5, 24    # ENA=GPIO12(PWM), IN1=GPIO5, IN2=GPIO24
PWM_FREQ = 1000
RAMP_STEP, RAMP_INTERVAL = 5, 0.20
NORMAL_TARGET, HAZARD_TARGET = 100, 0

def motor_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENA, GPIO.OUT); GPIO.setup(IN1, GPIO.OUT); GPIO.setup(IN2, GPIO.OUT)
    pwm = GPIO.PWM(ENA, PWM_FREQ); pwm.start(0)
    GPIO.output(IN1, GPIO.HIGH); GPIO.output(IN2, GPIO.LOW)  # forward
    return pwm

def motor_cleanup(pwm):
    try: pwm.ChangeDutyCycle(0); pwm.stop()
    except Exception: pass
    try: GPIO.output(IN1, GPIO.LOW); GPIO.output(IN2, GPIO.LOW)
    except Exception: pass
    GPIO.cleanup()

# ---- LEDs & Button
ledR = LED(17); ledG = LED(27); ledY = LED(23)
button = Button(18, hold_time=3)

# ---- Heart sensor
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)
monitoring_enabled = True
abnormal_counter = 0
emergency_mode = False
warning_intervals = [10, 20]
warning_issued = set()

_last_flash, _flash_period = 0.5, 0.5
def emergency_flash_tick():
    # Only for *internal* emergency_mode (not simulate)
    global _last_flash
    now = time.time()
    if now - _last_flash >= _flash_period:
        ledR.toggle(); ledG.toggle(); ledY.toggle()
        _last_flash = now

def open_maps_for_hospitals():
    lat, lon = 53.2772, -9.0106
    try: webbrowser.open(f"https://www.google.com/maps/search/hospital/@{lat},{lon},15z")
    except Exception: pass

def toggle_monitoring():
    global monitoring_enabled
    if emergency_mode: return
    monitoring_enabled = not monitoring_enabled
    if not monitoring_enabled:
        ledY.on(); ledR.off(); ledG.off(); write_hr(0, "Disabled")
    else:
        ledY.off()

def reset_emergency():
    global emergency_mode, monitoring_enabled, abnormal_counter
    emergency_mode = False; monitoring_enabled = True
    abnormal_counter = 0; warning_issued.clear()
    ledR.off(); ledG.off(); ledY.off()

button.when_pressed = toggle_monitoring
button.when_held    = reset_emergency

def main():
    global monitoring_enabled, abnormal_counter, emergency_mode

    # ---- Motor
    pwm = motor_setup()
    cur_duty, target_duty = 100, NORMAL_TARGET    # start car running
    pwm.ChangeDutyCycle(cur_duty)                  # IMPORTANT: actually apply 100% now
    write_speed(cur_duty, hazard=False)

    # ---- Sensor
    sensor_ok = sensor.begin()
    if sensor_ok:
        try: sensor.sensor_start_collect(); print("Sensor connected")
        except Exception: print("Sensor collect start issue; continuing.")
    else:
        print("Sensor not found")

    last_motor_tick = 0.0
    last_sensor = 0.0
    last_display_bpm = 90
    hazard_hr = last_display_bpm
    HAZARD_HR_TARGET = 160
    HAZARD_HR_STEP   = 5

    try:
        while True:
            now = time.time()

            # External hazard (simulate) OR internal emergency
            ext_hazard = read_external_hazard()
            hazard_active = ext_hazard or emergency_mode

            # ---- Motor ramp + write speed
            target_duty = HAZARD_TARGET if hazard_active else NORMAL_TARGET
            if now - last_motor_tick >= RAMP_INTERVAL:
                if cur_duty < target_duty: cur_duty = min(cur_duty + RAMP_STEP, target_duty)
                elif cur_duty > target_duty: cur_duty = max(cur_duty - RAMP_STEP, target_duty)
                try: pwm.ChangeDutyCycle(cur_duty)
                except Exception: pass
                last_motor_tick = now
                write_speed(cur_duty, hazard=hazard_active)

            # Flash LEDs for *internal* emergency only (simulate keeps them steady)
            if emergency_mode:
                emergency_flash_tick()

            # ---- Heart rate read/publish (~1 Hz)
            if now - last_sensor >= 1.0:
                last_sensor = now

                # Read raw sensor if monitoring + sensor ok
                Bpm = -1
                if monitoring_enabled and sensor_ok:
                    try:
                        sensor.get_heartbeat_SPO2()
                        Bpm = sensor.heartbeat
                    except Exception:
                        Bpm = -1

                # Decide what to display
                if hazard_active:
                    # Simulate heart attack: spike HR upwards and mark Abnormal
                    base = Bpm if (Bpm != -1 and Bpm > 0) else last_display_bpm
                    hazard_hr = min(HAZARD_HR_TARGET, max(base, hazard_hr + HAZARD_HR_STEP))
                    display_bpm = int(hazard_hr)
                    display_status = "Abnormal"
                else:
                    # Normal: use sensor (or hold steady if no read)
                    if Bpm == -1:
                        display_bpm = max(55, min(110, int(last_display_bpm + random.randint(-2,2))))
                        display_status = "NoRead"
                    else:
                        display_bpm = int(Bpm)
                        display_status = "Normal" if display_bpm < 100 else "Abnormal"
                        hazard_hr = display_bpm  # reset baseline for next simulate/emergency

                write_hr(display_bpm, display_status)
                last_display_bpm = display_bpm

                # Internal emergency trigger (only when NOT in external simulate)
                if not ext_hazard and monitoring_enabled and sensor_ok and Bpm != -1:
                    if Bpm < 35 or Bpm > 400:
                        abnormal_counter += 1
                        ledR.on(); ledG.off()
                        for w in warning_intervals:
                            if abnormal_counter >= w and w not in warning_issued:
                                print(f"⚠️ WARNING: Abnormal heart rate detected for {w} seconds ⚠️")
                                warning_issued.add(w)
                        if abnormal_counter >= 30:
                            print("⚠️ EMERGENCY: Sustained abnormal heart rate! ⚠️")
                            emergency_mode = True
                            monitoring_enabled = False
                            open_maps_for_hospitals()
                    else:
                        ledR.off(); ledG.on()
                        abnormal_counter = 0
                        warning_issued.clear()
                elif not monitoring_enabled:
                    ledY.on(); ledR.off(); ledG.off()
                    abnormal_counter = 0; warning_issued.clear()

            time.sleep(0.02)

    finally:
        try: write_hr(0, "Stopped"); write_speed(0, hazard=False)
        except Exception: pass
        motor_cleanup(pwm)
        ledR.off(); ledG.off(); ledY.off()

if __name__ == "__main__":
    def _quit(signum, frame): sys.exit(0)
    signal.signal(signal.SIGINT, _quit)
    signal.signal(signal.SIGTERM, _quit)
    main()
