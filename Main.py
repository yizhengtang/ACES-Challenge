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

# Separate ramp profiles
RAMP_UP_STEP,        RAMP_UP_INTERVAL        = 4, 0.15   # gentler startup after reset
RAMP_DOWN_STEP_NORM, RAMP_DOWN_INTERVAL_NORM = 3, 0.24   # normal gentle slowing

# Asymptotic hazard slowdown (easing)
HAZARD_EASE_FACTOR     = 0.035  # ~3.5% of current duty per step (slower drop)
HAZARD_EASE_MIN_STEP   = 1      # never drop slower than 1%
HAZARD_EASE_INTERVAL   = 0.45   # slightly longer pause between eased steps

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
ledR = LED(17)  # Red
ledG = LED(27)  # Green
ledY = LED(23)  # Yellow
button = Button(18, hold_time=3)

# ---- Heart sensor
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)
monitoring_enabled = True
abnormal_counter = 0
emergency_mode = False
warning_intervals = [10, 20]
warning_issued = set()

# --- Internal emergency: flash all LEDs
_last_flash, _flash_period = 0.5, 0.5
def emergency_flash_tick():
    global _last_flash
    now = time.time()
    if now - _last_flash >= _flash_period:
        ledR.toggle(); ledG.toggle(); ledY.toggle()
        _last_flash = now

# --- Simulate-hazard (external) LED flicker: **RED** blink, faster
_hazard_flash_last = 0.0
_HAZARD_FLASH_PERIOD = 0.25  # seconds
def hazard_flash_tick():
    global _hazard_flash_last
    now = time.time()
    if now - _hazard_flash_last >= _HAZARD_FLASH_PERIOD:
        ledR.toggle()  # RED flicker
        _hazard_flash_last = now

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
        print("[MON] Monitoring disabled")
    else:
        ledY.off()
        print("[MON] Monitoring enabled")

def reset_emergency():
    global emergency_mode, monitoring_enabled, abnormal_counter
    emergency_mode = False; monitoring_enabled = True
    abnormal_counter = 0; warning_issued.clear()
    ledR.off(); ledG.off(); ledY.off()
    print("[EMERGENCY] Cleared via long-press; resuming live sensor.")

button.when_pressed = toggle_monitoring
button.when_held    = reset_emergency

def _ramp_to(pwm, start, target, step, interval, hazard=False):
    duty = start
    write_speed(duty, hazard=hazard)
    try: pwm.ChangeDutyCycle(duty)
    except Exception: pass
    while duty != target:
        time.sleep(interval)
        if target > duty:
            duty = min(target, duty + step)
        else:
            duty = max(target, duty - step)
        try: pwm.ChangeDutyCycle(duty)
        except Exception: pass
        write_speed(duty, hazard=hazard)
    return duty

def main():
    global monitoring_enabled, abnormal_counter, emergency_mode

    # ---- Motor
    pwm = motor_setup()
    # Start gently from 0% to 100%
    cur_duty = _ramp_to(pwm, 0, 100, RAMP_UP_STEP, RAMP_UP_INTERVAL, hazard=False)
    target_duty = NORMAL_TARGET
    print(f"[MOTOR] Ramped up to {cur_duty}% duty")
    write_speed(cur_duty, hazard=False)

    # ---- Sensor
    sensor_ok = sensor.begin()
    if sensor_ok:
        try:
            sensor.sensor_start_collect()
            print("[SENSOR] Connected and collecting")
        except Exception:
            print("[SENSOR] Collect start issue; continuing.")
    else:
        print("[SENSOR] Not found")

    last_motor_tick = 0.0
    last_sensor = 0.0
    last_display_bpm = 90
    hazard_hr = last_display_bpm
    HAZARD_HR_TARGET = 160
    HAZARD_HR_STEP   = 5

    prev_hazard_active = False  # for edge detection

    try:
        while True:
            now = time.time()

            # External hazard (simulate) OR internal emergency
            ext_hazard = read_external_hazard()
            hazard_active = ext_hazard or emergency_mode

            if hazard_active and not prev_hazard_active:
                print("[HAZARD] Activated (Simulate or Internal).")
            if not hazard_active and prev_hazard_active:
                print("[HAZARD] Cleared.")
                if monitoring_enabled:
                    ledR.off()
                # resume live HR cleanly after a simulated hazard
                monitoring_enabled = True
                emergency_mode = False
                abnormal_counter = 0
                warning_issued.clear()

            prev_hazard_active = hazard_active

            # ---- Motor ramp + write speed
            target_duty = HAZARD_TARGET if hazard_active else NORMAL_TARGET

            if cur_duty < target_duty:
                step = RAMP_UP_STEP
                interval = RAMP_UP_INTERVAL
                next_duty = min(cur_duty + step, target_duty)
            elif cur_duty > target_duty:
                if hazard_active:
                    step = max(HAZARD_EASE_MIN_STEP, int(round(cur_duty * HAZARD_EASE_FACTOR)))
                    interval = HAZARD_EASE_INTERVAL
                else:
                    step = RAMP_DOWN_STEP_NORM
                    interval = RAMP_DOWN_INTERVAL_NORM
                next_duty = max(cur_duty - step, target_duty)
            else:
                interval = None
                next_duty = cur_duty

            if interval is not None and (now - last_motor_tick) >= interval:
                cur_duty = next_duty
                try:
                    pwm.ChangeDutyCycle(cur_duty)
                except Exception:
                    pass
                last_motor_tick = now
                write_speed(cur_duty, hazard=hazard_active)

            # ---- LED policies
            if emergency_mode:
                emergency_flash_tick()
            elif ext_hazard:
                hazard_flash_tick()

            # ---- Heart rate read/publish (~1 Hz)
            if now - last_sensor >= 1.0:
                last_sensor = now

                Bpm = -1
                if monitoring_enabled and sensor_ok:
                    try:
                        sensor.get_heartbeat_SPO2()
                        Bpm = sensor.heartbeat
                    except Exception:
                        Bpm = -1

                if hazard_active:
                    base = Bpm if (Bpm != -1 and Bpm > 0) else last_display_bpm
                    hazard_hr = min(HAZARD_HR_TARGET, max(base, hazard_hr + HAZARD_HR_STEP))
                    display_bpm = int(hazard_hr)
                    display_status = "Abnormal"
                    print(f"[HR] Simulated hazard BPM={display_bpm} (status={display_status})")
                else:
                    if Bpm == -1:
                        display_bpm = max(55, min(110, int(last_display_bpm + random.randint(-2,2))))
                        display_status = "NoRead"
                        print(f"[HR] NoRead from sensor; showing fallback {display_bpm} bpm")
                    else:
                        display_bpm = int(Bpm)
                        display_status = "Normal" if display_bpm < 100 else "Abnormal"
                        hazard_hr = display_bpm
                        print(f"[HR] Sensor BPM={display_bpm} (status={display_status})")

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
