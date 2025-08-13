# Red LED signals abnormal bpm
# Green LED signals healthy bpm
# Yellow LED signals monitor disabled
# All LEDs flash when in emergency mode
# Maps opens when emergency mode is triggered
# Push button: short press toggles monitoring; long-press (3s) resets emergency
# Change test thresholds here: elif Bpm < 35 or Bpm > 400:

from gpiozero import LED, Button
from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c
import webbrowser
import json, time, os

lowerthreshold = 35
upperthreshold = 400


# -------- Dashboard handoff (app.py reads this) --------
HR_FILE = "/tmp/hr.json"

def write_hr(bpm, status="Normal"):
    try:
        with open(HR_FILE, "w") as f:
            json.dump({"bpm": int(bpm), "status": status, "ts": time.time()}, f)
    except Exception:
        pass

# -------- LEDs --------
ledR = LED(17)  # Red
ledG = LED(27)  # Green
ledY = LED(23)  # Yellow

# -------- Push Button --------
# Short press toggles monitoring; long press (3s) resets emergency
button = Button(18, hold_time=3)

# -------- Heart Rate Monitor --------
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)

monitoring_enabled = True
abnormal_counter = 0
emergency_mode = False
warning_intervals = [10, 20]  # Seconds to warn before emergency mode
warning_issued = set()

# for non-blocking LED flashing in emergency mode
_last_flash = 0.0
_flash_period = 0.5  # seconds

def open_maps_for_hospitals():
    # Coordinates for ATU Galway (replace if needed)
    lat = 53.2772
    lon = -9.0106
    url = f"https://www.google.com/maps/search/hospital/@{lat},{lon},15z"
    try:
        webbrowser.open(url)
        print("Opening Google Maps to search for nearby hospitals...")
    except Exception as e:
        print(f"Could not open browser: {e}")

def toggle_monitoring():
    global monitoring_enabled, abnormal_counter, emergency_mode
    if emergency_mode:
        # ignore short press while in emergency (use long-press to reset)
        return
    monitoring_enabled = not monitoring_enabled
    print("Monitoring Enabled" if monitoring_enabled else "Monitoring Disabled")
    if not monitoring_enabled:
        ledY.on(); ledR.off(); ledG.off()
        write_hr(0, "Disabled")  # reflect state in dashboard
    else:
        ledY.off()

def reset_emergency():
    """Long-press to clear emergency and resume monitoring."""
    global emergency_mode, monitoring_enabled, abnormal_counter
    if emergency_mode:
        print("Emergency reset via long-press.")
    emergency_mode = False
    monitoring_enabled = True
    abnormal_counter = 0
    warning_issued.clear()
    ledR.off(); ledG.off(); ledY.off()
    # the next loop will set LEDs correctly based on BPM

# wire up button events
button.when_pressed = toggle_monitoring
button.when_held = reset_emergency

def emergency_flash_tick():
    """Non-blocking: toggle LEDs at a fixed period while in emergency."""
    global _last_flash
    now = time.time()
    if now - _last_flash >= _flash_period:
        ledR.toggle(); ledG.toggle(); ledY.toggle()
        _last_flash = now

if sensor.begin():
    print("Sensor connected")
    try:
        sensor.sensor_start_collect()
    except Exception:
        pass

    while True:
        if emergency_mode:
            # Non-blocking flash so other events still process
            emergency_flash_tick()
            # keep writing a status so dashboard knows we're in emergency
            write_hr(0, "Emergency")
            time.sleep(0.05)  # small sleep to reduce CPU, responsive to long-press
            continue

        if monitoring_enabled:
            # ---- Read sensor ----
            try:
                sensor.get_heartbeat_SPO2()
                Bpm = sensor.heartbeat
            except Exception as e:
                print(f"Sensor read error: {e}")
                Bpm = -1

            # ---- Publish status to dashboard ----
            if Bpm == -1:
                status = "NoRead"
            elif Bpm > lowerthreshold or Bpm < upperthreshold:
                status = "Normal"
            else:
                status = "Abnormal"
            write_hr(Bpm if Bpm != -1 else 0, status)

            # ---- LED & logic ----
            if Bpm == -1:
                print("No valid heart rate detected.")
                ledG.off(); ledR.off()

            elif Bpm < lowerthreshold or Bpm > upperthreshold:  # Testing thresholds
                abnormal_counter += 1
                print(f"Abnormal Heart Rate!: {Bpm} bpm")
                ledR.on(); ledG.off()

                # warnings at defined intervals
                for w in warning_intervals:
                    if abnormal_counter >= w and w not in warning_issued:
                        print(f"⚠️ WARNING: Abnormal heart rate detected for {w} seconds ⚠️\nIf false alarm, press button to reset")
                        warning_issued.add(w)

                # emergency after 30s sustained abnormal
                if abnormal_counter >= 30:
                    print("⚠️ EMERGENCY: Sustained abnormal heart rate! ⚠️")
                    emergency_mode = True
                    monitoring_enabled = False
                    open_maps_for_hospitals()
                    # on next loop we’ll enter flashing branch
                    continue

            else:
                print(f"Heart rate is normal: {Bpm} bpm")
                ledR.off(); ledG.on()
                abnormal_counter = 0
                warning_issued.clear()

        else:
            # Monitoring disabled
            print("Monitoring is disabled.")
            ledY.on(); ledR.off(); ledG.off()
            abnormal_counter = 0
            warning_issued.clear()
            write_hr(0, "Disabled")

        time.sleep(1)

else:
    print("Sensor not found")
    # keep dashboard informative even without sensor
    write_hr(0, "NoSensor")
