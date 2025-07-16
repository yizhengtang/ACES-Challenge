# Red LED signals abnormal bpm
# Green LED signals healthy bpm
# Yellow LED signal shr monitor detection is disabled
# The push button enables/disables the monitoring 

from gpiozero import LED, Button
import time
from time import sleep
from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c

# LEDs
ledR = LED(17)  # Red
ledG = LED(27)  # Green
ledY = LED(23)  # Yellow

# Push Button
button = Button(18)

# Heart Rate Monitor
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)

monitoring_enabled = True
abnormal_counter = 0
emergency_mode = False
warning_intervals = [10, 20]  # Seconds to warn before emergency mode
warning_issued = set()

def toggle_monitoring():
    global monitoring_enabled, abnormal_counter, emergency_mode
    if emergency_mode:
        return  # Ignore toggle during emergency
    monitoring_enabled = not monitoring_enabled
    print("Monitoring Enabled:" if monitoring_enabled else "Monitoring Disabled")
    if not monitoring_enabled:
        ledY.on()
        ledR.off()
        ledG.off()
    else:
        ledY.off()

# Set button to toggle monitoring only
button.when_pressed = toggle_monitoring

def flash_all_leds():
    ledG.off()
    ledR.off()
    ledY.off()
    while emergency_mode:
        ledR.toggle()
        ledG.toggle()
        ledY.toggle()
        time.sleep(0.5)

if sensor.begin():
    print("Sensor connected")
    sensor.sensor_start_collect()

    while True:
        if emergency_mode:
            flash_all_leds()
            continue

        if monitoring_enabled:
            sensor.get_heartbeat_SPO2()
            Bpm = sensor.heartbeat

            if Bpm == -1:
                print("No valid heart rate detected.")
                ledG.off()
                ledR.off()

            elif Bpm < 35 or Bpm > 40:  # Change threshold for testing
                abnormal_counter += 1
                print(f"Abnormal Heart Rate!: {Bpm} bpm")
                ledR.on()
                ledG.off()

                # Warnings every 10s
                for w in warning_intervals:
                    if abnormal_counter >= w and w not in warning_issued:
                        print(f"⚠️ WARNING: Abnormal heart rate detected for {w} seconds ⚠️\n If false alarm, press button to reset")
                        warning_issued.add(w)

                # Enter emergency mode after 30s
                if abnormal_counter >= 30:
                    # What to add:
                    # Motor slows down and stops
                    # Contact emergency services
                    # Shows nearest hospital/emergency room on webpage?
                    print("⚠️ EMERGENCY: Sustained abnormal heart rate! ⚠️")
                    emergency_mode = True # Flashes LEDs
                    monitoring_enabled = False
                    continue

            else:
                print(f"Heart rate is normal: {Bpm} bpm")
                ledR.off()
                ledG.on()
                abnormal_counter = 0
                warning_issued.clear()

        else:
            print("Monitoring is disabled.")
            abnormal_counter = 0
            warning_issued.clear()

        time.sleep(1)
else:
    print("Sensor not found")
