# Red LED signals abnormal bpm
# Green LED signals healthy bpm
# Yellow LED signal shr monitor detection is disabled
# All LEDs flash when in emergency mode
# Maps opens when emergency mode is triggered
# The push button enables/disables the monitoring 

from gpiozero import LED, Button
import time
from time import sleep
from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c
import webbrowser # used for opening maps


# LEDs
ledR = LED(17)  # Red
ledG = LED(27)  # Green
ledY = LED(23)  # Yellow

# Push Button
button = Button(18)

# Heart Rate Monitor
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57) # The pi 5 has been set up to use i2c
                                                     # If using other pi look up how to set up in terminal, value may be different

monitoring_enabled = True
abnormal_counter = 0
emergency_mode = False
warning_intervals = [10, 20]  # Seconds to warn before emergency mode
warning_issued = set()

import webbrowser

def open_maps_for_hospitals():
    lat = 53.2772
    lon = -9.0106
    url = f"https://www.google.com/maps/search/hospital/@{lat},{lon},15z" # url for google maps to search for nearest hospitals
    webbrowser.open(url)
    print("Opening Google Maps to search for nearby hospitals...")

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
    # Turn all off first so they flash together
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
        #What to add
        #Turn Motor on, gradually speed up for x seconds until max speed is reached
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

            elif Bpm < 35 or Bpm > 400:  # Change threshold for testing
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
                    print("⚠️ EMERGENCY: Sustained abnormal heart rate! ⚠️")
                    emergency_mode = True # Flashes LEDs
                    monitoring_enabled = False
                    open_maps_for_hospitals()
                    continue

            else:
                print(f"Heart rate is normal: {Bpm} bpm")
                ledR.off()
                ledG.on()
                abnormal_counter = 0
                warning_issued.clear()

        else:
            print("Monitoring is disabled.")
            abnormal_counter = 0 # resets abnormal hr counter 
            warning_issued.clear()

        time.sleep(1)
else:
    print("Sensor not found")
