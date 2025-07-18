# This code calculates the average HR of the person who is in contact with
# the monitor. It then raises an alarm if sudden changes of 15bpm are noticed. 
# It constantly updates the average if the bpm are increasing/decreasin by 4bpm

from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c
import time

# Initialize sensor
sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)

baseline_bpm = []
valid_readings_needed = 10 # number of valid readings needed for baseline

def collect_baseline():
    print("\nCollecting baseline... Please stay still and keep contact with the sensor.")
    while len(baseline_bpm) < valid_readings_needed:
        sensor.get_heartbeat_SPO2()
        bpm = sensor.heartbeat
        if bpm != -1:
            print(f"Valid BPM: {bpm}")
            baseline_bpm.append(bpm)
        else:
            print("No BPM detected. Waiting...")
        time.sleep(1)

    avg = sum(baseline_bpm) / len(baseline_bpm) # Calculate average
    print(f"\nBaseline average: {int(avg)}")
    return int(avg)

def monitor_heart_rate(dynamic_threshold):
    print("\nStarting heart rate monitoring...")
    previous_bpm = dynamic_threshold
    
    while True:
        sensor.get_heartbeat_SPO2()
        bpm = sensor.heartbeat
        
        if bpm == -1:
            print("No BPM detected")
        else:
            print(f"BPM: {bpm}")
            BpmAlarm = abs(bpm - previous_bpm)
            
            if BpmAlarm >= 15: # if sudden change is greater than 15 bpm
                print("Sudden abnormal BPM change detected!")
            else:
                # If change is gradual (1-4 bpm), update the dynamic threshold
                if abs(bpm - dynamic_threshold) <= 4:
                    dynamic_threshold = bpm
                    print(f"Gradual change, updated threshold: {dynamic_threshold}")
                else:
                    print("BPM is within expected gradual range")
            previous_bpm = bpm

        time.sleep(1)

# Main
if sensor.begin():
    print("Sensor connected")
    sensor.sensor_start_collect()

    base = collect_baseline()
    monitor_heart_rate(base)
else:
    print("Sensor not found")
