from flask import Flask, render_template, jsonify
from collections import deque
import json, os, time, random
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, "templates")
STATIC_DIR   = os.path.join(BASE_DIR, "static")

app = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)


# ---- Optional: direct sensor access (works even if Main.py isn't running) ----
try:
    from DFRobot_BloodOxygen_S import DFRobot_BloodOxygen_S_i2c
    _sensor = DFRobot_BloodOxygen_S_i2c(bus=1, addr=0x57)
    _sensor_ok = _sensor.begin()
    if _sensor_ok:
        try:
            _sensor.sensor_start_collect()
        except Exception:
            pass
    else:
        _sensor = None
except Exception:
    _sensor = None

# ---- Shared files written by Main.py and driver_monitor.py (see patches below) ----
HR_FILE = "/tmp/hr.json"                 # written by Main.py loop (bpm & status)
DRIVER_STATUS_FILE = "/tmp/driver_status.json"  # written by driver_monitor.py loop

# Keep a small trend for the chart when not reading a file
trend = deque(maxlen=30)
_last_bpm = 75

def read_bpm_from_file():
    """Read bpm written by Main.py (if available)."""
    if not os.path.exists(HR_FILE):
        return None
    try:
        with open(HR_FILE, "r") as f:
            data = json.load(f)
        # ignore stale data older than 5s
        if time.time() - float(data.get("ts", 0)) > 5.0:
            return None
        return int(data.get("bpm"))
    except Exception:
        return None

def read_bpm_direct():
    """Read bpm directly from sensor as fallback."""
    global _last_bpm
    if _sensor:
        try:
            _sensor.get_heartbeat_SPO2()
            bpm = _sensor.heartbeat
            if bpm == -1:
                bpm = max(45, min(120, _last_bpm + random.randint(-2, 2)))
        except Exception:
            bpm = max(55, min(110, _last_bpm + random.randint(-3, 3)))
    else:
        bpm = max(55, min(110, _last_bpm + random.randint(-3, 3)))
    _last_bpm = bpm
    return int(bpm)

def read_driver_status():
    """Read drowsy/yawn/distraction written by driver_monitor.py (if available)."""
    if not os.path.exists(DRIVER_STATUS_FILE):
        return {"drowsiness": False, "yawning": False, "distraction": False}
    try:
        with open(DRIVER_STATUS_FILE, "r") as f:
            data = json.load(f)
        if time.time() - float(data.get("ts", 0)) > 5.0:
            return {"drowsiness": False, "yawning": False, "distraction": False}
        return {
            "drowsiness": bool(data.get("drowsiness", False)),
            "yawning": bool(data.get("yawning", False)),
            "distraction": bool(data.get("distraction", False)),
        }
    except Exception:
        return {"drowsiness": False, "yawning": False, "distraction": False}

# Car state for simulate/reset
_speed = 100
_hazard = False

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/heart_rate")
def heart_rate():
    # Prefer bpm from Main.py file, fallback to direct sensor
    bpm = read_bpm_from_file()
    if bpm is None:
        bpm = read_bpm_direct()

    trend.append(bpm)
    status = "Normal" if bpm < 100 else "Abnormal"
    return jsonify({"bpm": bpm, "status": status, "series": list(trend)})

@app.route("/driver_status")
def driver_status():
    return jsonify(read_driver_status())

@app.route("/car_status")
def car_status():
    global _speed, _hazard
    if _hazard and _speed > 0:
        _speed = max(0, _speed - 3)
    return jsonify({"speed": _speed, "hazard": _hazard})

@app.route("/simulate_heart_attack")
def simulate_heart_attack():
    global _speed, _hazard
    _hazard = True
    _speed = min(_speed, 60)
    return jsonify({"message": "Emergency! Hazard ON, slowing down."})

@app.route("/reset_heart_attack")
def reset_heart_attack():
    global _speed, _hazard
    _hazard = False
    _speed = 100
    return jsonify({"message": "Reset done. Back to normal."})

@app.route("/emergency_location")
def emergency_location():
    # Dummy origin near ATU Galway (same area you used earlier)
    origin = {"lat": 53.2772, "lon": -9.0106}

    # Dummy “nearest hospital” info (placeholder values)
    hospital = {
        "name": "University Hospital Galway",
        "address": "Newcastle Rd, Galway, H91 YR71, Ireland",
        "phone": "+353 91 524 222",
        "lat": 53.2769,
        "lon": -9.0699
    }

    distance_km = 3.2   # dummy distance
    eta_min = 8         # dummy ETA

    maps_url = (
        "https://www.google.com/maps/dir/?api=1"
        f"&origin={origin['lat']},{origin['lon']}"
        f"&destination={hospital['lat']},{hospital['lon']}"
        "&travelmode=driving"
    )

    return jsonify({
        "sent": True,
        "origin": origin,
        "hospital": hospital,
        "distance_km": distance_km,
        "eta_min": eta_min,
        "maps_url": maps_url
    })


if __name__ == "__main__":
    print("Expecting template at:", os.path.join(TEMPLATE_DIR, "index.html"))
    print("Exists?  ", os.path.exists(os.path.join(TEMPLATE_DIR, "index.html")))
    print("Static dir:", STATIC_DIR, "Exists?", os.path.exists(STATIC_DIR))
    app.run(host="0.0.0.0", port=5000, debug=True)
