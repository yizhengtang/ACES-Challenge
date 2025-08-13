from flask import Flask, render_template, jsonify
from collections import deque
import json, os, time, random, tempfile

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATE_DIR = os.path.join(BASE_DIR, "templates")
STATIC_DIR   = os.path.join(BASE_DIR, "static")
app = Flask(__name__, template_folder=TEMPLATE_DIR, static_folder=STATIC_DIR)

# Files produced/consumed by Main.py
HR_FILE        = "/tmp/hr.json"        # {"bpm":int,"status":str,"ts":float}
SPEED_FILE     = "/tmp/speed.json"     # {"duty":0-100,"hazard":bool,"ts":float}
MOTOR_CMD_FILE = "/tmp/motor_cmd.json" # {"hazard":bool,"ts":float}
DRIVER_STATUS_FILE = "/tmp/driver_status.json"

def write_motor_cmd(hazard: bool):
    payload = {"hazard": bool(hazard), "ts": time.time()}
    d = os.path.dirname(MOTOR_CMD_FILE) or "."
    with tempfile.NamedTemporaryFile("w", dir=d, delete=False) as tf:
        json.dump(payload, tf); tmp = tf.name
    os.replace(tmp, MOTOR_CMD_FILE)

trend = deque(maxlen=30)
_last_bpm = 75

def read_hr():
    if os.path.exists(HR_FILE):
        try:
            with open(HR_FILE, "r") as f: data = json.load(f)
            if time.time() - float(data.get("ts", 0)) <= 5.0:
                return int(data.get("bpm", 0)), str(data.get("status", "Normal"))
        except Exception:
            pass
    # fallback if Main.py isn’t running
    global _last_bpm
    bpm = max(55, min(110, _last_bpm + random.randint(-2, 2)))
    _last_bpm = bpm
    return bpm, ("Normal" if bpm < 100 else "Abnormal")

def read_speed():
    if os.path.exists(SPEED_FILE):
        try:
            with open(SPEED_FILE, "r") as f: data = json.load(f)
            if time.time() - float(data.get("ts", 0)) <= 3.0:
                return int(data.get("duty", 0)), bool(data.get("hazard", False))
        except Exception:
            pass
    return 100, False

def read_driver_status():
    if not os.path.exists(DRIVER_STATUS_FILE):
        return {"drowsiness": False, "yawning": False, "distraction": False}
    try:
        with open(DRIVER_STATUS_FILE, "r") as f: data = json.load(f)
        if time.time() - float(data.get("ts", 0)) > 5.0:
            return {"drowsiness": False, "yawning": False, "distraction": False}
        return {
            "drowsiness": bool(data.get("drowsiness", False)),
            "yawning":    bool(data.get("yawning", False)),
            "distraction":bool(data.get("distraction", False)),
        }
    except Exception:
        return {"drowsiness": False, "yawning": False, "distraction": False}

@app.route("/")
def home():
    return render_template("index.html")

@app.route("/heart_rate")
def heart_rate():
    bpm, status = read_hr()
    trend.append(bpm)
    return jsonify({"bpm": bpm, "status": status, "series": list(trend)})

@app.route("/car_status")
def car_status():
    duty, hazard = read_speed()
    return jsonify({"speed": duty, "hazard": hazard})

@app.route("/driver_status")
def driver_status():
    return jsonify(read_driver_status())

@app.route("/simulate_heart_attack")
def simulate_heart_attack():
    write_motor_cmd(True)
    return jsonify({"message": "Emergency! Hazard ON, slowing down."})

@app.route("/reset_heart_attack")
def reset_heart_attack():
    write_motor_cmd(False)
    return jsonify({"message": "Reset done. Back to normal."})

@app.route("/emergency_location")
def emergency_location():
    origin = {"lat": 53.2772, "lon": -9.0106}
    hospital = {
        "name": "University Hospital Galway",
        "address": "Newcastle Rd, Galway, H91 YR71, Ireland",
        "phone": "+353 91 524 222",
        "lat": 53.2769, "lon": -9.0699
    }
    distance_km, eta_min = 3.2, 8
    maps_url = (
        "https://www.google.com/maps/dir/?api=1"
        f"&origin={origin['lat']},{origin['lon']}"
        f"&destination={hospital['lat']},{hospital['lon']}"
        "&travelmode=driving"
    )
    return jsonify({"sent": True, "origin": origin, "hospital": hospital,
                    "distance_km": distance_km, "eta_min": eta_min, "maps_url": maps_url})

if __name__ == "__main__":
    print("Expecting template at:", os.path.join(TEMPLATE_DIR, "index.html"))
    print("Exists?  ", os.path.exists(os.path.join(TEMPLATE_DIR, "index.html")))
    print("Static dir:", STATIC_DIR, "Exists?", os.path.exists(STATIC_DIR))
    app.run(host="0.0.0.0", port=5000, debug=True)
