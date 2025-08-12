import cv2
import dlib
import numpy as np
from scipy.spatial import distance
import time
from collections import deque
import csv
import os
from datetime import datetime
import pygame  # sounds

# ---------- Speed knobs ----------
cv2.setUseOptimized(True)
cv2.setNumThreads(2)            # 2–3 is good on Pi 5
PROCESS_SIZE = (640, 480)       # try (512, 384) if you still see lag

DETECT_EVERY = 12               # run heavy DNN every N frames when tracker is good
REDETECT_ON_BAD_QUALITY = 6.0   # if tracker quality < this, force re-detect

# ---------- Paths ----------
PREDICTOR_PATH = "/home/aces2025/ProjectMain/ACES-Challenge/shape_predictor_68_face_landmarks.dat"
LOG_FILE       = "/home/aces2025/ProjectMain/ACES-Challenge/driver_alerts.csv"
DROWSY_SOUND   = "/home/aces2025/ProjectMain/ACES-Challenge/drowsy.wav"
YAWN_SOUND     = "/home/aces2025/ProjectMain/ACES-Challenge/yawn.wav"
DISTRACT_SOUND = "/home/aces2025/ProjectMain/ACES-Challenge/distraction.wav"

DNN_PROTO = "/home/aces2025/ProjectMain/ACES-Challenge/deploy.prototxt"
DNN_MODEL = "/home/aces2025/ProjectMain/ACES-Challenge/res10_300x300_ssd_iter_140000.caffemodel"

# ---------- Models ----------
detector_hog = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(PREDICTOR_PATH)

dnn_net = None
try:
    dnn_net = cv2.dnn.readNetFromCaffe(DNN_PROTO, DNN_MODEL)
    dnn_net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    dnn_net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    print("OpenCV DNN face detector loaded.")
except Exception as e:
    print(f"[INFO] Could not load DNN face detector: {e}")

# ---------- Utils ----------
LEFT_EYE  = list(range(42, 48))
RIGHT_EYE = list(range(36, 42))

model_points = np.array([
    (0.0,   0.0,    0.0),
    (0.0,  -330.0, -65.0),
    (-225.0, 170.0, -135.0),
    (225.0, 170.0, -135.0),
    (-150.0,-150.0, -125.0),
    (150.0,-150.0, -125.0)
], dtype="double")

def eye_aspect_ratio(eye_pts):
    A = distance.euclidean(eye_pts[1], eye_pts[5])
    B = distance.euclidean(eye_pts[2], eye_pts[4])
    C = distance.euclidean(eye_pts[0], eye_pts[3])
    return (A + B) / (2.0 * C)

def mouth_opening_ratio(shape_np):
    top = np.concatenate((shape_np[50:53], shape_np[61:64]), axis=0)
    bottom = np.concatenate((shape_np[56:59], shape_np[65:68]), axis=0)
    mouth_open_px = abs(np.mean(top[:, 1]) - np.mean(bottom[:, 1]))
    face_scale = np.linalg.norm(shape_np[8] - shape_np[30]) + 1e-6
    return mouth_open_px / face_scale

def normalize_gray(gray):
    # CLAHE only used for HOG fallback; not shown on screen
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    return clahe.apply(gray)

def log_event(alert_type, value):
    with open(LOG_FILE, mode="a", newline="") as f:
        csv.writer(f).writerow([datetime.now().strftime("%Y-%m-%d %H:%M:%S"), alert_type, f"{value:.3f}"])

# ---------- Sounds ----------
def play_sound(path, rate_limit_ms=2000):
    now = int(time.time() * 1000)
    last = sound_last.get(path, 0)
    if now - last < rate_limit_ms:
        return
    try:
        if pygame.mixer.get_init() is not None:
            pygame.mixer.music.load(path)
            pygame.mixer.music.play()
            sound_last[path] = now
    except pygame.error:
        pass

# ---------- Alert manager (keeps messages visible) ----------
class AlertManager:
    def __init__(self, hold_ms=2500):
        self.hold_ms = hold_ms
        self.active = {}  # name -> (message, expiry_ms)

    def trigger(self, name, message):
        expiry = int(time.time() * 1000) + self.hold_ms
        self.active[name] = (message, expiry)

    def draw(self, frame):
        # purge expired, draw remaining stacked
        now = int(time.time() * 1000)
        to_show = []
        for k, (msg, exp) in list(self.active.items()):
            if now > exp:
                del self.active[k]
            else:
                to_show.append(msg)

        y = 60
        for msg in to_show[:3]:  # show up to 3
            (w, h), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            cv2.rectangle(frame, (8, y - h - 10), (12 + w, y + 8), (0, 0, 0), -1)
            cv2.putText(frame, msg, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            y += h + 18

# ---------- DNN + HOG detector ----------
def detect_faces_dnn(frame_bgr):
    if dnn_net is None:
        return []
    h, w = frame_bgr.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame_bgr, (300, 300)),
                                 1.0, (300, 300), (104.0, 177.0, 123.0),
                                 swapRB=False, crop=False)
    dnn_net.setInput(blob)
    detections = dnn_net.forward()
    rects = []
    for i in range(detections.shape[2]):
        conf = float(detections[0, 0, i, 2])
        if conf < 0.45:
            continue
        x1, y1, x2, y2 = (detections[0, 0, i, 3:7] * np.array([w, h, w, h])).astype(int)
        x1 = max(0, x1); y1 = max(0, y1); x2 = min(w - 1, x2); y2 = min(h - 1, y2)
        pad = int(0.06 * max(x2 - x1, y2 - y1))
        rects.append(dlib.rectangle(max(0, x1 - pad), max(0, y1 - pad),
                                    min(w - 1, x2 + pad), min(h - 1, y2 + pad)))
    return rects

def detect_faces(gray, frame_bgr):
    rects = detect_faces_dnn(frame_bgr)
    if not rects:
        rects = detector_hog(normalize_gray(gray), 0)
    return rects

# ---------- Pose ----------
def compute_yaw(shape_np, frame):
    image_points = np.array([
        shape_np[30], shape_np[8], shape_np[36], shape_np[45], shape_np[48], shape_np[54]
    ], dtype="double")
    focal_length = frame.shape[1]
    center = (frame.shape[1]/2.0, frame.shape[0]/2.0)
    camera_matrix = np.array([[focal_length, 0, center[0]],
                              [0, focal_length, center[1]],
                              [0, 0, 1]], dtype="double")
    dist_coeffs = np.zeros((4,1))
    ok, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
    if not ok:
        return None
    rmat, _ = cv2.Rodrigues(rvec)
    proj = np.hstack((rmat, tvec))
    _, _, _, _, _, _, euler = cv2.decomposeProjectionMatrix(proj)
    return float(euler[1,0])

# ---------- CSV header ----------
if not os.path.exists(LOG_FILE):
    with open(LOG_FILE, mode="w", newline="") as f:
        csv.writer(f).writerow(["Timestamp", "Alert Type", "Value"])

# ---------- Camera ----------
use_picam2 = False
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(
        main={"format": "BGR888", "size": PROCESS_SIZE},
        buffer_count=2
    )
    picam2.configure(cfg)
    try:
        picam2.set_controls({"FrameRate": 30})
    except Exception:
        pass
    picam2.start()
    time.sleep(0.4)
    use_picam2 = True
except Exception as e:
    print(f"[INFO] Picamera2 not used ({e}). Falling back to OpenCV VideoCapture.")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, PROCESS_SIZE[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, PROCESS_SIZE[1])
    if not cap.isOpened():
        print("❌ Failed to open camera.")
        raise SystemExit

# ---------- Pygame ----------
sound_last = {}
try:
    pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=512)
    print("Pygame mixer initialized.")
except pygame.error as e:
    print(f"Audio init error: {e}")

# ---------- States ----------
yaw_history = deque(maxlen=9)
ear_ema = None
EMA_ALPHA = 0.25               # EAR smoothing (0.25 works well)
closed_frames = 0
alert_mgr = AlertManager(hold_ms=2500)  # keep alerts visible

# Calibration
calib_time_sec = 7
calib_start = time.time()
calib_ears, calib_mouths, calib_yaws = [], [], []
ear_thresh = None
yawn_thresh = None
yaw_center = 0.0
yaw_thresh_deg = 25.0

# PERCLOS (eye closure percentage over window)
PERCLOS_WIN_SEC = 3.0
perclos_events = deque()  # (ts_ms, closed_bool)

# Tracker
tracker = dlib.correlation_tracker()
tracker_active = False
last_rect = None
last_seen_ts_ms = 0
grace_ms = 700

# Draw toggles
draw_landmarks = False
draw_boxes = True
clean_view = False

frame_idx = 0

# ---------- Main loop ----------
try:
    while True:
        if use_picam2:
            frame = picam2.capture_array()
        else:
            ok, frame = cap.read()
            if not ok: break

        frame_idx += 1
        if frame.shape[1] != PROCESS_SIZE[0] or frame.shape[0] != PROCESS_SIZE[1]:
            frame = cv2.resize(frame, PROCESS_SIZE, interpolation=cv2.INTER_AREA)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        now_ms = int(time.time() * 1000)

        # --- Detection + tracking strategy ---
        need_heavy = False
        if not tracker_active or (frame_idx % DETECT_EVERY == 0):
            need_heavy = True

        rect = None
        if tracker_active and not need_heavy:
            quality = tracker.update(frame)
            if quality >= REDETECT_ON_BAD_QUALITY:
                pos = tracker.get_position()
                rect = dlib.rectangle(int(pos.left()), int(pos.top()), int(pos.right()), int(pos.bottom()))
            else:
                tracker_active = False
                need_heavy = True

        if need_heavy:
            rects = detect_faces(gray, frame)
            if rects:
                rect = rects[0]
                tracker.start_track(frame, dlib.rectangle(rect.left(), rect.top(), rect.right(), rect.bottom()))
                tracker_active = True
            else:
                rect = None

        # grace window if detection blips
        if rect is not None:
            last_rect = rect
            last_seen_ts_ms = now_ms
        else:
            if last_rect and (now_ms - last_seen_ts_ms) < grace_ms:
                rect = last_rect

        # ----- Calibration banner -----
        if ear_thresh is None or yawn_thresh is None:
            cv2.putText(frame, "CALIBRATING... Look forward, eyes OPEN, mouth relaxed",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            elapsed = time.time() - calib_start
            bar_w = frame.shape[1] - 20
            cv2.rectangle(frame, (10, 40),
                          (10 + int(bar_w * (min(elapsed, calib_time_sec)/calib_time_sec)), 55),
                          (0, 255, 255), -1)

        if rect is None:
            cv2.putText(frame, "FACE NOT DETECTED", (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            alert_mgr.draw(frame)
            cv2.putText(frame, "Press 'v' clean view, 'l' landmarks, 'b' boxes, 'c' recalib, 'q' quit",
                        (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)
            cv2.imshow("Driver Monitor (DNN + Tracker, smooth)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            if key == ord('c'):
                calib_start = time.time()
                calib_ears.clear(); calib_mouths.clear(); calib_yaws.clear()
                ear_thresh = None; yawn_thresh = None; yaw_center = 0.0
            if key == ord('l'): draw_landmarks = not draw_landmarks
            if key == ord('b'): draw_boxes = not draw_boxes
            if key == ord('v'): clean_view = not clean_view
            continue

        # Optional box
        if draw_boxes and not clean_view:
            cv2.rectangle(frame, (rect.left(), rect.top()), (rect.right(), rect.bottom()), (255,255,0), 1)

        # Landmarks
        shape = predictor(gray, rect)
        shape_np = np.array([(shape.part(i).x, shape.part(i).y) for i in range(68)])

        if draw_landmarks and not clean_view:
            for (x, y) in shape_np:
                cv2.circle(frame, (x, y), 1, (0, 255, 255), -1)
            cv2.polylines(frame, [shape_np[LEFT_EYE]],  True, (0, 255, 0), 1)
            cv2.polylines(frame, [shape_np[RIGHT_EYE]], True, (0, 255, 0), 1)

        # Measures
        left_eye  = shape_np[LEFT_EYE]
        right_eye = shape_np[RIGHT_EYE]
        ear_raw = (eye_aspect_ratio(left_eye) + eye_aspect_ratio(right_eye)) / 2.0
        ear_ema = ear_raw if ear_ema is None else (EMA_ALPHA * ear_raw + (1-EMA_ALPHA) * ear_ema)
        mouth_ratio = mouth_opening_ratio(shape_np)
        yaw_deg = compute_yaw(shape_np, frame)
        if yaw_deg is not None:
            yaw_history.append(yaw_deg)
        yaw_avg = float(np.mean(yaw_history)) if yaw_history else 0.0

        # Show yaw readout (helpful to confirm left/right)
        if not clean_view:
            cv2.putText(frame, f"Yaw: {yaw_avg:+.1f} deg (center {yaw_center:+.1f})",
                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200,200,200), 1)

        # Calibration samples
        if ear_thresh is None or yawn_thresh is None:
            if (time.time() - calib_start) <= calib_time_sec:
                calib_ears.append(ear_ema)
                calib_mouths.append(mouth_ratio)
                if yaw_deg is not None:
                    calib_yaws.append(yaw_deg)
            else:
                if calib_ears and calib_mouths:
                    ear_mean, ear_std = np.mean(calib_ears), np.std(calib_ears)
                    mouth_mean, mouth_std = np.mean(calib_mouths), np.std(calib_mouths)
                    ear_thresh = min(ear_mean * 0.70, ear_mean - 2.0 * ear_std)
                    yawn_thresh = max(mouth_mean * 1.60, mouth_mean + 3.0 * mouth_std)
                    yaw_center = np.median(calib_yaws) if calib_yaws else 0.0

        # After calibration: detection logic
        if ear_thresh is not None and yawn_thresh is not None:
            # --- PERCLOS over last N seconds ---
            closed_now = 1 if ear_ema < ear_thresh else 0
            perclos_events.append((now_ms, closed_now))
            # drop old
            cutoff = now_ms - int(PERCLOS_WIN_SEC * 1000)
            while perclos_events and perclos_events[0][0] < cutoff:
                perclos_events.popleft()
            # percentage closed
            if perclos_events:
                perclos = sum(v for _, v in perclos_events) / len(perclos_events)
            else:
                perclos = 0.0

            # Keep consecutive frames logic as secondary
            if closed_now:
                closed_frames += 1
            else:
                closed_frames = 0

            # Drowsiness triggers:
            # A) sustained closure (frames)
            # B) high PERCLOS over window (more robust)
            if (closed_frames >= 5) or (perclos >= 0.40):
                alert_mgr.trigger("Drowsy", "DROWSINESS ALERT")
                log_event("Drowsy", float(ear_ema))
                play_sound(DROWSY_SOUND)
                # reset a bit so it won't spam
                closed_frames = 0
                perclos_events.clear()

            # Yawning
            if mouth_ratio > yawn_thresh:
                alert_mgr.trigger("Yawn", "YAWNING ALERT")
                log_event("Yawn", float(mouth_ratio))
                play_sound(YAWN_SOUND)

            # Distraction: yaw deviation
            deviation = (yaw_avg - yaw_center)
            abs_dev = abs(deviation)
            direction = "RIGHT" if deviation > 0 else "LEFT"
            if abs_dev > yaw_thresh_deg:
                alert_mgr.trigger("Distraction", f"DISTRACTION ALERT! ({direction})")
                log_event("Distraction", float(deviation))
                play_sound(DISTRACT_SOUND)

        # Footer + alerts
        alert_mgr.draw(frame)
        if not clean_view:
            cv2.putText(frame, "Press 'v' clean view, 'l' landmarks, 'b' boxes, 'c' recalib, 'q' quit",
                        (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

        cv2.imshow("Driver Monitor (DNN + Tracker, smooth)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('c'):
            calib_start = time.time()
            calib_ears.clear(); calib_mouths.clear(); calib_yaws.clear()
            ear_thresh = None; yawn_thresh = None; yaw_center = 0.0
        if key == ord('l'):
            draw_landmarks = not draw_landmarks
        if key == ord('b'):
            draw_boxes = not draw_boxes
        if key == ord('v'):
            clean_view = not clean_view

finally:
    try:
        if use_picam2:
            picam2.stop()
        else:
            cap.release()
    except Exception:
        pass
    cv2.destroyAllWindows()
