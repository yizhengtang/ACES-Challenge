import cv2
import dlib
import numpy as np
from scipy.spatial import distance
import time
from collections import deque

# ---------------- Paths ----------------
PREDICTOR_PATH = "/home/aces2025/shape_predictor_68_face_landmarks.dat"

# ---------------- Load models ----------------
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(PREDICTOR_PATH)

# ---------------- Utility functions ----------------
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

# ---------------- Landmark indices ----------------
LEFT_EYE = list(range(42, 48))
RIGHT_EYE = list(range(36, 42))

# 3D model points for head pose
model_points = np.array([
    (0.0,   0.0,    0.0),    # Nose tip
    (0.0,  -330.0, -65.0),   # Chin
    (-225.0, 170.0, -135.0), # Left eye left corner
    (225.0, 170.0, -135.0),  # Right eye right corner
    (-150.0,-150.0, -125.0), # Left mouth corner
    (150.0,-150.0, -125.0)   # Right mouth corner
], dtype="double")

# ---------------- Thresholds ----------------
EAR_THRESH = 0.23           # lower so eyes must close to trigger
DROWSY_FRAMES = 10          # frames before drowsy alert
YAWN_THRESH = 0.35          # normalized mouth opening threshold
DISTRACT_YAW_DEG = 25       # degrees for head turn

# ---------------- State ----------------
closed_frames = 0
yaw_history = deque(maxlen=5)  # smooth yaw

# ---------------- Camera setup ----------------
use_picam2 = False
frame_size = (640, 480)

try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    cfg = picam2.create_preview_configuration(main={"format": "BGR888", "size": frame_size})
    picam2.configure(cfg)
    picam2.start()
    time.sleep(1.0)
    use_picam2 = True
except Exception as e:
    print(f"[INFO] Picamera2 not used ({e}). Falling back to OpenCV VideoCapture.")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_size[0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_size[1])
    if not cap.isOpened():
        print("❌ Failed to open camera.")
        raise SystemExit

# ---------------- Main loop ----------------
try:
    while True:
        if use_picam2:
            frame = picam2.capture_array()
        else:
            ok, frame = cap.read()
            if not ok:
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = detector(gray, 0)

        if len(rects) == 0:
            cv2.putText(frame, "FACE NOT DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        for rect in rects:
            shape = predictor(gray, rect)
            shape_np = np.array([(shape.part(i).x, shape.part(i).y) for i in range(68)])

            # Draw all 68 landmarks
            for (x, y) in shape_np:
                cv2.circle(frame, (x, y), 1, (0, 255, 255), -1)

            # ---- Eyes / Drowsiness ----
            left_eye = shape_np[LEFT_EYE]
            right_eye = shape_np[RIGHT_EYE]
            ear = (eye_aspect_ratio(left_eye) + eye_aspect_ratio(right_eye)) / 2.0

            if ear < EAR_THRESH:
                closed_frames += 1
            else:
                closed_frames = 0

            if closed_frames >= DROWSY_FRAMES:
                cv2.putText(frame, "DROWSINESS ALERT", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Eye outline
            cv2.polylines(frame, [left_eye], True, (0, 255, 0), 1)
            cv2.polylines(frame, [right_eye], True, (0, 255, 0), 1)

            # ---- Yawn detection ----
            mouth_ratio = mouth_opening_ratio(shape_np)
            if mouth_ratio > YAWN_THRESH:
                cv2.putText(frame, "YAWNING ALERT", (10, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            # ---- Head pose ----
            image_points = np.array([
                shape_np[30],  # Nose tip
                shape_np[8],   # Chin
                shape_np[36],  # Left eye left corner
                shape_np[45],  # Right eye right corner
                shape_np[48],  # Left mouth corner
                shape_np[54]   # Right mouth corner
            ], dtype="double")

            focal_length = frame.shape[1]
            center = (frame.shape[1] / 2.0, frame.shape[0] / 2.0)
            camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype="double")
            dist_coeffs = np.zeros((4, 1))

            ok, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs)
            if ok:
                rmat, _ = cv2.Rodrigues(rvec)
                proj = np.hstack((rmat, tvec))
                _, _, _, _, _, _, euler = cv2.decomposeProjectionMatrix(proj)
                yaw_deg = float(euler[1, 0])

                # smooth yaw
                yaw_history.append(yaw_deg)
                yaw_avg = np.mean(yaw_history)

                if abs(yaw_avg) > DISTRACT_YAW_DEG:
                    cv2.putText(frame, "DISTRACTION ALERT!", (10, 90),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Driver Monitor", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    if use_picam2:
        try:
            picam2.stop()
        except Exception:
            pass
    else:
        cap.release()
    cv2.destroyAllWindows()
