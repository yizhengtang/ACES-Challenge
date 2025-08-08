import cv2
import dlib
import numpy as np
from scipy.spatial import distance
import time

# ---------- Paths ----------
PREDICTOR_PATH = "/home/aces2025/shape_predictor_68_face_landmarks.dat"  # <- adjust if you moved it

# ---------- Load models ----------
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor(PREDICTOR_PATH)

# ---------- Utility functions ----------
def eye_aspect_ratio(eye_pts):
    A = distance.euclidean(eye_pts[1], eye_pts[5])
    B = distance.euclidean(eye_pts[2], eye_pts[4])
    C = distance.euclidean(eye_pts[0], eye_pts[3])
    return (A + B) / (2.0 * C)

def lip_distance(shape_np):
    # top lip pts: 50-52 + 61-63 ; bottom lip pts: 56-58 + 65-67
    top = np.concatenate((shape_np[50:53], shape_np[61:64]), axis=0)
    bottom = np.concatenate((shape_np[56:59], shape_np[65:68]), axis=0)
    return abs(np.mean(top[:, 1]) - np.mean(bottom[:, 1]))

# Indices for eyes
LEFT_EYE = list(range(42, 48))
RIGHT_EYE = list(range(36, 42))

# 3D model points for head pose (nose, chin, eye corners, mouth corners)
model_points = np.array([
    (0.0,   0.0,    0.0),    # Nose tip
    (0.0,  -330.0, -65.0),   # Chin
    (-225.0, 170.0, -135.0), # Left eye left corner
    (225.0, 170.0, -135.0),  # Right eye right corner
    (-150.0,-150.0, -125.0), # Left mouth corner
    (150.0,-150.0, -125.0)   # Right mouth corner
], dtype="double")

# ---------- Tunables (start here if you need to tweak) ----------
EAR_THRESH = 0.28         # eye closed if below this
DROWSY_FRAMES = 3         # frames below threshold before alert (use 1 for instant)
YAWN_NORM_THRESH = 0.36   # normalized mouth open threshold (tune 0.30–0.45)
DISTRACT_YAW_DEG = 25     # yaw angle in degrees

# ---------- Camera setup (prefer Picamera2) ----------
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

# ---------- State ----------
closed_frames = 0

# ---------- Main loop ----------
try:
    while True:
        if use_picam2:
            frame = picam2.capture_array()
        else:
            ok, frame = cap.read()
            if not ok:
                print("❌ Failed to read frame.")
                break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects = detector(gray, 0)

        if len(rects) == 0:
            cv2.putText(frame, "FACE NOT DETECTED", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            for rect in rects:
                shape = predictor(gray, rect)
                shape_np = np.array([(shape.part(i).x, shape.part(i).y) for i in range(68)])

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

                # Draw eye hulls for feedback
                cv2.drawContours(frame, [cv2.convexHull(left_eye)],  -1, (0, 255, 0), 1)
                cv2.drawContours(frame, [cv2.convexHull(right_eye)], -1, (0, 255, 0), 1)

                # ---- Yawn (normalized by face size) ----
                mouth_open_px = lip_distance(shape_np)
                # normalize by nose-to-chin distance so threshold works at different scales
                face_scale = np.linalg.norm(shape_np[8] - shape_np[30]) + 1e-6
                mouth_norm = mouth_open_px / face_scale
                if mouth_norm > YAWN_NORM_THRESH:
                    cv2.putText(frame, "YAWNING ALERT", (10, 120),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # ---- Head pose / Distraction (yaw) ----
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

                ok, rvec, tvec = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok:
                    rmat, _ = cv2.Rodrigues(rvec)
                    proj = np.hstack((rmat, tvec))
                    _, _, _, _, _, _, euler = cv2.decomposeProjectionMatrix(proj)
                    yaw_deg = float(euler[1, 0])

                    if abs(yaw_deg) > DISTRACT_YAW_DEG:
                        cv2.putText(frame, "DISTRACTION ALERT!", (10, 90),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                # Debug overlays
                cv2.putText(frame, f"EAR:{ear:.2f}", (10, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(frame, f"MOUTH:{mouth_norm:.2f}", (10, 175),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        cv2.imshow("Driver Monitor (Drowsy + Yawn + Distraction)", frame)
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
