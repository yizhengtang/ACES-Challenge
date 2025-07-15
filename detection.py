import cv2
import dlib
import numpy as np
from scipy.spatial import distance

# Load dlib face detector and predictor
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Define eye aspect ratio function
def eye_aspect_ratio(eye):
    A = distance.euclidean(eye[1], eye[5])
    B = distance.euclidean(eye[2], eye[4])
    C = distance.euclidean(eye[0], eye[3])
    return (A + B) / (2.0 * C)

# Eye landmark indices
LEFT_EYE = list(range(42, 48))
RIGHT_EYE = list(range(36, 42))

# 3D model points of face (nose, eyes, chin, etc.)
model_points = np.array([
    (0.0, 0.0, 0.0),             # Nose tip
    (0.0, -330.0, -65.0),        # Chin
    (-225.0, 170.0, -135.0),     # Left eye left corner
    (225.0, 170.0, -135.0),      # Right eye right corner
    (-150.0, -150.0, -125.0),    # Left mouth corner
    (150.0, -150.0, -125.0)      # Right mouth corner
])

# Start camera
cap = cv2.VideoCapture(0)

COUNTER = 0
EAR_THRESH = 0.25
EAR_FRAMES = 20

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = detector(gray)

    if len(faces) == 0:
        cv2.putText(frame, "FACE NOT DETECTED", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    for face in faces:
        shape = predictor(gray, face)
        shape = np.array([(shape.part(i).x, shape.part(i).y) for i in range(68)])

        # Eye detection
        leftEye = shape[LEFT_EYE]
        rightEye = shape[RIGHT_EYE]
        leftEAR = eye_aspect_ratio(leftEye)
        rightEAR = eye_aspect_ratio(rightEye)
        ear = (leftEAR + rightEAR) / 2.0

        if ear < EAR_THRESH:
            COUNTER += 1
            if COUNTER >= EAR_FRAMES:
                cv2.putText(frame, "DROWSINESS ALERT", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            COUNTER = 0

        # Head pose estimation
        image_points = np.array([
            shape[30],     # Nose tip
            shape[8],      # Chin
            shape[36],     # Left eye left corner
            shape[45],     # Right eye right corner
            shape[48],     # Left mouth corner
            shape[54]      # Right mouth corner
        ], dtype="double")

        # Camera internals
        focal_length = frame.shape[1]
        center = (frame.shape[1]/2, frame.shape[0]/2)
        camera_matrix = np.array(
            [[focal_length, 0, center[0]],
             [0, focal_length, center[1]],
             [0, 0, 1]], dtype="double"
        )
        dist_coeffs = np.zeros((4, 1))  # Assuming no lens distortion

        success, rotation_vector, translation_vector = cv2.solvePnP(
            model_points, image_points, camera_matrix, dist_coeffs
        )

        # Get rotation matrix to estimate yaw
        rmat, _ = cv2.Rodrigues(rotation_vector)
        proj_matrix = np.hstack((rmat, translation_vector))
        _, _, _, _, _, _, eulerAngles = cv2.decomposeProjectionMatrix(proj_matrix)
        yaw = eulerAngles[1, 0]

        # Detect if head is turned
        if abs(yaw) > 25:  # Threshold in degrees
            cv2.putText(frame, "DISTRACTION ALERT!", (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Visual feedback
        for (x, y) in image_points.astype("int"):
            cv2.circle(frame, (x, y), 3, (0, 255, 255), -1)

    cv2.imshow("Drowsiness + Distraction", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
