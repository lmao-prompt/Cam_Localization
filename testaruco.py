import cv2
import numpy as np

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
aruco_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

cap = cv2.VideoCapture(0)  # ganti index kalau pakai kamera USB lain

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        for i, corner in enumerate(corners):
            c = corner[0]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            cv2.putText(frame, f"ID:{ids[i]}", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"ID {ids[i]} center: ({cx},{cy})")

    cv2.imshow("ArUco Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()