import sys
import cv2
import numpy as np


class MarkerKalman:
    def __init__(self, dt=1/30):
        self.dt = dt
        # State: [cx, cy, vx, vy]
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 500  # uncertainty awal gede, biar cepat "percaya" measurement pertama

        # Transisi state: posisi baru = posisi lama + v*dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Kita cuma ukur posisi (cx, cy), bukan velocity
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        # Process noise: seberapa "dipercaya" model constant-velocity
        q = 5.0
        self.Q = np.eye(4) * q

        # Measurement noise: seberapa noisy hasil detect ArUco (px)
        r = 4.0
        self.R = np.eye(2) * r

        self.initialized = False
        self.lost_frames = 0
        self.max_lost = 5  # setelah berapa frame predict tanpa correct, dianggap lost

    def predict(self, damping=0.85):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[2, 0] *= damping
        self.x[3, 0] *= damping
        return self.x[0, 0], self.x[1, 0]

    def correct(self, cx, cy):
        z = np.array([[cx], [cy]])
        if not self.initialized:
            self.x[0, 0] = cx
            self.x[1, 0] = cy
            self.initialized = True
            self.lost_frames = 0
            return

        y = z - self.H @ self.x                          # innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)          # Kalman gain

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.lost_frames = 0

    def mark_missed(self):
        self.lost_frames += 1

    def is_lost(self):
        return self.lost_frames > self.max_lost


try:
    import rclpy
    from geometry_msgs.msg import Point
    from std_msgs.msg import Int32MultiArray
    rclpy.init()
    ros_node = rclpy.create_node('testaruco_publisher')
    orange_pub = ros_node.create_publisher(Point, '/orange_position', 10)
    yellow_pub = ros_node.create_publisher(Point, '/yellow_position', 10)
    aruco_pub = ros_node.create_publisher(Int32MultiArray, '/aruco_markers', 10)
    ROS_AVAILABLE = True
    print("[ROS 2] Publisher aktif: /orange_position, /yellow_position, /aruco_markers")
except Exception as e:
    ROS_AVAILABLE = False
    print(f"[ROS 2] Tidak tersedia ({e}), hanya menampilkan di console")

DICTS = {
    'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
    'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
    'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
    'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
    'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
    'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
    'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
    'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
    'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
    'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
    'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
    'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
    'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
    'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
    'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
    'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
    'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
    'DICT_APRILTAG_16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'DICT_APRILTAG_25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'DICT_APRILTAG_36h10': cv2.aruco.DICT_APRILTAG_36h10,
    'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
}

cam_index = 1
dict_name = None
auto_mode = False

args = sys.argv[1:]
for i, a in enumerate(args):
    if a == '--auto':
        auto_mode = True
    elif a == '--dict' and i + 1 < len(args):
        dict_name = args[i + 1]
    elif a not in ('--dict',) and not a.startswith('--') and dict_name is None:
        cam_index = int(a)

if dict_name:
    if dict_name not in DICTS:
        print(f"Dictionary '{dict_name}' tidak dikenal. Pilihan: {', '.join(DICTS.keys())}")
        sys.exit(1)
    chosen_dicts = [dict_name]
elif auto_mode:
    chosen_dicts = list(DICTS.keys())
else:
    chosen_dicts = ['DICT_APRILTAG_36h11']

print(f"Menggunakan kamera index {cam_index}")
if auto_mode:
    print("Mode AUTO: mencoba semua dictionary...")
else:
    print(f"Dictionary: {chosen_dicts[0]}")

cap = cv2.VideoCapture(cam_index)

current_dict_idx = 0
detectors = {}
for name in chosen_dicts:
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICTS[name])
    params = cv2.aruco.DetectorParameters()
    # Tuning biar lebih toleran ke marker yang blur (gerak cepat / motion blur)
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02      # izinkan marker lebih kecil/blur kedetect
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE  # skip subpixel, lebih toleran ke blur
    params.errorCorrectionRate = 0.8          # naikin toleransi error-correction bit ID (default 0.6)
    detectors[name] = cv2.aruco.ArucoDetector(aruco_dict, params)

current_dict = chosen_dicts[0]
detector = detectors[current_dict]
switch_cooldown = 0

marker_trackers = {}

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Sharpening ringan sebelum deteksi ArUco -- bantu tajemin tepi marker yang blur
    # dikit (motion blur ringan), gak ngefek banyak kalau gambar udah tajam.
    sharpen_kernel = np.array([[0, -1, 0],
                                [-1, 5, -1],
                                [0, -1, 0]])
    gray_sharp = cv2.filter2D(gray, -1, sharpen_kernel)

    # --- Deteksi warna orange ---
    # Cuma 1 range, dijauhin dari hue merah (~0-8 dan ~172-179) biar gak ketuker
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([0, 29, 212])
    upper_orange = np.array([22, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    largest_orange = None
    largest_area = 0
    orange_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in orange_contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 165, 255), 2)
        cv2.putText(frame, "Orange", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
        cx_orange = x + w // 2
        cy_orange = y + h // 2
        print(f"Orange center: ({cx_orange},{cy_orange}) area:{int(area)}")
        if area > largest_area:
            largest_area = area
            largest_orange = (cx_orange, cy_orange, int(area))

    # --- Deteksi warna kuning ---
    lower_yellow = np.array([13, 122, 100])
    upper_yellow = np.array([43, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    largest_yellow = None
    largest_area = 0
    yellow_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in yellow_contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(frame, "Yellow", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        cx_yellow = x + w // 2
        cy_yellow = y + h // 2
        print(f"Yellow center: ({cx_yellow},{cy_yellow}) area:{int(area)}")
        if area > largest_area:
            largest_area = area
            largest_yellow = (cx_yellow, cy_yellow, int(area))

    if ROS_AVAILABLE and largest_orange:
        msg = Point()
        msg.x = float(largest_orange[0])
        msg.y = float(largest_orange[1])
        msg.z = float(largest_orange[2])
        orange_pub.publish(msg)

    if ROS_AVAILABLE and largest_yellow:
        msg = Point()
        msg.x = float(largest_yellow[0])
        msg.y = float(largest_yellow[1])
        msg.z = float(largest_yellow[2])
        yellow_pub.publish(msg)

    # --- Deteksi ArUco ---
    if auto_mode and switch_cooldown > 0:
        switch_cooldown -= 1

    corners, ids, rejected = detector.detectMarkers(gray_sharp)
    detected_ids = set()

    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        aruco_data = [chosen_dicts.index(current_dict)]
        for i, corner in enumerate(corners):
            c = corner[0]  # 4 titik sudut: [top-left, top-right, bottom-right, bottom-left]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            marker_id = int(ids[i])
            detected_ids.add(marker_id)

            # hitung yaw dari vektor top-left -> top-right
            dx = c[1][0] - c[0][0]
            dy = c[1][1] - c[0][1]
            angle_rad = np.arctan2(dy, dx)          # radian, -pi..pi
            angle_mdeg = int(np.degrees(angle_rad) * 1000)  # scale ke milli-derajat biar presisi

            # update Kalman filter buat marker ini
            if marker_id not in marker_trackers:
                marker_trackers[marker_id] = MarkerKalman()
            marker_trackers[marker_id].predict()
            marker_trackers[marker_id].correct(cx, cy)

            aruco_data.extend([marker_id, cx, cy, angle_mdeg])
            cv2.putText(frame, f"ID:{marker_id} {np.degrees(angle_rad):.1f}deg", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"[{current_dict}] ID {marker_id} center: ({cx},{cy}) angle: {np.degrees(angle_rad):.1f}")

        if ROS_AVAILABLE:
            msg = Int32MultiArray()
            msg.data = aruco_data
            aruco_pub.publish(msg)

        label = f"{current_dict} - ID:{ids[0]}"
    elif auto_mode and switch_cooldown == 0:
        current_dict_idx = (current_dict_idx + 1) % len(chosen_dicts)
        current_dict = chosen_dicts[current_dict_idx]
        detector = detectors[current_dict]
        switch_cooldown = 15
        label = f"Mencoba: {current_dict}"
    else:
        label = current_dict if not auto_mode else f"Mencoba: {current_dict}"

    # --- Predict posisi buat marker yang barusan gak kedetect (misal karena blur) ---
    for marker_id in list(marker_trackers.keys()):
        tracker = marker_trackers[marker_id]
        if marker_id not in detected_ids:
            if tracker.initialized:
                tracker.mark_missed()
                pred_cx, pred_cy = tracker.predict()
                if not tracker.is_lost():
                    cv2.circle(frame, (int(pred_cx), int(pred_cy)), 8, (0, 0, 255), 2)
                    cv2.putText(frame, f"ID:{marker_id} (predicted)", (int(pred_cx), int(pred_cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    del marker_trackers[marker_id]

    if auto_mode:
        cv2.putText(frame, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    if ROS_AVAILABLE:
        rclpy.spin_once(ros_node, timeout_sec=0)

    cv2.imshow("ArUco Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if ROS_AVAILABLE:
    ros_node.destroy_node()
    rclpy.shutdown()