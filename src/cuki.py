import sys
import os
import cv2
import numpy as np


class MarkerKalman:
    def __init__(self, dt=1/30):
        self.dt = dt
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 500
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        q = 5.0
        self.Q = np.eye(4) * q
        r = 4.0
        self.R = np.eye(2) * r
        self.initialized = False
        self.lost_frames = 0
        self.max_lost = 5

    def predict(self, damping=0.85):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.x[2, 0] *= damping
        self.x[3, 0] *= damping
        return float(self.x[0, 0]), float(self.x[1, 0])

    def correct(self, cx, cy):
        z = np.array([[cx], [cy]])
        if not self.initialized:
            self.x[0, 0] = cx
            self.x[1, 0] = cy
            self.initialized = True
            self.lost_frames = 0
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.lost_frames = 0

    def mark_missed(self):
        self.lost_frames += 1

    def is_lost(self):
        return self.lost_frames > self.max_lost


try:
    import rclpy
    from geometry_msgs.msg import Point, Twist, PoseArray, Pose, PoseStamped
    from std_msgs.msg import Int32MultiArray
    rclpy.init()
    ros_node = rclpy.create_node('testaruco_publisher')
    orange_pub = ros_node.create_publisher(Point, '/orange_position', 10)
    yellow_pub = ros_node.create_publisher(Point, '/yellow_position', 10)
    aruco_pub = ros_node.create_publisher(Int32MultiArray, '/aruco_markers', 10)
    obstacle_pub = ros_node.create_publisher(PoseArray, '/obstacle_yellow', 10)
    robot_pose_pub = ros_node.create_publisher(PoseStamped, '/robot_pose', 10)
    cmd_vel_pub = ros_node.create_publisher(Twist, '/cmd_vel', 10)
    ROS_AVAILABLE = True
    print("[ROS 2] Publisher aktif: /orange_position, /yellow_position, /aruco_markers, /cmd_vel, /obstacle_yellow")
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

cam_index = 2
dict_name = None
auto_mode = False
homography_path = None

args = sys.argv[1:]
skip_next = False
for i, a in enumerate(args):
    if skip_next:
        skip_next = False
        continue
    if a == '--auto':
        auto_mode = True
    elif a == '--dict' and i + 1 < len(args):
        dict_name = args[i + 1]
        skip_next = True
    elif a == '--homography' and i + 1 < len(args):
        homography_path = args[i + 1]
        skip_next = True
    elif not a.startswith('--'):
        try:
            cam_index = int(a)
        except ValueError:
            cam_index = a

if dict_name:
    if dict_name not in DICTS:
        print(f"Dictionary '{dict_name}' tidak dikenal. Pilihan: {', '.join(DICTS.keys())}")
        sys.exit(1)
    chosen_dicts = [dict_name]
elif auto_mode:
    chosen_dicts = list(DICTS.keys())
else:
    chosen_dicts = ['DICT_APRILTAG_36h11']

# --- Load homography matrix ---
H = None
FIELD_W = 1.2
FIELD_H = 1.2

def load_homography(path):
    import json
    with open(path, 'r') as f:
        data = json.load(f)
    return np.array(data['H'], dtype=np.float64)

default_homography = os.path.expanduser('~/Downloads/Nexus_Gazebo/src/homography.json')

if homography_path:
    try:
        H = load_homography(homography_path)
        print(f"[Homography] Loaded dari {homography_path}")
        print(f"[Homography] Field: {FIELD_W}m x {FIELD_H}m, origin: center")
    except Exception as e:
        print(f"[Homography] Gagal load {homography_path}: {e}")
        sys.exit(1)
elif os.path.exists(default_homography):
    try:
        H = load_homography(default_homography)
        print(f"[Homography] Auto-loaded dari {default_homography}")
        print(f"[Homography] Field: {FIELD_W}m x {FIELD_H}m, origin: center")
    except Exception as e:
        print(f"[Homography] Gagal auto-load: {e}")
else:
    print("[Homography] Tidak ada file, menggunakan pixel coordinates (fallback)")


def pixel_to_cm(cx, cy):
    """Konversi pixel (cx, cy) ke real-world (x, y) dalam cm."""
    if H is not None:
        pt = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, H)
        x_m, y_m = result[0][0]
        x_m -= FIELD_W / 2.0
        y_m -= FIELD_H / 2.0
        return float(x_m * 100.0), float(y_m * 100.0)
    
    scale = 0.5
    x_cm = (cx - frame_w / 2) * scale
    y_cm = -(cy - frame_h / 2) * scale
    return float(x_cm), float(y_cm)

print(f"Menggunakan kamera index {cam_index}")
if auto_mode:
    print("Mode AUTO: mencoba semua dictionary...")
else:
    print(f"Dictionary: {chosen_dicts[0]}")

cap = cv2.VideoCapture(cam_index)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
center_x = frame_w // 2

# --- Navigation constants ---
KP_LINEAR   = 0.0005
MAX_LINEAR  = 0.2
MAX_ANGULAR = 0.5
STOP_AREA   = 8000

# --- Align-to-target constants ---
YAW_DEADZONE_DEG = 7.0
KP_ANGULAR_YAW   = 0.01
FORWARD_SPEED    = 0.15

# --- Potential Field constants ---
AVOID_DISTANCE_CM = 35.0
OBSTACLE_FRONT_ANGLE = 60.0
AVOID_KP = 0.025
MIN_SPEED_FACTOR = 0.2

# --- Path tracking ---
PATH_MAX_POINTS = 200
robot_path = []  # List of (cx, cy) in pixels

current_dict_idx = 0
detectors = {}
for name in chosen_dicts:
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICTS[name])
    params = cv2.aruco.DetectorParameters()
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.02
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    params.errorCorrectionRate = 0.8
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

    sharpen_kernel = np.array([[0, -1, 0],
                                [-1, 5, -1],
                                [0, -1, 0]])
    gray_sharp = cv2.filter2D(gray, -1, sharpen_kernel)

    # --- Deteksi warna orange (OBSTACLE) ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([170, 100, 100])
    upper_orange = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    
    largest_orange = None
    largest_area = 0
    obstacle_list = []
    orange_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in orange_contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "Obstacle", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cx_orange = x + w // 2
        cy_orange = y + h // 2
        cv2.circle(frame, (cx_orange, cy_orange), 5, (0, 0, 255), -1)
        
        print(f"Obstacle center: ({cx_orange},{cy_orange}) area:{int(area)}")
        ox_cm, oy_cm = pixel_to_cm(cx_orange, cy_orange)
        if ox_cm is not None:
            print(f"  → Obstacle: x={ox_cm:.1f}cm y={oy_cm:.1f}cm")
            obstacle_list.append((ox_cm, oy_cm, area, w, h))
        
        if area > largest_area:
            largest_area = area
            largest_orange = (cx_orange, cy_orange, int(area))

    # --- Deteksi warna kuning (TARGET) ---
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
        yx_cm, yy_cm = pixel_to_cm(cx_yellow, cy_yellow)
        if yx_cm is not None:
            print(f"  → Yellow: x={yx_cm:.1f}cm y={yy_cm:.1f}cm")
        if area > largest_area:
            largest_area = area
            largest_yellow = (cx_yellow, cy_yellow, int(area))

    if ROS_AVAILABLE and largest_orange:
        msg = Point()
        msg.x = float(largest_orange[0])
        msg.y = float(largest_orange[1])
        orange_pub.publish(msg)

    if ROS_AVAILABLE and largest_yellow:
        msg = Point()
        msg.x = float(largest_yellow[0])
        msg.y = float(largest_yellow[1])
        yellow_pub.publish(msg)

    # --- Deteksi ArUco ---
    if auto_mode and switch_cooldown > 0:
        switch_cooldown -= 1

    corners, ids, rejected = detector.detectMarkers(gray_sharp)
    detected_ids = set()
    angle_312 = None
    marker_312_pos = None

    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        aruco_data = [int(chosen_dicts.index(current_dict))]
        for i, corner in enumerate(corners):
            c = corner[0]
            cx = int(c[:, 0].mean())
            cy = int(c[:, 1].mean())
            marker_id = int(ids[i])
            detected_ids.add(marker_id)

            dx = c[1][0] - c[0][0]
            dy = c[1][1] - c[0][1]
            angle_rad = np.arctan2(dy, dx)
            angle_deg = np.degrees(angle_rad)
            angle_mdeg = int(angle_deg * 1000)

            if marker_id == 81:
                angle_312 = angle_deg
                marker_312_pos = (cx, cy)

            if marker_id not in marker_trackers:
                marker_trackers[marker_id] = MarkerKalman()
            marker_trackers[marker_id].predict()
            marker_trackers[marker_id].correct(cx, cy)

            aruco_data.extend([int(marker_id), int(cx), int(cy), int(angle_mdeg)])
            cv2.putText(frame, f"ID:{marker_id} {angle_deg:.1f}deg", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"[{current_dict}] ID {marker_id} center: ({cx},{cy}) angle: {angle_deg:.1f}")

            if marker_id == 81:
                x_cm, y_cm = pixel_to_cm(cx, cy)
                if x_cm is not None:
                    pose_msg = PoseStamped()
                    pose_msg.header.frame_id = 'map'
                    pose_msg.header.stamp = ros_node.get_clock().now().to_msg()
                    pose_msg.pose.position.x = float(x_cm) / 100.0
                    pose_msg.pose.position.y = float(y_cm) / 100.0
                    pose_msg.pose.position.z = 0.0
                    
                    yaw_rad = float(np.radians(angle_deg))
                    pose_msg.pose.orientation.x = 0.0
                    pose_msg.pose.orientation.y = 0.0
                    pose_msg.pose.orientation.z = float(np.sin(yaw_rad / 2))
                    pose_msg.pose.orientation.w = float(np.cos(yaw_rad / 2))
                    
                    robot_pose_pub.publish(pose_msg)
                    print(f"  → Robot pose: x={x_cm:.1f}cm y={y_cm:.1f}cm yaw={angle_deg:.1f}°")

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

    # --- Predict posisi buat marker yang barusan gak kedetect ---
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

    # ============================================================
    # === POTENTIAL FIELD: TARGET = BOLA KUNING LANGSUNG ===
    # === DENGAN PATH VISUALIZATION ===
    # ============================================================
    twist = Twist()
    
    if ROS_AVAILABLE:
        if angle_312 is not None and marker_312_pos is not None and largest_yellow is not None:
            angle_312_f = float(angle_312)
            mx_pixel, my_pixel = marker_312_pos
            yc_x_pixel, yc_y_pixel, yc_area = largest_yellow
            
            mx_cm, my_cm = pixel_to_cm(mx_pixel, my_pixel)
            yc_x_cm, yc_y_cm = pixel_to_cm(yc_x_pixel, yc_y_pixel)
            
            if mx_cm is not None and yc_x_cm is not None:
                
                # === UPDATE PATH (trail robot) ===
                robot_path.append((mx_pixel, my_pixel))
                if len(robot_path) > PATH_MAX_POINTS:
                    robot_path.pop(0)
                
                # === 1. TARGET LANGSUNG KE POSISI BOLA KUNING ===
                # Target bearing = arah dari robot ke bola kuning (langsung)
                target_bearing = float(np.degrees(
                    np.arctan2(yc_y_cm - my_cm, yc_x_cm - mx_cm)
                ))
                
                # === 2. OBSTACLE REPULSION FORCE ===
                repulse_angle = 0.0
                repulse_strength = 0.0
                
                for obs in obstacle_list:
                    ox, oy, oarea, ow, oh = obs
                    dx = ox - mx_cm
                    dy = oy - my_cm
                    dist_to_obs = np.sqrt(dx**2 + dy**2)
                    
                    angle_to_obs = float(np.degrees(np.arctan2(dy, dx)))
                    angle_diff = ((angle_to_obs - angle_312_f + 180) % 360) - 180
                    
                    if abs(angle_diff) < OBSTACLE_FRONT_ANGLE and dist_to_obs < AVOID_DISTANCE_CM:
                        force = min(1.0, AVOID_DISTANCE_CM / max(dist_to_obs, 5.0))
                        
                        # Arah menghindar: perpendicular ke obstacle
                        avoid_left = ((angle_to_obs + 90 - angle_312_f + 180) % 360) - 180
                        avoid_right = ((angle_to_obs - 90 - angle_312_f + 180) % 360) - 180
                        
                        target_diff_left = abs(((avoid_left + angle_312_f - target_bearing + 180) % 360) - 180)
                        target_diff_right = abs(((avoid_right + angle_312_f - target_bearing + 180) % 360) - 180)
                        
                        if target_diff_left < target_diff_right:
                            repulse_angle += avoid_left * force
                        else:
                            repulse_angle += avoid_right * force
                        
                        repulse_strength = max(repulse_strength, force)
                        
                        # Visualisasi obstacle force
                        obs_pixel_x = int(mx_pixel + 50 * np.cos(np.radians(angle_to_obs)))
                        obs_pixel_y = int(my_pixel + 50 * np.sin(np.radians(angle_to_obs)))
                        cv2.line(frame, (mx_pixel, my_pixel), (obs_pixel_x, obs_pixel_y), (0, 0, 255), 2)
                        
                        print(f"  ⚠️ OBSTACLE: dist={dist_to_obs:.1f}cm force={force:.2f} "
                              f"angle_diff={angle_diff:.1f}° repulse={repulse_angle:.1f}°")
                
                # === 3. BLEND: Target langsung ke kuning + Repulsion ===
                target_weight = max(MIN_SPEED_FACTOR, 1.0 - repulse_strength * 0.7)
                repulse_weight = 1.0 - target_weight
                
                # Avoid bearing = target bearing + repulse offset
                # Repulse angle sudah dalam frame global (relatif ke yaw, tapi kita convert)
                # Sebenarnya repulse_angle adalah offset dari yaw, jadi:
                avoid_bearing = angle_312_f + repulse_angle
                
                # Tapi kita mau blend antara target dan avoid
                # Cara benar: weighted average dari dua bearing
                # Gunakan vector averaging untuk avoid wrap-around issue
                target_x = np.cos(np.radians(target_bearing))
                target_y = np.sin(np.radians(target_bearing))
                avoid_x = np.cos(np.radians(avoid_bearing))
                avoid_y = np.sin(np.radians(avoid_bearing))
                
                blend_x = target_x * target_weight + avoid_x * repulse_weight
                blend_y = target_y * target_weight + avoid_y * repulse_weight
                
                blended_bearing = float(np.degrees(np.arctan2(blend_y, blend_x)))
                
                # === 4. SPEED MODULATION ===
                speed_factor = max(MIN_SPEED_FACTOR, 1.0 - repulse_strength * 0.8)
                base_speed = FORWARD_SPEED * speed_factor
                
                # === 5. HITUNG CMD_VEL ===
                raw_err = angle_312_f - blended_bearing
                err_angle = ((raw_err + 180.0) % 360.0) - 180.0
                
                ang_z = -err_angle * KP_ANGULAR_YAW * 4
                ang_z = max(-MAX_ANGULAR, min(MAX_ANGULAR, ang_z))
                
                if abs(err_angle) < YAW_DEADZONE_DEG or repulse_strength > 0:
                    twist.linear.x = float(base_speed)
                else:
                    twist.linear.x = 0.0
                
                twist.angular.z = float(ang_z)
                
                # === 6. VISUALISASI PATH & NAVIGASI ===
                
                # Gambar path (trail robot) - garis putus-putus merah
                if len(robot_path) > 1:
                    for i in range(1, len(robot_path)):
                        alpha = int(255 * i / len(robot_path))
                        color = (0, 0, min(255, alpha + 100))
                        cv2.line(frame, robot_path[i-1], robot_path[i], color, 2)
                
                # Gambar garis dari robot ke target (bola kuning) - hijau solid
                cv2.arrowedLine(frame, (mx_pixel, my_pixel),
                               (yc_x_pixel, yc_y_pixel), (0, 255, 0), 3)
                
                # Gambar blended direction (biru) - arah yang sebenarnya diambil
                blend_end_x = int(mx_pixel + 100 * np.cos(np.radians(blended_bearing)))
                blend_end_y = int(my_pixel + 100 * np.sin(np.radians(blended_bearing)))
                cv2.arrowedLine(frame, (mx_pixel, my_pixel),
                               (blend_end_x, blend_end_y), (255, 165, 0), 2)
                
                # Status text
                if repulse_strength > 0:
                    cv2.putText(frame, f"AVOIDING", (mx_pixel + 10, my_pixel - 25),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                cv2.putText(frame, f"Target: {target_bearing:.1f}°", (10, frame_h - 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Blend: {blended_bearing:.1f}°", (10, frame_h - 40),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 2)
                cv2.putText(frame, f"Mode: {'AVOID' if repulse_strength > 0 else 'TRACK'} | "
                                  f"Speed: {base_speed:.2f}", (10, frame_h - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                print(f"  → NAVIGATE: target_bearing={target_bearing:.1f}° "
                      f"blend={blended_bearing:.1f}° err={err_angle:.1f}° "
                      f"ang.z={twist.angular.z:.3f} lin.x={twist.linear.x:.3f} "
                      f"repulse={repulse_strength:.2f}")
            
            else:
                print("  → [WARN] Homography not loaded")
        else:
            print(f"  → [IDLE] angle_312={angle_312 is not None}, "
                  f"marker_pos={marker_312_pos is not None}, "
                  f"yellow={largest_yellow is not None}")

    # --- Publish obstacle (orange) ke ROS ---
    if ROS_AVAILABLE and obstacle_list:
        obs_msg = PoseArray()
        obs_msg.header.frame_id = 'map'
        obs_msg.header.stamp = ros_node.get_clock().now().to_msg()
        
        for obs in obstacle_list:
            ox, oy, oarea, ow, oh = obs
            pose = Pose()
            pose.position.x = float(ox) / 100.0
            pose.position.y = float(oy) / 100.0
            pose.position.z = 0.0
            pose.orientation.x = float(ow) / 100.0
            pose.orientation.y = float(oh) / 100.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            obs_msg.poses.append(pose)
        
        obstacle_pub.publish(obs_msg)
        print(f"[Obstacle] Published {len(obs_msg.poses)} obstacles")

    # --- Publish cmd_vel (SELALU) ---
    if ROS_AVAILABLE:
        cmd_vel_pub.publish(twist)
        print(f"[CmdVel] linear.x={twist.linear.x:.3f}, angular.z={twist.angular.z:.3f}")

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