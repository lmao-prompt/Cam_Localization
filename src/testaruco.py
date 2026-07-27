import sys
import time
import heapq
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


class AStarPathfinder:
    def __init__(self, frame_w, frame_h, cell_size=10):
        self.cell_size = cell_size
        self.grid_w = frame_w // cell_size
        self.grid_h = frame_h // cell_size
        self.last_grid = None  # grid terakhir dipakai find_path, buat dasar line-of-sight check

    def _to_grid(self, px, py):
        gx = int(px / self.cell_size)
        gy = int(py / self.cell_size)
        gx = max(0, min(self.grid_w - 1, gx))
        gy = max(0, min(self.grid_h - 1, gy))
        return gx, gy

    def _to_pixel(self, gx, gy):
        return gx * self.cell_size + self.cell_size // 2, gy * self.cell_size + self.cell_size // 2

    def find_path(self, start_px, goal_px, obstacles_px, inflation_cells=3):
        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)

        for obs in obstacles_px:
            ogx, ogy = self._to_grid(obs[0], obs[1])
            for dx in range(-inflation_cells, inflation_cells + 1):
                for dy in range(-inflation_cells, inflation_cells + 1):
                    if dx * dx + dy * dy <= inflation_cells * inflation_cells:
                        nx, ny = ogx + dx, ogy + dy
                        if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                            grid[ny, nx] = 1

        self.last_grid = grid  # simpan buat dipakai simplify_path()

        sgx, sgy = self._to_grid(start_px[0], start_px[1])
        ggx, ggy = self._to_grid(goal_px[0], goal_px[1])

        if grid[sgy, sgx] == 1 or grid[ggy, ggx] == 1:
            return None

        open_set = [(0, sgx, sgy)]
        came_from = {}
        g_score = {(sgx, sgy): 0}

        neighbors = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                     (-1, -1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (1, 1, 1.414)]

        while open_set:
            _, cx, cy = heapq.heappop(open_set)

            if (cx, cy) == (ggx, ggy):
                path = []
                cur = (ggx, ggy)
                while cur in came_from:
                    path.append(self._to_pixel(cur[0], cur[1]))
                    cur = came_from[cur]
                path.append(self._to_pixel(sgx, sgy))
                path.reverse()
                return path

            for dx, dy, cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h and grid[ny, nx] == 0:
                    tentative = g_score[(cx, cy)] + cost * self.cell_size
                    if tentative < g_score.get((nx, ny), float('inf')):
                        came_from[(nx, ny)] = (cx, cy)
                        g_score[(nx, ny)] = tentative
                        h = np.sqrt((nx - ggx) ** 2 + (ny - ggy) ** 2) * self.cell_size
                        heapq.heappush(open_set, (tentative + h, nx, ny))

        return None

    def _line_clear(self, p1, p2):
        """Cek garis lurus p1->p2 (koordinat pixel) bebas obstacle, pakai Bresenham di grid terakhir."""
        if self.last_grid is None:
            return True

        x0, y0 = self._to_grid(p1[0], p1[1])
        x1, y1 = self._to_grid(p2[0], p2[1])

        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        x, y = x0, y0
        while True:
            if self.last_grid[y, x] == 1:
                return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return True

    def simplify_path(self, path):
        """String-pulling: buang waypoint yang gak perlu selama garis lurus masih aman.
        Ubah path zigzag hasil A* grid jadi beberapa segmen lurus saja."""
        if not path or len(path) < 3:
            return path

        simplified = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._line_clear(path[i], path[j]):
                j -= 1
            simplified.append(path[j])
            i = j

        return simplified


try:
    import rclpy
    from geometry_msgs.msg import Point, Twist
    from std_msgs.msg import Int32MultiArray
    rclpy.init()
    ros_node = rclpy.create_node('testaruco_publisher')
    orange_pub = ros_node.create_publisher(Point, '/orange_position', 10)
    yellow_pub = ros_node.create_publisher(Point, '/yellow_position', 10)
    aruco_pub = ros_node.create_publisher(Int32MultiArray, '/aruco_markers', 10)
    ROS_AVAILABLE = True
    cmd_vel_pub = ros_node.create_publisher(Twist, '/cmd_vel', 10)
    print("[ROS 2] Publisher aktif: /orange_position, /yellow_position, /aruco_markers, /cmd_vel")
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
            cam_index = ar

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
FIELD_W = 1.2  # meter (dari calibrate_map.py)
FIELD_H = 1.2  # meter (dari calibrate_map.py)

def load_homography(path):
    """Load homography matrix 3x3 dari file JSON."""
    import json
    with open(path, 'r') as f:
        data = json.load(f)
    return np.array(data['H'], dtype=np.float64)

if homography_path:
    try:
        H = load_homography(homography_path)
        print(f"[Homography] Loaded dari {homography_path}")
        print(f"[Homography] Field: {FIELD_W}m x {FIELD_H}m, origin: center")
    except Exception as e:
        print(f"[Homography] Gagal load {homography_path}: {e}")
        sys.exit(1)
else:
    print("[Homography] Tidak ada file, menggunakan pixel coordinates")


def pixel_to_cm(cx, cy):
    """Konversi pixel (cx, cy) ke real-world (x, y) dalam cm.
    Return (None, None) jika homography belum di-load."""
    if H is None:
        return None, None
    pt = np.array([[[float(cx), float(cy)]]], dtype=np.float32)
    result = cv2.perspectiveTransform(pt, H)
    x_m, y_m = result[0][0]
    x_m -= FIELD_W / 2.0
    y_m -= FIELD_H / 2.0
    return x_m * 100.0, y_m * 100.0

print(f"Menggunakan kamera index {cam_index}")
if auto_mode:
    print("Mode AUTO: mencoba semua dictionary...")
else:
    print(f"Dictionary: {chosen_dicts[0]}")

cap = cv2.VideoCapture(cam_index)
frame_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
center_x = frame_w // 2
center_y = frame_h // 2

# --- Chase / align constants ---
KP_LINEAR   = 0.0005      # proportional gain untuk maju (berdasar area yellow)
MAX_LINEAR  = 0.2         # m/s
STOP_AREA   = 8000        # area yellow terlalu dekat -> stop

# --- Align-to-yaw constants (marker 81) ---
YAW_DEADZONE_DEG = 30.0     # toleransi "sudah lurus" ke arah target
FORWARD_SPEED    = 0.15    # m/s, kecepatan maju konstan begitu dalam deadzone

MAX_ANGULAR    = 0.5      # rad/s, ini yang dikirim ke /cmd_vel selama pulse
                           # (hardware cuma punya 1 level PWM tetap ~65 buat rotasi,
                           # jadi ini BUKAN proporsional lagi -- cuma nentuin arah put muter)
SATURATION_DEG = 90.0      # dipakai sbg referensi lama, gak dipakai lagi buat gain
KP_ANGULAR_YAW = MAX_ANGULAR / SATURATION_DEG   # disimpan buat referensi, sudah tidak dipakai

# --- Pulse-mode cmd_vel (bukan continuous!) ---
# Rotasi hardware bang-bang (fixed PWM), jadi angular.z gak boleh "nyala" lama-lama.
# Alurnya: publish gerak sebentar (PULSE_DURATION) -> publish stop -> diam (cooldown
# CMD_VEL_PERIOD) buat kamera baca ulang posisi marker yang sudah stabil -> evaluasi lagi.
PULSE_DURATION   = 0.1     # detik, durasi satu kali "sentakan" rotasi
CMD_VEL_PERIOD   = 1.0     # detik, jeda/cooldown sebelum evaluasi ulang setelah pulse berhenti
last_cmd_vel_time = 0.0    # timestamp terakhir kali cooldown "start"
pulse_active      = False  # lagi dalam fase gerak (belum waktunya stop)
pulse_end_time    = 0.0    # kapan pulse ini harus di-stop
last_astar_path = None    # cache path terakhir, digambar tiap frame

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
pathfinder = AStarPathfinder(frame_w, frame_h, cell_size=10)

CLEARANCE_CM_DEFAULT = 5
cv2.namedWindow("ArUco Detection")
cv2.namedWindow("Control")
cv2.createTrackbar("Clearance(cm)", "Control", CLEARANCE_CM_DEFAULT, 30, lambda x: None)

pixels_per_cm = frame_w / (FIELD_W * 100.0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    clearance_cm = cv2.getTrackbarPos("Clearance(cm)", "Control")

    if H is not None:
        cm1 = pixel_to_cm(center_x, center_y)
        cm2 = pixel_to_cm(center_x + 10, center_y)
        if cm1[0] is not None and cm2[0] is not None:
            diff = abs(cm2[0] - cm1[0])
            if diff > 0:
                pixels_per_cm = 10.0 / diff

    INFLATION_CELLS = max(1, round(clearance_cm * pixels_per_cm / pathfinder.cell_size))

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Sharpening ringan sebelum deteksi ArUco
    sharpen_kernel = np.array([[0, -1, 0],
                                [-1, 5, -1],
                                [0, -1, 0]])
    gray_sharp = cv2.filter2D(gray, -1, sharpen_kernel)

    # --- Deteksi warna orange ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([100, 100, 100])
    upper_orange = np.array([179, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    largest_orange = None
    largest_area = 0
    orange_obstacles_px = []
    orange_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    orange_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in orange_contours:
        area = cv2.contourArea(cnt)
        if area < 1500:
            continue

        cv2.drawContours(frame, [cnt], -1, (0, 165, 255), 2)
        cv2.putText(frame, "Obstacle", (cnt[0][0][0], cnt[0][0][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        M = cv2.moments(cnt)
        if M["m00"] == 0:
            continue
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # === FIX: Fill seluruh kontur ke mask, bukan cuma titik sudut ===
        cv2.drawContours(orange_mask, [cnt], -1, 255, thickness=-1)

        cx_orange, cy_orange = cX, cY
        print(f"Orange obstacle center: ({cx_orange},{cy_orange}) area:{int(area)}")
        ox_cm, oy_cm = pixel_to_cm(cx_orange, cy_orange)
        if ox_cm is not None:
            print(f"  -> Orange obstacle: x={ox_cm:.1f}cm y={oy_cm:.1f}cm")
        if area > largest_area:
            largest_area = area
            largest_orange = (cx_orange, cy_orange, int(area))

    # Ambil SEMUA pixel obstacle dari mask yang sudah di-fill
    ys, xs = np.where(orange_mask > 0)
    orange_obstacles_px = list(zip(xs.tolist(), ys.tolist()))

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
        yx_cm, yy_cm = pixel_to_cm(cx_yellow, cy_yellow)
        if yx_cm is not None:
            print(f"  -> Yellow: x={yx_cm:.1f}cm y={yy_cm:.1f}cm")
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
    angle_81 = None
    marker_81_pos = None

    if ids is not None and len(ids) > 0:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        aruco_data = [chosen_dicts.index(current_dict)]
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

            if marker_id == 312:
                angle_81 = angle_deg
                marker_81_pos = (cx, cy)

            if marker_id not in marker_trackers:
                marker_trackers[marker_id] = MarkerKalman()
            marker_trackers[marker_id].predict()
            marker_trackers[marker_id].correct(cx, cy)

            aruco_data.extend([marker_id, cx, cy, angle_mdeg])
            cv2.putText(frame, f"ID:{marker_id} {angle_deg:.1f}deg", (cx, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            print(f"[{current_dict}] ID {marker_id} center: ({cx},{cy}) angle: {angle_deg:.1f}")

            if marker_id == 312:
                x_cm, y_cm = pixel_to_cm(cx, cy)
                if x_cm is not None:
                    print(f"  -> Marker 81: x={x_cm:.1f}cm y={y_cm:.1f}cm")
                else:
                    print(f"  -> Marker 81: (homography belum di-load)")

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

    # --- A* path (dihitung tiap frame buat digambar & dipakai saat evaluasi cmd_vel) ---
    last_astar_path = None

    if angle_81 is not None and marker_81_pos is not None and largest_yellow is not None:
        mx, my = marker_81_pos
        yc_x, yc_y, yc_area = largest_yellow

        if orange_obstacles_px:
            raw_path = pathfinder.find_path(
                (mx, my), (yc_x, yc_y), orange_obstacles_px, inflation_cells=INFLATION_CELLS
            )
            if raw_path:
                last_astar_path = pathfinder.simplify_path(raw_path)  # <-- path disederhanakan
            if last_astar_path and len(last_astar_path) >= 2:
                wp_x, wp_y = last_astar_path[1]
                print(f"  -> A* PATH (simplified): {len(last_astar_path)} waypoints")

    # --- Align-to-target logic (PULSE MODE, bukan continuous) ---
    if ROS_AVAILABLE:
        now = time.time()

        # 1) Kalau lagi pulsing dan durasinya udah lewat -> kirim STOP sekali, mulai cooldown
        if pulse_active and now >= pulse_end_time:
            stop_twist = Twist()
            cmd_vel_pub.publish(stop_twist)
            pulse_active = False
            last_cmd_vel_time = now
            print("  -> [PULSE END] stop, mulai cooldown")

        # 2) Kalau gak lagi pulsing dan cooldown udah abis -> evaluasi ulang sekali
        elif not pulse_active and (now - last_cmd_vel_time >= CMD_VEL_PERIOD):
            twist = Twist()

            if angle_81 is not None and marker_81_pos is not None and largest_yellow is not None:
                mx, my = marker_81_pos
                yc_x, yc_y, yc_area = largest_yellow

                target_bearing = None
                if last_astar_path and len(last_astar_path) >= 2:
                    wp_x, wp_y = last_astar_path[1]
                    target_bearing = float(np.degrees(np.arctan2(wp_y - my, wp_x - mx)))
                if target_bearing is None:
                    target_bearing = float(np.degrees(np.arctan2(yc_y - my, yc_x - mx)))

                angle_81_f = float(angle_81)
                raw_err = angle_81_f - target_bearing
                err_angle = ((raw_err + 180.0) % 360.0) - 180.0

                if abs(err_angle) > YAW_DEADZONE_DEG:
                    # Hardware cuma punya 1 level PWM (~65) buat rotasi -> gak proporsional,
                    # cuma arah putarnya (tanda) yang dipakai. Durasi pulse yang jadi "dosis" koreksi.
                    ang_z = -MAX_ANGULAR if err_angle > 0 else MAX_ANGULAR
                    twist.angular.z = float(ang_z)
                    twist.linear.x = 0.0
                    cmd_vel_pub.publish(twist)
                    pulse_active = True
                    pulse_end_time = now + PULSE_DURATION
                    print(f"  -> [PULSE START] yaw={angle_81_f:.1f} bearing={target_bearing:.1f} "
                          f"err={err_angle:.1f}deg ang.z={ang_z:.3f} selama {PULSE_DURATION}s")
                else:
                    twist.angular.z = 0.0
                    twist.linear.x = float(FORWARD_SPEED)
                    cmd_vel_pub.publish(twist)
                    last_cmd_vel_time = now
                    print(f"  -> FACING TARGET: yaw={angle_81_f:.1f} bearing={target_bearing:.1f} "
                          f"err={err_angle:.1f}deg lin={twist.linear.x:.2f}")
            else:
                # gak ada marker/target yang kelihatan -> tetap diam, tunggu cooldown lagi
                last_cmd_vel_time = now

    if auto_mode:
        cv2.putText(frame, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.putText(frame, f"Clearance: {clearance_cm}cm (cells:{INFLATION_CELLS})", (10, frame_h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    if ROS_AVAILABLE:
        rclpy.spin_once(ros_node, timeout_sec=0)

    if last_astar_path and len(last_astar_path) >= 2:
        for i in range(len(last_astar_path) - 1):
            p1 = (int(last_astar_path[i][0]), int(last_astar_path[i][1]))
            p2 = (int(last_astar_path[i+1][0]), int(last_astar_path[i+1][1]))
            cv2.line(frame, p1, p2, (255, 0, 255), 2)
        wp_x, wp_y = last_astar_path[1]
        cv2.circle(frame, (int(wp_x), int(wp_y)), 6, (255, 0, 255), -1)
        cv2.putText(frame, "A*WP", (int(wp_x) + 8, int(wp_y)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

    cv2.imshow("ArUco Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if ROS_AVAILABLE:
    ros_node.destroy_node()
    rclpy.shutdown()