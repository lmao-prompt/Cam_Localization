#!/usr/bin/env python3
"""
Nexus Vision Navigation - ULTRA LITE
ArUco + A* + ROS2 TF/Marker (CPU-friendly)
"""

import sys
import time
import heapq
import argparse
import signal
import threading
import cv2
import numpy as np

# ============================================================================
# CONFIG
# ============================================================================
FIELD_W, FIELD_H = 1.2, 1.2
CELL_SIZE = 20
CLEARANCE_CM = 8

# Throttle (pisah robot vs target)
ROBOT_POSE_MIN_PERIOD = 0.08
TARGET_POSE_MIN_PERIOD = 0.15
POSE_POS_THRESH = 1.5
POSE_YAW_THRESH = np.deg2rad(3.0)

AStar_MAX_HZ = 5.0
PRINT_INTERVAL = 2.0

# Pulse cmd_vel
PULSE_DURATION = 0.1
CMD_VEL_PERIOD = 0.9
YAW_DEADZONE_DEG = 20.0
FORWARD_SPEED = 0.15
MAX_ANGULAR = 0.5
TARGET_HEARTBEAT_PERIOD = 1.0

# Color
LOWER_ORANGE = np.array([100, 100, 100])
UPPER_ORANGE = np.array([179, 255, 255])
LOWER_YELLOW = np.array([13, 122, 100])
UPPER_YELLOW = np.array([43, 255, 255])


class MarkerKalman:
    def __init__(self, dt=1/30):
        self.dt = dt
        self.x = np.zeros((4, 1))
        self.P = np.eye(4) * 500
        self.F = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        self.H = np.array([[1,0,0,0],[0,1,0,0]])
        self.Q = np.eye(4) * 5.0
        self.R = np.eye(2) * 4.0
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
        z = np.array([[cx],[cy]])
        if not self.initialized:
            self.x[0,0], self.x[1,0] = cx, cy
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


class AStarLite:
    def __init__(self, frame_w, frame_h, cell_size=CELL_SIZE):
        self.cs = cell_size
        self.gw = max(1, frame_w // cell_size)
        self.gh = max(1, frame_h // cell_size)
        self.last_grid = None

    def _g(self, px, py):
        return max(0, min(self.gw-1, int(px/self.cs))), max(0, min(self.gh-1, int(py/self.cs)))

    def _p(self, gx, gy):
        return gx*self.cs + self.cs//2, gy*self.cs + self.cs//2

    def find_path(self, s, g, obstacles, inflate=3):
        grid = np.zeros((self.gh, self.gw), dtype=np.uint8)
        for ox, oy, r in obstacles:
            gx, gy = self._g(ox, oy)
            rc = int(r / self.cs) + inflate
            x0, x1 = max(0, gx-rc), min(self.gw, gx+rc+1)
            y0, y1 = max(0, gy-rc), min(self.gh, gy+rc+1)
            grid[y0:y1, x0:x1] = 1
        self.last_grid = grid
        sg = self._g(s[0], s[1])
        gg = self._g(g[0], g[1])
        if grid[sg[1], sg[0]] or grid[gg[1], gg[0]]:
            return None
        open_set = [(0, sg[0], sg[1])]
        came = {}
        gscore = {sg: 0}
        neigh = [(-1,0,1),(1,0,1),(0,-1,1),(0,1,1),(-1,-1,1.414),(1,-1,1.414),(-1,1,1.414),(1,1,1.414)]
        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            if (cx,cy) == gg:
                path = []
                cur = gg
                while cur in came:
                    path.append(self._p(cur[0], cur[1]))
                    cur = came[cur]
                path.append(self._p(sg[0], sg[1]))
                path.reverse()
                return path
            for dx,dy,cost in neigh:
                nx, ny = cx+dx, cy+dy
                if 0 <= nx < self.gw and 0 <= ny < self.gh and not grid[ny,nx]:
                    tent = gscore[(cx,cy)] + cost*self.cs
                    if tent < gscore.get((nx,ny), 1e9):
                        came[(nx,ny)] = (cx,cy)
                        gscore[(nx,ny)] = tent
                        h = ((nx-gg[0])**2 + (ny-gg[1])**2)**0.5 * self.cs
                        heapq.heappush(open_set, (tent+h, nx, ny))
        return None

    def simplify(self, path):
        if not path or len(path) < 3:
            return path
        simp = [path[0]]
        i = 0
        while i < len(path)-1:
            j = len(path)-1
            while j > i+1 and not self._line_ok(path[i], path[j]):
                j -= 1
            simp.append(path[j]); i = j
        return simp

    def _line_ok(self, a, b):
        if self.last_grid is None:
            return True
        x0,y0 = self._g(a[0], a[1])
        x1,y1 = self._g(b[0], b[1])
        dx, dy = abs(x1-x0), -abs(y1-y0)
        sx = 1 if x0<x1 else -1
        sy = 1 if y0<y1 else -1
        err = dx+dy
        while True:
            if self.last_grid[y0,x0]:
                return False
            if x0==x1 and y0==y1:
                break
            e2 = 2*err
            if e2 >= dy:
                err += dy; x0 += sx
            if e2 <= dx:
                err += dx; y0 += sy
        return True


def load_homography(path):
    import json
    with open(path) as f:
        return np.array(json.load(f)['H'], dtype=np.float64)


def yaw_to_q(yaw):
    h = yaw*0.5
    return 0.0, 0.0, np.sin(h), np.cos(h)


def build_pose(node, x_cm, y_cm, yaw=None, frame_id="map"):
    from geometry_msgs.msg import PoseStamped
    m = PoseStamped()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = frame_id
    m.pose.position.x = float(x_cm)/100.0
    m.pose.position.y = float(y_cm)/100.0
    m.pose.position.z = 0.0
    if yaw is not None:
        q = yaw_to_q(yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
    else:
        m.pose.orientation.w = 1.0
    return m


def build_marker(node, x_m, y_m, yaw=None, mid=0, ns="r", col=(0,1,0)):
    from visualization_msgs.msg import Marker
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = node.get_clock().now().to_msg()
    m.ns, m.id, m.action = ns, mid, Marker.ADD
    if yaw is not None:
        m.type = Marker.ARROW
        q = yaw_to_q(yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        m.scale.x, m.scale.y, m.scale.z = 0.25, 0.08, 0.08
    else:
        m.type = Marker.SPHERE
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
    m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x_m), float(y_m), 0.0
    m.color.r, m.color.g, m.color.b, m.color.a = float(col[0]), float(col[1]), float(col[2]), 1.0
    m.lifetime.sec = 2
    return m


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=1)
    parser.add_argument("--dict", type=str, default="DICT_APRILTAG_36h11")
    parser.add_argument("--auto", action="store_true")
    parser.add_argument("--homography", type=str, default=None)
    parser.add_argument("--robot-id", type=int, default=0)
    parser.add_argument("--clearance", type=int, default=CLEARANCE_CM)
    args = parser.parse_args()

    rid = args.robot_id
    cam = args.camera
    dname = args.dict
    auto = args.auto
    hpath = args.homography
    clearance_cm = args.clearance

    DICTS = {
        'DICT_4X4_50': cv2.aruco.DICT_4X4_50, 'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
        'DICT_4X4_250': cv2.aruco.DICT_4X4_250, 'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
        'DICT_5X5_50': cv2.aruco.DICT_5X5_50, 'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
        'DICT_5X5_250': cv2.aruco.DICT_5X5_250, 'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
        'DICT_6X6_50': cv2.aruco.DICT_6X6_50, 'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        'DICT_6X6_250': cv2.aruco.DICT_6X6_250, 'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
        'DICT_7X7_50': cv2.aruco.DICT_7X7_50, 'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
        'DICT_7X7_250': cv2.aruco.DICT_7X7_250, 'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        'DICT_ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
        'DICT_APRILTAG_16h5': cv2.aruco.DICT_APRILTAG_16h5,
        'DICT_APRILTAG_25h9': cv2.aruco.DICT_APRILTAG_25h9,
        'DICT_APRILTAG_36h10': cv2.aruco.DICT_APRILTAG_36h10,
        'DICT_APRILTAG_36h11': cv2.aruco.DICT_APRILTAG_36h11,
    }
    if dname not in DICTS:
        print(f"Dict '{dname}' unknown"); sys.exit(1)
    chosen = [dname] if not auto else list(DICTS.keys())

    H = None
    if hpath:
        try:
            H = load_homography(hpath)
            print(f"[H] Loaded {hpath}  Field {FIELD_W}x{FIELD_H}m")
        except Exception as e:
            print(f"[H] Fail {e}"); sys.exit(1)
    else:
        print("[H] No homography")

    def p2cm(cx, cy):
        if H is None:
            return None, None
        r = cv2.perspectiveTransform(np.array([[[float(cx),float(cy)]]],dtype=np.float32), H)[0][0]
        return (r[0]-FIELD_W/2.0)*100.0, (r[1]-FIELD_H/2.0)*100.0

    print(f"Cam={cam}  RobotID={rid}  Dict={chosen[0]}  Clearance={clearance_cm}cm")
    print("Keys: Q=quit  +/-=clearance  A=auto  S=switch")

    cap = cv2.VideoCapture(cam)
    if not cap.isOpened():
        print("Camera fail"); sys.exit(1)
    fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # ROS init
    ROS = False
    node = None; tf_br = None; ex = None; spth = None
    pubs = {}
    try:
        import rclpy
        from geometry_msgs.msg import Point, Twist, PoseStamped, TransformStamped
        from std_msgs.msg import Int32MultiArray
        from visualization_msgs.msg import Marker
        from tf2_ros import TransformBroadcaster
        from rclpy.executors import SingleThreadedExecutor
        rclpy.init()
        node = rclpy.create_node("nexus_vis_lite")
        pubs = {
            'orange': node.create_publisher(Point, '/orange_position', 10),
            'yellow': node.create_publisher(Point, '/yellow_position', 10),
            'aruco': node.create_publisher(Int32MultiArray, '/aruco_markers', 10),
            'cmd': node.create_publisher(Twist, '/cmd_vel', 10),
            'rpose': node.create_publisher(PoseStamped, '/robot_pose', 10),
            'tpose': node.create_publisher(PoseStamped, '/target_yellow_pose', 10),
            'rmark': node.create_publisher(Marker, '/robot_marker', 10),
            'tmark': node.create_publisher(Marker, '/target_marker', 10),
        }
        tf_br = TransformBroadcaster(node)
        ROS = True
        ex = SingleThreadedExecutor(); ex.add_node(node)
        def _spin():
            while rclpy.ok():
                try:
                    ex.spin_once(timeout_sec=0.05)
                except Exception:
                    break
        spth = threading.Thread(target=_spin, daemon=True)
        spth.start()
        print("[ROS] OK")
    except Exception as e:
        print(f"[ROS] Skip {e}")

    dets = {}
    for n in chosen:
        ad = cv2.aruco.getPredefinedDictionary(DICTS[n])
        pr = cv2.aruco.DetectorParameters()
        pr.adaptiveThreshWinSizeMin, pr.adaptiveThreshWinSizeMax, pr.adaptiveThreshWinSizeStep = 3, 23, 10
        pr.minMarkerPerimeterRate = 0.02
        pr.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        pr.errorCorrectionRate = 0.8
        dets[n] = cv2.aruco.ArucoDetector(ad, pr)
    cidx, cdet, cname = 0, dets[chosen[0]], chosen[0]
    swcool = 0
    trackers = {}
    pf = AStarLite(fw, fh)

    # State
    last_robot_pub_t = 0.0
    last_target_pub_t = 0.0
    last_robot_cache = None
    last_tgt_cache = None
    last_cmd_t = 0.0
    pulse_active = False
    pulse_end = 0.0
    last_astar = None
    last_astar_t = 0.0
    running = True
    print_t = time.time()
    summary = {"astar":0, "rpub":0, "tpub":0, "frame":0}

    def should_pub_robot(now, new, old):
        if old is None:
            return True
        if now - last_robot_pub_t < ROBOT_POSE_MIN_PERIOD:
            return False
        dx = abs(new[0]-old[0]); dy = abs(new[1]-old[1])
        dyaw = abs(((new[2]-old[2]+np.pi)%(2*np.pi))-np.pi)
        return dx>POSE_POS_THRESH or dy>POSE_POS_THRESH or dyaw>POSE_YAW_THRESH

    def should_pub_target(now, new, old):
        if old is None:
            return True
        if now - last_target_pub_t >= TARGET_HEARTBEAT_PERIOD:
            return True
        if now - last_target_pub_t < TARGET_POSE_MIN_PERIOD:
            return False
        dx = abs(new[0]-old[0]); dy = abs(new[1]-old[1])
        return dx>POSE_POS_THRESH or dy>POSE_POS_THRESH

    def cleanup():
        nonlocal running
        if not running:
            return
        running = False
        cap.release()
        cv2.destroyAllWindows()
        if ROS and ex:
            try:
                ex.shutdown()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
        print("[Exit] Done")

    signal.signal(signal.SIGINT, lambda s,f: (cleanup(), sys.exit(0)))
    signal.signal(signal.SIGTERM, lambda s,f: (cleanup(), sys.exit(0)))

    while running:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.005); continue
        summary["frame"] += 1
        now = time.time()

        # --- Color detection ---
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Orange
        mo = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)
        mo = cv2.morphologyEx(mo, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))
        co, _ = cv2.findContours(mo, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_obs = []
        largest_orange = None
        for cnt in co:
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue
            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue
            cx, cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
            r = int((area/np.pi)**0.5 * 1.2)
            orange_obs.append((cx, cy, r))
            cv2.drawContours(frame, [cnt], -1, (0,165,255), 2)
            if largest_orange is None or area > largest_orange[3]:
                largest_orange = (cx, cy, area, r)

        # Yellow
        my = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
        my = cv2.morphologyEx(my, cv2.MORPH_CLOSE, np.ones((7,7),np.uint8))
        cy, _ = cv2.findContours(my, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_yellow = None
        for cnt in cy:
            area = cv2.contourArea(cnt)
            if area < 1500:
                continue
            x,y,w,h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,255), 2)
            cx, cy = x+w//2, y+h//2
            if largest_yellow is None or area > largest_yellow[2]:
                largest_yellow = (cx, cy, area)

        if ROS:
            if largest_orange:
                m = Point(); m.x=float(largest_orange[0]); m.y=float(largest_orange[1]); pubs['orange'].publish(m)
            if largest_yellow:
                m = Point(); m.x=float(largest_yellow[0]); m.y=float(largest_yellow[1]); pubs['yellow'].publish(m)

        # --- ArUco ---
        if auto and swcool > 0:
            swcool -= 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cdet.detectMarkers(gray)
        detected = set()
        rangle = None
        rpos = None
        if ids is not None and len(ids) > 0:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            adata = [chosen.index(cname)]
            for i, c in enumerate(corners):
                c = c[0]
                cx, cy = int(c[:,0].mean()), int(c[:,1].mean())
                mid = int(ids[i])
                detected.add(mid)
                ang = np.degrees(np.arctan2(c[1][1]-c[0][1], c[1][0]-c[0][0]))
                if mid == rid:
                    rangle = ang
                    rpos = (cx, cy)
                if mid not in trackers:
                    trackers[mid] = MarkerKalman()
                trackers[mid].predict()
                trackers[mid].correct(cx, cy)
                adata.extend([mid, cx, cy, int(ang*1000)])
                cv2.putText(frame, f"ID:{mid} {ang:.1f}d", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
            if ROS:
                m = Int32MultiArray(); m.data = adata; pubs['aruco'].publish(m)
            label = f"{cname} ID:{ids[0]}"
        elif auto and swcool == 0:
            cidx = (cidx+1)%len(chosen); cname = chosen[cidx]; cdet = dets[cname]; swcool = 15
            label = f"Try:{cname}"
        else:
            label = cname if not auto else f"Try:{cname}"

        for mid in list(trackers.keys()):
            if mid not in detected:
                tr = trackers[mid]
                if tr.initialized:
                    tr.mark_missed()
                    px, py = tr.predict()
                    if not tr.is_lost():
                        cv2.circle(frame, (int(px),int(py)), 7, (0,0,255), 2)
                    else:
                        del trackers[mid]

        # --- A* (throttled) ---
        do_astar = (rangle is not None and rpos is not None and largest_yellow is not None
                    and (now - last_astar_t) >= (1.0/AStar_MAX_HZ))
        if do_astar:
            last_astar_t = now
            mx, my = rpos
            yx, yy = largest_yellow[0], largest_yellow[1]
            if orange_obs:
                raw = pf.find_path((mx,my), (yx,yy), orange_obs, inflate=max(1, int(clearance_cm*fw/(FIELD_W*100.0)/CELL_SIZE)))
                if raw:
                    last_astar = pf.simplify(raw)
                else:
                    last_astar = None
            summary["astar"] += 1

        # --- Publish Robot Pose + TF + Marker ---
        if ROS:
            if rangle is not None and rpos is not None:
                rx, ry = p2cm(rpos[0], rpos[1])
                if rx is not None:
                    yaw = np.radians(float(rangle))
                    nr = (rx, ry, yaw)
                    if should_pub_robot(now, nr, last_robot_cache):
                        pubs['rpose'].publish(build_pose(node, rx, ry, yaw, "map"))
                        xm, ym = rx/100.0, ry/100.0
                        t = TransformStamped()
                        t.header.stamp = node.get_clock().now().to_msg()
                        t.header.frame_id = "map"; t.child_frame_id = "base_link"
                        t.transform.translation.x, t.transform.translation.y = float(xm), float(ym)
                        t.transform.translation.z = 0.0
                        q = yaw_to_q(yaw)
                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
                        tf_br.sendTransform(t)
                        pubs['rmark'].publish(build_marker(node, xm, ym, yaw, 0, "robot", (0,0,1)))
                        last_robot_cache = nr
                        last_robot_pub_t = now
                        summary["rpub"] += 1

        # --- Publish Target Yellow (FIX: throttle terpisah, selalu publish pertama kali) ---
        if ROS and largest_yellow is not None:
            tx, ty = p2cm(largest_yellow[0], largest_yellow[1])
            if tx is not None:
                nt = (tx, ty)
                if should_pub_target(now, nt, last_tgt_cache):
                    pubs['tpose'].publish(build_pose(node, tx, ty, None, "map"))
                    pubs['tmark'].publish(build_marker(node, tx/100.0, ty/100.0, None, 1, "target", (1,1,0)))
                    last_tgt_cache = nt
                    last_target_pub_t = now
                    summary["tpub"] += 1

        # --- Pulse cmd_vel ---
        if ROS:
            if pulse_active and now >= pulse_end:
                pubs['cmd'].publish(Twist())
                pulse_active = False
                last_cmd_t = now
            elif not pulse_active and (now - last_cmd_t) >= CMD_VEL_PERIOD:
                tw = Twist()
                if rangle is not None and rpos is not None and largest_yellow is not None:
                    mx, my = rpos
                    yx, yy = largest_yellow[0], largest_yellow[1]
                    bear = None
                    if last_astar and len(last_astar) >= 2:
                        bear = float(np.degrees(np.arctan2(last_astar[1][1]-my, last_astar[1][0]-mx)))
                    if bear is None:
                        bear = float(np.degrees(np.arctan2(yy-my, yx-mx)))
                    err = ((float(rangle)-bear+180)%360)-180
                    if abs(err) > YAW_DEADZONE_DEG:
                        tw.angular.z = -MAX_ANGULAR if err>0 else MAX_ANGULAR
                        tw.linear.x = 0.0
                        pubs['cmd'].publish(tw)
                        pulse_active = True
                        pulse_end = now + PULSE_DURATION
                    else:
                        tw.angular.z = 0.0
                        tw.linear.x = FORWARD_SPEED
                        pubs['cmd'].publish(tw)
                        last_cmd_t = now
                else:
                    last_cmd_t = now

        # --- Draw UI ---
        if auto:
            cv2.putText(frame, label, (10,25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        cv2.putText(frame, f"Clr:{clearance_cm}cm  A*:{summary['astar']}  Rpub:{summary['rpub']}  Tpub:{summary['tpub']}", (10, fh-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)

        # === PATH MAGENTA SELALU DIGAMBAR KALAU ADA ===
        if last_astar is not None and len(last_astar) >= 2:
            for i in range(len(last_astar)-1):
                p1 = (int(last_astar[i][0]), int(last_astar[i][1]))
                p2 = (int(last_astar[i+1][0]), int(last_astar[i+1][1]))
                cv2.line(frame, p1, p2, (255,0,255), 2)
            # waypoint target
            cv2.circle(frame, (int(last_astar[-1][0]), int(last_astar[-1][1])), 5, (255,0,255), -1)
            # next waypoint
            if len(last_astar) >= 2:
                cv2.circle(frame, (int(last_astar[1][0]), int(last_astar[1][1])), 5, (0,255,255), -1)

        cv2.imshow("ArUco Detection", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key in (ord('+'), ord('=')):
            clearance_cm = min(30, clearance_cm+1)
        elif key == ord('-'):
            clearance_cm = max(1, clearance_cm-1)
        elif key == ord('a'):
            auto = not auto
        elif key == ord('s'):
            cidx = (cidx+1)%len(chosen); cname = chosen[cidx]; cdet = dets[cname]

        if now - print_t >= PRINT_INTERVAL:
            fps = summary['frame']/(now-print_t)
            print(f"[SUM] fps={fps:.1f}  A*={summary['astar']}  Rpub={summary['rpub']}  Tpub={summary['tpub']}  Clr={clearance_cm}cm")
            summary = {"astar":0, "rpub":0, "tpub":0, "frame":0}
            print_t = now

    cleanup()


if __name__ == '__main__':
    main()
