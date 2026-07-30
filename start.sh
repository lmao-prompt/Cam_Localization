#!/bin/bash

# ==================== PATHS ====================
URDF_PATH="$HOME/Downloads/Nexus_Gazebo/nexus_robot.urdf"
SDF_PATH="$HOME/Downloads/Nexus_Gazebo/nexus.urdf"

# <-- TAMBAHAN: Path ke script spawn
SPAWN_OBS_SCRIPT="$HOME/Downloads/Nexus_Gazebo/src/spawn_obs.py"
SPAWN_ROBOT_SCRIPT="$HOME/Downloads/Nexus_Gazebo/src/spawn_robot.py"

HOMOGRAPHY_PATH="$HOME/Downloads/Nexus_Gazebo/src/homography.json"
VISION_SCRIPT="$HOME/Downloads/Nexus_Gazebo/src/testaruco.py"
RVIZ_CONFIG="$HOME/Downloads/Nexus_Gazebo/nexus.rviz"

CAMERA_INDEX=2
ARUCO_DICT="DICT_APRILTAG_36h11"
ROBOT_ID=3
CLEARANCE_CM=10

source /opt/ros/jazzy/setup.bash
source ~/Downloads/Nexus_Gazebo/install/setup.bash 2>/dev/null || true

echo "=== PRE-FLIGHT CHECK ==="

if ! command -v gz &> /dev/null; then
    echo "❌ gz sim tidak ditemukan!"
    exit 1
fi
echo "✅ gz sim ditemukan"

if [ ! -f "$SDF_PATH" ]; then
    echo "❌ SDF world tidak ditemukan di: $SDF_PATH"
    exit 1
fi
echo "✅ SDF world ditemukan"

# Cek file eksternal di dalam SDF
STL_REF="/home/$USER/Documents/Nexus_Gazebo/test_ukuran_base.stl"
PNG_REF="/home/$USER/Downloads/Nexus_Gazebo/materials/textures/aruco_0.png"
[ ! -f "$STL_REF" ] && echo "⚠️  WARNING: STL tidak ditemukan di $STL_REF"
[ ! -f "$PNG_REF" ] && echo "⚠️  WARNING: Texture ArUco tidak ditemukan di $PNG_REF"

# ==================== KILL ZOMBIE ====================
echo "🧹 Kill instance Gazebo lama..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gzclient" 2>/dev/null || true
sleep 2

# ==================== 1. GAZEBO (SDF World) ====================
echo ""
echo "[1/6] Starting Gazebo: gz sim -r $SDF_PATH"

gz sim -r -v 4 "$SDF_PATH" 2>&1 | tee /tmp/gazebo_debug.log &
GAZEBO_PID=$!
sleep 8

if ! ps -p $GAZEBO_PID > /dev/null; then
    echo "❌ Gazebo crash! Last error:"
    tail -n 20 /tmp/gazebo_debug.log
    exit 1
fi
echo "✅ Gazebo jalan (PID: $GAZEBO_PID)"

# ==================== 2. SPAWN ROBOT & OBSTACLES ====================
echo "[2/6] Spawning entities..."

if [ -f "$SPAWN_ROBOT_SCRIPT" ]; then
    echo "    → spawn_robot.py"
    python3 "$SPAWN_ROBOT_SCRIPT" &
    SPAWN_ROBOT_PID=$!
else
    echo "    ⚠️  $SPAWN_ROBOT_SCRIPT tidak ditemukan, skip."
    SPAWN_ROBOT_PID=""
fi

if [ -f "$SPAWN_OBS_SCRIPT" ]; then
    echo "    → spawn_obs.py"
    python3 "$SPAWN_OBS_SCRIPT" &
    SPAWN_OBS_PID=$!
else
    echo "    ⚠️  $SPAWN_OBS_SCRIPT tidak ditemukan, skip."
    SPAWN_OBS_PID=""
fi

sleep 3

# ==================== 3. ROBOT STATE PUBLISHER ====================
if [ -f "$URDF_PATH" ]; then
    echo "[3/6] Starting robot_state_publisher..."
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p robot_description:="$(cat "$URDF_PATH")" &
    RSP_PID=$!
    sleep 2
else
    echo "[3/6] WARNING: URDF tidak ditemukan di $URDF_PATH"
    RSP_PID=""
fi

# ==================== 4. VISION NAVIGATION ====================
echo "[4/6] Starting vision navigation..."
python3 "$VISION_SCRIPT" \
    --dict "$ARUCO_DICT" \
    --robot-id "$ROBOT_ID" \
    --homography "$HOMOGRAPHY_PATH" \
    --camera "$CAMERA_INDEX" \
    --clearance "$CLEARANCE_CM" &
VISION_PID=$!
sleep 3

# ==================== 5. RVIZ ====================
echo "[5/6] Starting RViz..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
    RVIZ_PID=$!
else
    rviz2 &
    RVIZ_PID=$!
fi

echo "[6/6] ✅ All systems up! Press Ctrl+C to shutdown."

# ==================== CLEANUP ====================
cleanup() {
    echo ""
    echo "[SHUTDOWN] Stopping all nodes..."
    [ -n "$RVIZ_PID" ]       && kill $RVIZ_PID 2>/dev/null || true
    [ -n "$VISION_PID" ]     && kill $VISION_PID 2>/dev/null || true
    [ -n "$RSP_PID" ]        && kill $RSP_PID 2>/dev/null || true
    [ -n "$SPAWN_OBS_PID" ]  && kill $SPAWN_OBS_PID 2>/dev/null || true
    [ -n "$SPAWN_ROBOT_PID" ]&& kill $SPAWN_ROBOT_PID 2>/dev/null || true
    [ -n "$GAZEBO_PID" ]     && kill $GAZEBO_PID 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "gzserver" 2>/dev/null || true
    pkill -9 -f "gzclient" 2>/dev/null || true
    wait
    echo "[SHUTDOWN] Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

wait