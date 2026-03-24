SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../paxi_ws"

colcon build --cmake-args -DBUILD_TESTING=true --packages-ignore slamkit_ros2 sllidar_ros2

colcon test --packages-skip slamkit_ros2 sllidar_ros2
