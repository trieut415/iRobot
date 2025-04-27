#!/usr/bin/env bash
set -e

# -----------------------------------------------------------------------------
# install_dependencies_wsl.sh — Install ROS 2 Iron & webots_ros2 into current directory workspace
# -----------------------------------------------------------------------------

# 0. Save current working directory
WORKSPACE_DIR="$(pwd)"

# 1. Verify we’re running under WSL2
if grep -qi microsoft /proc/version; then
  echo "Detected WSL2 environment"
else
  echo "ERROR: This script must be run inside WSL2 Ubuntu. Aborting."
  return 1 2>/dev/null || exit 1
fi

# 2. Configure locale
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 3. Enable Universe & install core dependencies
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  curl \
  gnupg \
  lsb-release

# 4. Add the ROS 2 apt repository
sudo mkdir -p /etc/apt/keyrings
curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) \
   signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
   http://packages.ros.org/ros2/ubuntu \
   $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 5. Install ROS 2 Iron desktop
sudo apt update
sudo apt install -y ros-iron-desktop

# 6. Source ROS 2 on every new shell
if ! grep -q "source /opt/ros/iron/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc
fi

# 7. Install ROS tooling and colcon
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-rosinstall-generator \
  python3-vcstool

# 8. Initialize rosdep
sudo rosdep init || true
rosdep update

# 9. Install extra ROS 2 dependencies needed by webots_ros2
sudo apt install -y \
  ros-iron-ackermann-msgs \
  ros-iron-tf-transformations \
  ros-iron-robot-localization \
  ros-iron-controller-manager \
  ros-iron-vision-msgs \
  ros-iron-xacro \
  ros-iron-joint-state-broadcaster \
  ros-iron-laser-filters \
  ros-iron-joint-trajectory-controller \
  ros-iron-diff-drive-controller

# 10. Setup current directory as workspace
echo
echo "📂  Setting up workspace in $WORKSPACE_DIR"

cd "$WORKSPACE_DIR"

# Ensure src folder exists
mkdir -p src

# Move p2p_ros_package into src if not already
if [ -d p2p_ros_package ] && [ ! -d src/p2p_ros_package ]; then
  echo "Moving p2p_ros_package into src/"
  mv p2p_ros_package src/
fi

# Clone webots_ros2 into src if missing
if [ ! -d src/webots_ros2 ]; then
  echo "Cloning webots_ros2 into src/"
  cd src
  git clone --recurse-submodules https://github.com/cyberbotics/webots_ros2.git
  cd ..
else
  echo "webots_ros2 already exists, pulling latest..."
  cd src/webots_ros2
  git pull
  git submodule update --init --recursive
  cd ../..
fi

# 11. Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro iron -y

# 11.5 Create ROS 2 resource index file
echo
echo "🛠  Setting up resource index for p2p_ros_package..."
mkdir -p "$WORKSPACE_DIR/resource"
echo "p2p_ros_package" > "$WORKSPACE_DIR/resource/p2p_ros_package"


# 12. Build the workspace
echo
echo "🔨  Building workspace at $WORKSPACE_DIR..."
colcon build --symlink-install

# 13. Source workspace on every new shell
if ! grep -q "source $WORKSPACE_DIR/install/local_setup.bash" ~/.bashrc; then
  echo "source $WORKSPACE_DIR/install/local_setup.bash" >> ~/.bashrc
fi

# 14. Source the newly built workspace immediately
source "$WORKSPACE_DIR/install/local_setup.bash"

echo
echo "✅  ROS 2 Iron + your workspace setup complete!"
echo "✅  Workspace environment loaded!"
echo "   • You can now immediately run:"
echo "     ros2 launch p2p_ros_package p2p_launch.py"
