#!/usr/bin/env bash

# =============================================================================
# setup.sh — Source this script to bootstrap ROS 2 Jazzy Jalisco from source on macOS
# Usage:   source setup.sh
# Requires: macOS 10.14+ (Mojave or later)
# =============================================================================


# Ensure the script is sourced, not executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "Please source this script instead of running it:"
  echo "  source setup.sh"
  return 1
fi

# --- 1. Check SIP status -----------------------------------------------------
if csrutil status | grep -q "enabled"; then
  echo "System Integrity Protection (SIP) is ENABLED."
  echo "Disable SIP before proceeding:"
  echo "  1. Reboot into Recovery (⌘+R on Intel; hold Power on Apple Silicon)"
  echo "  2. Open Terminal and run: csrutil disable"
  echo "  3. Reboot normally"
  return 1
else
  echo "SIP is already DISABLED."
fi

# --- 2. Install Homebrew packages --------------------------------------------
echo "Installing Homebrew dependencies..."
brew update
brew install asio assimp bison bullet cmake console_bridge cppcheck cunit eigen \
  freetype graphviz opencv openssl orocos-kdl pcre poco pyqt@5 python@3.11 qt@5 \
  sip spdlog tinyxml2 lz4 yaml-cpp pybind11 eigen cpprestsdk yaml-cpp eigen lz4 zstd ogre3d qt5 pybind11

# Prepend Python@3.11 so venv includes distutils
export PATH="$(brew --prefix python@3.11)/bin:$PATH"

# --- 3. Configure environment variables --------------------------------------
export OPENSSL_ROOT_DIR="$(brew --prefix openssl)"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH}:$(brew --prefix qt@5)"
export PATH="$PATH:$(brew --prefix qt@5)/bin"

# --- 4. Setup Python & pip tools in a dedicated venv -------------------------
# Put the venv outside the workspace to avoid colcon scanning it
VENV_DIR="$HOME/ros2_jazzy_venv"
echo "Creating Python virtual environment at $VENV_DIR..."
python3.11 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

echo "Upgrading pip..."
pip install --upgrade pip

# Install Cython early to satisfy downstream builds
pip install Cython

# Build pygraphviz with correct Graphviz headers
echo "Installing pygraphviz with Graphviz headers..."
CFLAGS="-I$(brew --prefix graphviz)/include" \
LDFLAGS="-L$(brew --prefix graphviz)/lib" \
pip install --no-binary pygraphviz pygraphviz || {
  echo "ERROR: pygraphviz build failed. Ensure Graphviz is installed and headers exist." >&2
  return 1
}

# Install remaining Python build tools
echo "Installing ROS 2 build tools (vcstool, colcon, rosdep, etc.)..."
pip install \
  argcomplete catkin_pkg colcon-common-extensions coverage \
  cryptography empy flake8 flake8-blind-except==0.1.1 \
  flake8-builtins flake8-class-newline flake8-comprehensions \
  flake8-deprecated flake8-docstrings flake8-import-order \
  flake8-quotes importlib-metadata packaging jsonschema lark==1.1.1 \
  lxml matplotlib mock mypy==0.931 netifaces nose pep8 pytest pytest-mock \
  psutil pydocstyle pydot pyparsing==2.4.7 pytest-cov rosdep rosdistro setuptools==59.6.0 vcstool

# --- 5. Initialize rosdep (once) ---------------------------------------------
if ! command -v rosdep >/dev/null 2>&1; then
  echo "rosdep not found—ensure venv is active and re-source this script." >&2
  return 1
fi
if [ ! -f /usr/local/etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# --- 6. Create & populate workspace ------------------------------------------
WORKSPACE_DIR="$HOME/ros2_jazzy"
echo "Creating ROS 2 workspace at $WORKSPACE_DIR..."
mkdir -p "$WORKSPACE_DIR/src"
cd "$WORKSPACE_DIR"

echo "Importing ROS 2 repositories..."
curl -sL https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos | vcs import src

# Pre-install pip packages for keys missing in macOS rosdep definitions
echo "Pre-installing Python packages to satisfy missing rosdep keys..."
pip install numpy lark-parser PyQt5 pytest-timeout cryptography argcomplete \
  pybind11 packaging pytest-cov jsonschema flake8-import-order flake8-builtins flake8-comprehensions flake8-docstrings

# rosdep install with skipped keys for unsupported dependencies on macOS
echo "Installing package dependencies (skipping macOS-unsupported keys)..."
SKIP_KEYS="python3-rosdistro-modules python3-lark-parser python3-qt5-bindings python3-numpy python3-pkg-resources python3-pytest-timeout python3-pytest-cov python3-packaging liblttng-ctl-dev liblttng-ust-dev lttng-tools file rti-connext-dds-6.0.1 pybind11-dev python3-cryptography python3-argcomplete python3-cairo python3-flake8-quotes python3-flake8-import-order python3-flake8-builtins python3-flake8-comprehensions python3-flake8-docstrings liblz4 liblz4-dev libqt5-svg libqt5-svg-dev libfreetype6 libfreetype-dev libyaml libyaml-dev python3-jsonschema python3-dev libsqlite3-dev libx11-dev libzstd-dev benchmark openssl acl python3-pykdl"
rosdep install --from-paths src --ignore-src --skip-keys "$SKIP_KEYS" -y --rosdistro jazzy

# --- 7. Build ROS 2 with colcon -----------------------------------------------
echo "Building ROS 2 (this can take 30+ minutes)..."
colcon build --symlink-install --packages-skip-by-dep python_qt_binding \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5

# --- 8. Final instructions ---------------------------------------------------
echo ""
echo "ROS 2 Jazzy Jalisco build complete!"
echo "To start using your workspace:"
echo "  source $WORKSPACE_DIR/install/setup.zsh  # for zsh users"
echo "  source $WORKSPACE_DIR/install/setup.bash # for bash users"
