#!/usr/bin/env bash
set -e

PROJECT_ROOT="${PROJECT_ROOT:-$HOME/platform_control_project}"
ROS_UNDERLAY="${ROS_UNDERLAY:-$HOME/ros2_jazzy}"
ROS_SETUP="${ROS_UNDERLAY}/install/setup.bash"

export LANG=en_US.UTF-8
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=jazzy
export MAKEFLAGS="${MAKEFLAGS:--j2}"

sudo apt update
sudo apt install -y ca-certificates curl wget gnupg lsb-release git

sudo rm -f /etc/apt/sources.list.d/ros2*.sources /etc/apt/sources.list.d/ros2*.list 2>/dev/null || true
sudo apt-get remove -y ros2-apt-source 2>/dev/null || true

ROS_APT_SOURCE_VERSION="$(curl -sS https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | sed -n 's/.*"tag_name": *"\([^"]*\)".*/\1/p')"
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.bookworm_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get install -f -y

sudo apt update
sudo apt-get remove -y python3-catkin-pkg python3-rospkg 2>/dev/null || true
sudo apt install -y -o Dpkg::Options::="--force-overwrite" python3-catkin-pkg-modules python3-rospkg-modules 2>/dev/null || true
sudo dpkg --configure -a 2>/dev/null || true
sudo apt-get -f install -y
sudo apt upgrade -y

sudo apt install -y \
  python3-pip python3-venv python3-opencv python3-pyqt5 python3-pyqt5.qtsvg \
  pyqt5-dev qtbase5-dev \
  build-essential cmake python3-dev python3-setuptools python3-empy python3-numpy \
  locales \
  libasio-dev libtinyxml2-dev libcunit1-dev libssl-dev pkg-config \
  libcurl4-openssl-dev libyaml-cpp-dev libeigen3-dev \
  libbullet-dev libopencv-dev libzmq3-dev

sudo sed -i 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen 2>/dev/null || true
sudo locale-gen en_US.UTF-8 2>/dev/null || true

sudo apt install -y ros-dev-tools \
  || sudo apt install -y -o Dpkg::Options::="--force-overwrite" ros-dev-tools \
  || true

export PATH="$HOME/.local/bin:$PATH"
python3 -m pip install --user --break-system-packages \
  colcon-common-extensions vcstool rosdep rosdistro catkin-pkg sip 2>/dev/null || true

if [ ! -d "$PROJECT_ROOT" ]; then
  git clone https://github.com/AndreiPurits/platform_control_project.git "$PROJECT_ROOT"
else
  git -C "$PROJECT_ROOT" pull --ff-only
fi
cd "$PROJECT_ROOT"

sudo mkdir -p /etc/ros/rosdep/sources.list.d
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list \
  -o /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update

if [ ! -f "$ROS_SETUP" ]; then
  echo ""
  echo ">>> Сборка ROS 2 Jazzy из исходников под Debian (arm64). Это долго (часы), зато без Ubuntu."
  echo ">>> Underlay: $ROS_UNDERLAY"
  echo ""
  mkdir -p "${ROS_UNDERLAY}/src"
  curl -fL -o "${ROS_UNDERLAY}/ros2.repos" \
    https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
  vcs import "${ROS_UNDERLAY}/src" < "${ROS_UNDERLAY}/ros2.repos"
  cd "${ROS_UNDERLAY}"
  python3 -m pip install --user --break-system-packages sip 2>/dev/null || true
  ROSDEP_SKIP="ament_python ament_cmake python3-pyqt5 python3-qt5-bindings python3-sip sip python3-sip-dev fastcdr rti-connext-dds-6.0.1 rti-connext-dds-7.3.0 urdfdom_headers python3-vcstool python3-catkin-pkg-modules python3-rosdistro-modules"
  rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy --skip-keys "$ROSDEP_SKIP"
  echo ""
  echo ">>> Пропуск rqt/rviz/qt_gui (свой GUI на PyQt5+rclpy; без SIP/qt_gui_cpp)."
  export PATH="${HOME}/.local/bin:${PATH}"
  export MAKEFLAGS="${MAKEFLAGS:--j2}"
  GUI_SKIP=$( (colcon list | grep -iE 'rqt|rviz|qt_gui|python_qt|tango_icons|interactive_markers' || true) | xargs )
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release ${GUI_SKIP:+--packages-skip $GUI_SKIP}
fi
cd "$PROJECT_ROOT/ros2_ws"
source "$ROS_SETUP"
ROSDEP_SKIP_WS="ament_python ament_cmake python3-pyqt5 python3-qt5-bindings python3-sip sip python3-sip-dev python3-vcstool python3-catkin-pkg-modules python3-rosdistro-modules"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy --skip-keys "$ROSDEP_SKIP_WS"
# --paths src не рекурсивно обходит подкаталоги с package.xml; --base-paths — да (ament_cmake/ament_python пакеты в src/<name>/).
colcon build --symlink-install --base-paths "$PROJECT_ROOT/ros2_ws/src"

cd "$PROJECT_ROOT"
rm -rf .venv
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -q -U pip 2>/dev/null || true
python -m pip install pyserial numpy
python -m pip install onnxruntime 2>/dev/null || true
deactivate

cat > "$HOME/run_rover.sh" <<EOF
#!/usr/bin/env bash
set -e
source "${ROS_UNDERLAY}/install/setup.bash"
source "${PROJECT_ROOT}/ros2_ws/install/setup.bash"
source "${PROJECT_ROOT}/.venv/bin/activate"
exec ros2 launch lidar_guard bringup.launch.py
EOF
chmod +x "$HOME/run_rover.sh"

cat > "$HOME/.platform_control_rosenv.sh" <<EOF
export PLATFORM_CONTROL_PROJECT="${PROJECT_ROOT}"
export ROS_UNDERLAY="${ROS_UNDERLAY}"
rosproj() {
  source "\$ROS_UNDERLAY/install/setup.bash"
  source "\$PLATFORM_CONTROL_PROJECT/ros2_ws/install/setup.bash"
  source "\$PLATFORM_CONTROL_PROJECT/.venv/bin/activate"
  cd "\$PLATFORM_CONTROL_PROJECT"
  echo "ROS Jazzy (underlay \$ROS_UNDERLAY) + проект"
}
EOF

touch "$HOME/.bashrc"
sed -i '/\\.platform_control_rosenv\\.sh/d' "$HOME/.bashrc" 2>/dev/null || true
echo "source \"$HOME/.platform_control_rosenv.sh\"" >> "$HOME/.bashrc"

mkdir -p "$HOME/Desktop"
cat > "$HOME/Desktop/Run_Rover.desktop" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Run Rover
Comment=Launch rover stack
Exec=bash -lc "$HOME/run_rover.sh"
Terminal=true
Icon=utilities-terminal
Categories=Utility;
EOF
chmod +x "$HOME/Desktop/Run_Rover.desktop"

cat > "$HOME/Desktop/Platform_Project.desktop" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Папка проекта
Comment=Открыть platform_control_project
Exec=sh -c "xdg-open \"${PROJECT_ROOT}\""
Icon=folder
Categories=Utility;
EOF
chmod +x "$HOME/Desktop/Platform_Project.desktop"

echo ""
echo "=== Готово: Raspberry Pi OS (Debian), ROS 2 Jazzy собран в $ROS_UNDERLAY ==="
echo "Проект:  $PROJECT_ROOT"
echo "Запуск:  $HOME/run_rover.sh"
echo "Шелл:    rosproj"
echo "Пересобрать underlay с нуля: rm -rf \"$ROS_UNDERLAY\" && bash install_rpi5.sh"
