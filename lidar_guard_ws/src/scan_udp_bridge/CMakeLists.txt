cmake_minimum_required(VERSION 3.5)
project(scan_udp_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Установим скрипты как исполняемые
install(PROGRAMS
  scan_udp_bridge/scan_to_udp.py
  scan_udp_bridge/udp_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Установим launch-файлы
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
