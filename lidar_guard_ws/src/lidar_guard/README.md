lidar_guard: ROS2 узел безопасности для лидара

Что это
Этот пакет подписывается на /scan (sensor_msgs/LaserScan), вычисляет минимальную дистанцию в секторе и публикует флаг /safety_stop (std_msgs/Bool). Есть синтетический издатель сканов для отладки без железа.

Требования
• Ubuntu 22.04
• ROS2 Humble
• colcon
• Python 3.10+

Установка
1) mkdir -p ~/lidar_guard_ws/src
2) распаковать архив в ~/lidar_guard_ws
3) cd ~/lidar_guard_ws
4) colcon build
5) source install/setup.bash

Запуск демо без железа
ros2 launch lidar_guard guard_demo.launch.py

Что должно быть видно
• топик /safety_stop с True при помехе
• топики /guard/state и /guard/min_distance

Запуск только узла охраны
ros2 run lidar_guard guard_node

Параметры узла guard_node
• stop_distance_m
• caution_distance_m
• fov_guard_deg
• hysteresis_m
• print_debug

Переключение на реальный лидар
1) Установить драйвер лидара, чтобы он публиковал /scan (LaserScan)
2) Запустить только guard_node и настроить параметры
3) Проверить fov и единицы измерения
Пример
ros2 run lidar_guard guard_node --ros-args -p stop_distance_m:=1.8 -p caution_distance_m:=2.6 -p fov_guard_deg:=120 -p print_debug:=true

Тесты
cd ~/lidar_guard_ws && colcon test

Безопасность
• проверить аппаратный стоп
• валидировать пороги в статике
• проверить таймаут поступления /scan
