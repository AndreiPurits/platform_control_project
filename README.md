# Platform Control Project — установка на Raspberry Pi

Инструкция только для развёртывания системы на «чистой» Raspberry Pi OS.

## 1. Образ системы

1. Откройте сайт Raspberry Pi: [Software](https://www.raspberrypi.com/software/) — скачайте **Raspberry Pi Imager** (Windows / macOS / Linux).
2. Установите на карту памяти образ **Raspberry Pi OS (64-bit)** на базе **Debian Bookworm** (рекомендуется для Pi 4 / Pi 5).
3. Вставьте карту в Raspberry Pi, выполните первичную настройку (язык, сеть, пользователь).

Подробности: [Operating systems](https://www.raspberrypi.com/software/operating-systems/).

## 2. Загрузочный скрипт (один раз в терминале)

На Raspberry Pi откройте терминал и выполните **одну** из команд ниже.

### Вариант A — скачать скрипт с GitHub и запустить

```bash
curl -fsSL https://raw.githubusercontent.com/AndreiPurits/platform_control_project/main/install_rpi5.sh -o /tmp/install_rpi5.sh
bash /tmp/install_rpi5.sh
```

### Вариант B — клонировать репозиторий и запустить локальный скрипт

```bash
git clone https://github.com/AndreiPurits/platform_control_project.git "$HOME/platform_control_project"
bash "$HOME/platform_control_project/install_rpi5.sh"
```

Скрипт:

- ставит зависимости и (при необходимости) собирает ROS 2 Jazzy в `~/ros2_jazzy`;
- клонирует/обновляет проект в `~/platform_control_project`;
- собирает overlay `ros2_ws`, создаёт venv, настраивает `~/run_rover.sh` и ярлыки на рабочем столе.

Первая установка с **сборкой ROS из исходников** может занять **много часов**. Повторный запуск без удаления `~/ros2_jazzy` обновляет только проект и overlay.

## 3. Запуск после установки

```bash
~/run_rover.sh
```

или в интерактивной оболочке:

```bash
rosproj
ros2 launch lidar_guard bringup.launch.py
```

## 4. Что лежит в репозитории (важные каталоги)

- `lidar_guard_ws/src/lidar_guard/ui/idle_images/` — фоны и иконки режимов для экрана Idle (PNG).
- `lidar_guard_ws/src/lidar_guard/ui/maps_repo/` — примеры карт и графов (PNG, JSON, скрипты подготовки).

Репозиторий: [github.com/AndreiPurits/platform_control_project](https://github.com/AndreiPurits/platform_control_project)
