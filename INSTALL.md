# Установка и сборка `forklift_ws`

Инструкция рассчитана на Ubuntu 24.04 и ROS 2 Jazzy. Все команды используют
переменную `FORKLIFT_WS`, поэтому путь не зависит от имени пользователя.

## 1. Подготовка окружения

```zsh
export FORKLIFT_WS="${FORKLIFT_WS:-$HOME/forklift_ws}"
source /opt/ros/jazzy/setup.zsh
cd "$FORKLIFT_WS"
```

Если workspace расположен в другом месте, задайте переменную явно:

```zsh
export FORKLIFT_WS=/path/to/forklift_ws
```

## 2. Системные зависимости

```zsh
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config zstd libpcl-dev \
  python3-numpy python3-pil python3-yaml \
  ros-jazzy-colcon-common-extensions \
  ros-jazzy-mola-lidar-odometry \
  ros-jazzy-mola-metric-maps \
  ros-jazzy-mola-sm-loop-closure \
  ros-jazzy-mola-state-estimation \
  ros-jazzy-mola-state-estimation-smoother \
  ros-jazzy-robot-localization \
  ros-jazzy-pcl-conversions \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-nav2-map-server \
  ros-jazzy-rviz2
```

Проверка ROS-зависимостей:

```zsh
source /opt/ros/jazzy/setup.zsh
rosdep update
rosdep install --from-paths "$FORKLIFT_WS/src" --ignore-src -r -y
```

## 3. Livox SDK2

`livox_ros_driver2` требует установленную библиотеку Livox SDK2. Установите
SDK2 по официальной инструкции:

<https://github.com/Livox-SDK/Livox-SDK2>

После установки должны существовать библиотека и заголовки:

```zsh
test -f /usr/local/lib/liblivox_lidar_sdk_shared.so
test -f /usr/local/include/livox_lidar_api.h
```

## 4. Сборка Livox-драйвера

Драйвер использует собственный `build.sh`, поэтому его нужно собирать первым:

```zsh
source /opt/ros/jazzy/setup.zsh
cd "$FORKLIFT_WS/src/livox_ros_driver2"
./build.sh jazzy
```

Скрипт драйвера очищает сгенерированные `build/`, `install/` и `devel/`
внутри workspace. Не запускайте его после сборки остальных пакетов без
последующей пересборки workspace.

## 5. Сборка зависимостей GLIM

GLIM использует `gtsam_points` и `iridescence`. Их нужно собрать локально
внутри `forklift_ws`, чтобы не подхватить несовместимые копии из других
рабочих пространств:

```zsh
source /opt/ros/jazzy/setup.zsh
cd "$FORKLIFT_WS"
colcon build --packages-select iridescence gtsam_points --symlink-install
```

## 6. Полная сборка рабочего пространства

```zsh
source /opt/ros/jazzy/setup.zsh
cd "$FORKLIFT_WS"

# Все пакеты рабочего пространства, кроме Livox:
colcon build --symlink-install --packages-skip livox_ros_driver2

# Зарегистрировать Livox в общем install рабочего пространства:
colcon build --packages-select livox_ros_driver2 \
  --symlink-install \
  --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy

# GLIM собирается с локальными зависимостями forklift_ws:
colcon build --packages-select glim glim_ros glim_bringup \
  --symlink-install \
  --cmake-args \
    -Dgtsam_points_DIR="$FORKLIFT_WS/install/gtsam_points/lib/cmake/gtsam_points" \
    -DIridescence_DIR="$FORKLIFT_WS/install/iridescence/lib/cmake/iridescence"

source "$FORKLIFT_WS/install/setup.zsh"
```

Для текущей конфигурации без CUDA runtime GLIM использует CPU-модули.

После сборки должны находиться все пакеты из `src`, включая GLIM,
`lidar_localization_mid360` и `livox_ros_driver2`.

Проверка установленных пакетов:

```zsh
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix glim_ros
ros2 pkg prefix lidar_localization_mid360
```

Все три команды должны вернуть пути внутри `FORKLIFT_WS/install`.

## 6. Дополнительные зависимости FAST-LIO

Режимы `fastlio_*` требуют пакеты `fast_lio` и `livox_to_pointcloud2`.
Они должны быть доступны в `FORKLIFT_WS/src` или в ROS underlay. Если эти
пакеты находятся в другом workspace, его нужно подключить до `FORKLIFT_WS`:

```zsh
source /opt/ros/jazzy/setup.zsh
source /path/to/fastlio_workspace/install/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
```

Проверка:

```zsh
ros2 pkg prefix fast_lio
ros2 pkg prefix livox_to_pointcloud2
```

## 7. Проверка launch-файлов

```zsh
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"

ros2 launch lidar_localization_mid360 mid360_driver.launch.py --show-args
ros2 launch lidar_localization_mid360 glim_mola_mapping.launch.py --show-args
ros2 launch lidar_localization_mid360 fastlio_mola_localization.launch.py --show-args
```

Папки с исходными launch-файлами:

```text
lidar_localization_mid360/launch/driver/
lidar_localization_mid360/launch/glim/
lidar_localization_mid360/launch/fastlio/
lidar_localization_mid360/launch/mola/
```

## 8. Карты и runtime-конфигурации

Карты хранятся в:

```text
$FORKLIFT_WS/maps
```

Конфигурации GLIM — в:

```text
$FORKLIFT_WS/runtime
```

Скрипты замыкания петель и построения metric map используют `FORKLIFT_WS`:

```zsh
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
ros2 run lidar_localization_mid360 close_loops.sh -- --map-name warehouse_01
ros2 run lidar_localization_mid360 build_metric_map.sh -- --map-name warehouse_01
```

## 9. Конвертация карты в Nav2

```zsh
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
python3 pcd_to_nav2_map.py \
  "$FORKLIFT_WS/maps/warehouse_01_lc.mm" \
  --output-dir .
```

Результат появится в `convert_pcd_to_nav2/`:

```text
nav2_map.pgm
nav2_map.yaml
```

## 10. Типовые проблемы

- `liblivox_lidar_sdk_shared.so not found` — не установлен Livox SDK2 или
  библиотека не находится в `/usr/local/lib`.
- `package not found` — не выполнен `source "$FORKLIFT_WS/install/setup.zsh"`
  или отсутствует underlay с FAST-LIO.
- `bind failed` — уже запущен второй Livox-драйвер или неверно настроены IP
  адреса в `MID360_config.json`.
- `glim_mola_localization` требует существующую `.mm` карту; сначала создайте
  её через mapping, loop closure и `build_metric_map.sh`.
