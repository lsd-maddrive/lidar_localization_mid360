# Конвертер MOLA/PCD-карт для Nav2 и RViz2

Эта инструкция описывает полный путь от полученного файла `.mm` до его отображения в RViz2.

Проект извлекает точки из сохранённой MOLA/MRPT-карты, строит по ним предварительную 2D-карту Nav2 и публикует её через `nav2_map_server`. SLAM и навигация здесь не запускаются.

## 1. Подготовить файл карты

Во всех командах ниже используется переменная `$FORKLIFT_WS`:

```bash
export FORKLIFT_WS="${FORKLIFT_WS:-$HOME/forklift_ws}"
```

Каталог проекта:

```bash
$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2
```

Перейдите в него, скопируйте туда полученный файл или используйте его полный путь:

```bash
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
ls -lh my_map.mm
file my_map.mm
```

Обычный вход для текущего скрипта — сжатый Zstandard-файл MOLA `.mm`. Команда `file` обычно показывает `Zstandard compressed data`. Распакованный файл `.raw.mm` текущий автоматический режим не обрабатывает.

## 2. Подготовить зависимости

В каждом терминале сначала выполняйте:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.zshrc
```

Проверьте Python-пакеты:

```bash
python3 -c "import numpy, PIL; print('Python dependencies: OK')"
```

Если нужно, установите их:

```bash
python3 -m pip install --user numpy pillow
```

Проверьте остальные зависимости:

```bash
command -v g++
command -v zstd
ls /opt/ros/jazzy/include/mrpt
```

`g++` автоматически соберёт экстрактор `.mm`, `zstd` распакует карту, а MRPT/MOLA должны быть установлены в ROS 2 Jazzy.

## 3. Построить карту Nav2 из `.mm`

Для MOLA `.mm` нужны установленные ROS 2 Jazzy MRPT/MOLA и `zstd`. При первом запуске
скрипт автоматически соберёт небольшой extractor `mm_extract.cpp`.

```bash
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
python3 pcd_to_nav2_map.py my_map.mm --output-dir .
```

Или укажите полный путь к входному файлу:

```bash
python3 pcd_to_nav2_map.py \
  /полный/путь/к/my_map.mm \
  --output-dir "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
```

При первом запуске скрипт автоматически соберёт `.mm_extract`. В результате должны появиться `nav2_map.pgm` и `nav2_map.yaml`.

Проверьте результат:

```bash
ls -lh nav2_map.pgm nav2_map.yaml
file nav2_map.pgm
cat nav2_map.yaml
```

Текущие параметры: высоты `0.05–1.5 м`, разрешение `0.05 м/пиксель`, минимум 1 точка на занятую клетку. Дополнительно одиночные клетки удаляются, если в радиусе `0.15 м` вокруг них нет соседней занятой клетки. Эти параметры задаются в начале `pcd_to_nav2_map.py`.

Для более сильной очистки увеличьте `ISOLATED_POINT_RADIUS_M`, например до `0.25`. Если нужно требовать несколько соседей, увеличьте `MIN_NEIGHBOR_CELLS`. Слишком большие значения могут удалить тонкие или разреженные препятствия.

## 4. Запустить `map_server`

```bash
source /opt/ros/jazzy/setup.bash
source ~/.zshrc

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2/nav2_map.yaml"
```

В первом терминале оставьте `map_server` запущенным. Во втором терминале активируйте его:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.zshrc
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 lifecycle get /map_server
```

Ожидаемое состояние — `active [3]`. Проверить публикацию можно командой `ros2 topic echo /map --once`.

## 5. Открыть карту в RViz2

В третьем терминале:

```bash
source /opt/ros/jazzy/setup.bash
source ~/.zshrc
rviz2 -d "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2/nav2_map.rviz"
```

В RViz2:

1. Откройте `Panels → Displays`.
2. В `Global Options` установите `Fixed Frame: map`.
3. Добавьте `Add → By topic → /map → Map`.
4. Для `Map` установите `Durability Policy: Transient Local` и `Reliability Policy: Reliable`.

Если TF-дерево отсутствует и нужна только проверка отображения:

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 0 0 0 map base_link
```

Для настоящей навигации Nav2 нужны реальные связи `map → odom → base_link` от SLAM/AMCL и одометрии.

## 6. Если карта не появилась

Проверьте:

```bash
ros2 node list
ros2 topic list | grep map
ros2 topic echo /map --once
ros2 lifecycle get /map_server
```

Частые причины: `map_server` не активирован, неверный путь в `yaml_filename`, в RViz2 выбран другой `Fixed Frame`, отсутствует TF или `.mm` несовместим с установленной версией MRPT/MOLA.

Если ошибка возникает уже при конвертации, повторите отдельно:

```bash
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
python3 pcd_to_nav2_map.py /полный/путь/к/my_map.mm --output-dir .
```

## 7. Ограничение

Скрипт отмечает клетки с точками как занятые, а остальные как свободные. Это предварительная 2D-проекция для проверки карты. Для надёжной навигации следует учитывать лучи лидара и неизвестную область.

Для текущего `test_glim_mola_02_lc.mm` результат после запуска:

```text
map=723x609, resolution=0.05 m/px
origin=[-12.425600, -10.801200, 0.0]
```

## 8. Альтернативный вход через PCD

Для бинарного PCD с полями `x y z intensity`:

```bash
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
python3 pcd2txt_xyzi.py /полный/путь/к/input.pcd points.txt
python3 pcd_to_nav2_map.py points.txt --output-dir .
```

После этого используются те же шаги запуска `map_server`, lifecycle-команд и RViz2.
