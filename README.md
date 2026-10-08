# lidar_localization_mid360

Пошаговый запуск Livox Mid-360, GLIM и MOLA в ROS 2 Jazzy. Основное рабочее пространство проекта задаётся переменной `$FORKLIFT_WS`. Карты и runtime-конфигурации хранятся внутри этого workspace.

## Структура пакета

Переходить перед запуском нужно в корень workspace:

```zsh
export FORKLIFT_WS="${FORKLIFT_WS:-$HOME/forklift_ws}"
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
```

Основной пакет находится в `$FORKLIFT_WS/src/lidar_localization_mid360`:

```text
lidar_localization_mid360/
├── launch/
│   ├── driver/   # Livox MID-360
│   ├── fastlio/  # FAST-LIO + MOLA
│   ├── glim/     # GLIM + MOLA
│   └── mola/     # чистый MOLA
├── config/       # MID-360, FAST-LIO, MOLA и RViz
├── convert_pcd_to_nav2/  # MOLA/PCD -> карта Nav2
└── scripts/      # loop closure, metric map и диагностика
```

Все launch-файлы запускаются по имени из корня пакета, даже если физически
лежат в подпапке `launch`. Например, `glim_mola_localization.launch.py`
находится в `launch/glim/`.

Быстрый выбор алгоритма:

| Задача | Папка | Launch-файл |
|---|---|---|
| Только драйвер MID-360 | `launch/driver` | `mid360_driver.launch.py` |
| GLIM + MOLA | `launch/glim` | `glim_mola_mapping.launch.py`, `glim_mola_localization.launch.py` |
| FAST-LIO + MOLA | `launch/fastlio` | `fastlio_mapping.launch.py`, `fastlio_mola_localization.launch.py` |
| Чистый MOLA | `launch/mola` | `mapping.launch.py`, `localization.launch.py` |
| Конвертация карты в Nav2 | `convert_pcd_to_nav2` | `pcd_to_nav2_map.py` |

## GLIM odometry + MOLA mapping/localization

В этом варианте GLIM работает только как LiDAR-IMU одометрия и публикует
`/glim_ros/odom`. Глобальные loop closures выполняются инструментами MOLA,
поэтому GLIM не публикует конфликтующий `map -> odom` TF.

Сначала соберите карту:

```bash
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
ros2 launch lidar_localization_mid360 glim_mola_mapping.launch.py map_name:=test_glim_mola
```

После завершения записи замкните петли MOLA и постройте metric map:

```bash
ros2 run lidar_localization_mid360 close_loops.sh -- --map-name test_glim_mola
ros2 run lidar_localization_mid360 build_metric_map.sh -- --map-name test_glim_mola
```

Затем запустите локализацию из корня `$FORKLIFT_WS`:

```bash
ros2 launch lidar_localization_mid360 glim_mola_localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/test_glim_mola_lc.mm"
```

`initial_pose` задаётся в формате `[x, y, z, yaw_deg, pitch_deg, roll_deg]`.

## 0. Окружение

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
```

После сборки пакета выполните `source "$FORKLIFT_WS/install/setup.zsh"`.
Конфигурация использует IP лидара `192.168.0.187` и компьютера `192.168.0.200`.

## 1. Mid-360

В терминале 1 оставьте драйвер запущенным:

```zsh
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
ros2 launch lidar_localization_mid360 mid360_driver.launch.py use_rviz:=True
```

В терминале 2 проверьте данные:

```zsh
source /opt/ros/jazzy/setup.zsh
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
ros2 topic hz /livox/lidar
ros2 topic echo /livox/lidar --once --field header
ros2 topic echo /livox/lidar --once --field fields
ros2 topic echo /livox/imu --once --field header
```

Ожидаются `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Imu` и `frame_id: livox_frame`.

## 2. MOLA карта с одометрией FAST-LIO

Отдельный launch `fastlio_mapping.launch.py` запускает MID-360 в CustomMsg режиме,
FAST-LIO, адаптер позы LiDAR, преобразователь облака в PointCloud2 и MOLA.
MOLA строит карту по `/livox/points`, IMU `/livox/imu` и внешней одометрии
`/fastlio/odometry_livox`. FAST-LIO публикует `/Odometry`; адаптер переносит
позу с body на `livox_frame`, используя extrinsics из того же YAML FAST-LIO.

Задайте имя карты без расширения. После записи остановите launch через `Ctrl+C`,
чтобы MOLA сохранила SimpleMap и lazy-load облака:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch lidar_localization_mid360 fastlio_mapping.launch.py map_name:=warehouse_01
```

По умолчанию выход — `$FORKLIFT_WS/maps/warehouse_01` и соседний
каталог `warehouse_01_Images/`. В SimpleMap сохраняются также принятые
не-keyframe наблюдения, чтобы офлайн замыкание петель получило полную историю
траектории. Для loop closure пройдите маршрут с достаточным перемещением и
повторным посещением участков; короткий тест на неподвижном лидаре не создаёт
полезных кандидатов замыкания.

## 3. Замыкание петель

Выберите имя карты без расширения. В терминале 3:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch lidar_localization_mid360 mapping.launch.py map_name:=warehouse_01
```

Первые секунды держите лидар неподвижно, затем пройдите маршрут и вернитесь в знакомую область. Завершите MOLA через `Ctrl+C`.

MOLA обычно сохраняет SimpleMap как сжатый файл без расширения:

```text
$FORKLIFT_WS/maps/warehouse_01
$FORKLIFT_WS/maps/warehouse_01_Images/
```

Если выбранный профиль сохранит файл с расширением `.simplemap`, это тоже поддерживается скриптами.

Установите модуль один раз:

```zsh
sudo apt update
sudo apt install ros-jazzy-mola-sm-loop-closure
source /opt/ros/jazzy/setup.zsh
```

Обработайте карту. Скрипт автоматически проверяет оба входных имени: `warehouse_01` и `warehouse_01.simplemap`:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 run lidar_localization_mid360 close_loops.sh -- --map-name warehouse_01
```

Результат сохраняется как `$FORKLIFT_WS/maps/warehouse_01_lc.simplemap`. Для явной замены результата добавьте `--force`.

Пример для уже записанной карты `1312`:

```zsh
ros2 run lidar_localization_mid360 close_loops.sh -- --map-name 1312
```

## 4. Метрическая карта

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 run lidar_localization_mid360 build_metric_map.sh -- --map-name warehouse_01
```

Результат: `$FORKLIFT_WS/maps/warehouse_01_lc.mm`. Pipeline сохраняет облако в слое `localmap`, который ожидает профиль локализации.

Скрипт также принимает loop-closed карту без расширения, если она была сохранена в таком виде.

Просмотр:

```zsh
mm-viewer "$FORKLIFT_WS/maps/warehouse_01_lc.mm"
```

## 5. Локализация

```zsh
ros2 launch lidar_localization_mid360 localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/warehouse_01_lc.mm" \
  initial_pose:="[0, 0, 0, 0, 0, 0]"
```

`initial_pose` имеет формат `[x, y, z, yaw_deg, pitch_deg, roll_deg]` в координатах карты. Нулевые значения подходят только для старта в начале карты.

## 6. Внешняя одометрия

Этот вариант принимает `nav_msgs/msg/Odometry`. Поза сообщения должна относиться к `livox_frame`, поскольку текущий профиль локализует сам лидар:

```zsh
ros2 launch lidar_localization_mid360 localization_odom.launch.py \
  map_file:="$FORKLIFT_WS/maps/warehouse_01_lc.mm" \
  odom_topic:=/wheel/odometry \
  odom_sensor_label:=odom_wheels \
  initial_pose:="[0, 0, 0, 0, 0, 0]"
```

Не включайте одновременно `odom_topic_name` и `forward_ros_tf_odom_to_mola`. Для одометрии `base_link` нужен отдельный согласованный TF-профиль.

## 7. Fast-LIO + локализация MOLA

Этот единый launch запускает драйвер MID-360 в Livox `CustomMsg`-режиме,
Fast-LIO, конвертер облака для MOLA и локализацию MOLA по метрической карте:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
ros2 launch lidar_localization_mid360 fastlio_mola_localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/warehouse_01_lc.mm" \
  initial_pose:="[0, 0, 0, 0, 0, 0]"
```

`--base-paths src` обязателен в этом workspace: `whan_mola/` содержит
отдельное reference-дерево MOLA, которое не должно участвовать в сборке
верхнего Jazzy-профиля.

Fast-LIO использует `/livox/lidar` как `livox_ros_driver2/msg/CustomMsg`.
`livox_to_pointcloud2` создаёт из него `/livox/points` типа
`sensor_msgs/msg/PointCloud2` для MOLA. Адаптер преобразует Fast-LIO
`/Odometry` из frame `body` в `livox_frame` и передаёт результат MOLA как
`/fastlio/odometry_livox`. Он публикует `odom -> livox_frame`, а MOLA
публикует `map -> odom`: итоговая REP-105 ветка —
`map -> odom -> livox_frame -> base_footprint`.

Не запускайте одновременно `mid360_driver.launch.py` или
`mid360_fastlio.launch.py`: они создадут второй экземпляр драйвера и займут
те же UDP-порты MID-360. Параметры `mapping.extrinsic_T` и
`mapping.extrinsic_R` в `fast_lio/config/mid360.yaml` используются и
Fast-LIO, и адаптером; замените их после точной калибровки IMU↔LiDAR.

### Эксперимент: сглаживание `map→odom` через robot_localization

Отдельный запуск отключает MOLA TF и встроенный smoother. MOLA публикует
сырую локализацию, Fast-LIO оставляет `odom→livox_frame`, а `robot_localization`
с `smooth_lagged_data` публикует `map→odom`. Это позволяет обработать
запоздавшие pose-сообщения и проверить, уменьшаются ли скачки без изменения
оценщика MOLA:

```zsh
ros2 launch lidar_localization_mid360 fastlio_mola_ekf_localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/1122map4_lc.mm" \
  initial_pose:="[0, 0, 0, 0, 0, 0]"
```

Проверяйте TF и filtered pose отдельно:

```zsh
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /odometry/filtered
```

Для первого опыта оставьте робота неподвижным, затем медленно пройдите знакомый
участок. EKF отбрасывает выбросы по Mahalanobis-порогу и хранит 1,5 секунды
истории; это экспериментальная настройка, не готовый профиль эксплуатации.

### Инертная локализация со сглаживанием `map→odom`

Этот профиль оставляет FAST-LIO источником локальной одометрии, а MOLA —
источником позы относительно карты. Встроенный MOLA state estimator выключен
(`use_state_estimator:=False`), и MOLA не публикует TF. Единственным
издателем `map→odom` становится отдельный `map_odom_smoother.py`; TF
`odom→livox_frame` продолжает публиковать адаптер FAST-LIO. Поэтому сглаживается
глобальная коррекция относительно карты, а локальная одометрия FAST-LIO не
фильтруется.

Узел берёт позу MOLA из `/lidar_odometry/pose` и синхронный по timestamp TF
`odom→livox_frame`, вычисляет `map→odom`, затем усредняет перенос и
ориентацию по скользящему окну. По умолчанию окно — 3 секунды, TF публикуется
с частотой 20 Гц. Кандидаты, отстоящие от принятой оценки более чем на 0,75 м
или 20 градусов, временно считаются выбросами; новая позиция принимается после
четырёх согласованных измерений. При отсутствии свежего входа дольше 0,75 с
публикация прекращается. Это экспериментальный фильтр с задержкой: увеличение
окна сильнее замедляет реакцию на настоящие изменения локализации.

Запуск:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
ros2 launch lidar_localization_mid360 fastlio_mola_inertial_localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/1122map4_lc.mm" \
  initial_pose:="[0, 0, 0, 0, 0, 0]" \
  window_seconds:=3.0
```

Для другой карты задайте её абсолютный путь в `map_file`. `window_seconds`
можно менять при запуске, например `window_seconds:=1.0` даст более быструю
реакцию, а `window_seconds:=5.0` — более инертную коррекцию. Остановите другой
запущенный localization launch перед этим запуском: MID-360 и TF должны иметь
только по одному владельцу.

## 8. Конвертация карты MOLA/PCD в Nav2

Конвертер находится отдельно от launch-файлов в папке:

`$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2`

Переходите именно в эту папку перед запуском скриптов конвертации.

### MOLA `.mm` → карта Nav2

```zsh
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
python3 pcd_to_nav2_map.py \
  "$FORKLIFT_WS/maps/warehouse_01_lc.mm" \
  --output-dir .
```

Результат появится в этой же папке:

```text
nav2_map.pgm
nav2_map.yaml
```

### PCD → текстовые точки → карта Nav2

```zsh
cd "$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2"
python3 pcd2txt_xyzi.py /полный/путь/input.pcd points.txt
python3 pcd_to_nav2_map.py points.txt --output-dir .
```

Запуск полученной карты через Nav2 Map Server:

```zsh
source /opt/ros/jazzy/setup.zsh
source "$FORKLIFT_WS/install/setup.zsh"
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2/nav2_map.yaml"
```

Для просмотра в RViz2 используйте конфигурацию:

`$FORKLIFT_WS/src/lidar_localization_mid360/convert_pcd_to_nav2/nav2_map.rviz`

## Диагностика

### Скачки `map -> odom` при FAST-LIO + MOLA

Остановите предыдущий объединённый launch через `Ctrl+C`. Диагностический
режим включает одновременно `/lidar_odometry/pose` (сырая локализация по
карте) и `/state_estimation/pose` (сглаженная оценка, если включён smoother).
Без сглаживателя TF публикует простая MOLA локализация; с `use_state_estimator:=True`
источником становится `state_estimation`. Калибровка и обработка сенсоров не меняются. Второй
экземпляр MOLA не запускается, файлы `/opt/ros/jazzy` не изменяются.
Wrapper передаёт пустой фильтр как YAML-литерал `""`, а не как пустое
значение переменной окружения: после подстановки MOLA должна получить
строку, не YAML `null`. Это проверяется тестом установленного MOLA/MRPT
парсера вместе с сохранением подписок и фильтра TF.

Терминал 1:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
ros2 launch lidar_localization_mid360 fastlio_mola_localization.launch.py \
  map_file:="$FORKLIFT_WS/maps/1312_lc.mm" \
  initial_pose:="[0, 0, 0, 0, 0, 0]" diagnostics:=True \
  use_state_estimator:=False
```

Сохраните карту и начальную позу предыдущего опыта. Нулевая начальная поза
правильна только при старте в начале карты. Дождитесь инициализации,
держите лидар неподвижно и в терминале 2 запишите baseline:

```zsh
cd "$FORKLIFT_WS"
source /opt/ros/jazzy/setup.zsh
source install/local_setup.zsh
ros2 run lidar_localization_mid360 fastlio_mola_tf_audit.py --duration 30 --label stationary
```

Затем повторите запись с перемещениями и поворотами:

```zsh
ros2 run lidar_localization_mid360 fastlio_mola_tf_audit.py --duration 40 --label motion
```

Скрипт ничего не публикует. JSON с уникальным именем сохраняется в
`runs/tf-diagnostics/`; `--output /полный/путь/имя.json` задаёт свой путь
без перезаписи существующего файла. `Ctrl+C` сохраняет неполный замер.
Пропавшие потоки выводятся как предупреждение, а не как «стабильные».

В записи есть позы, полные ковариации, TF, качество, статус MOLA, сведения
об издателях и метки облака/IMU. `analysis.raw_vs_smoothed` сравнивает позы
на одинаковых метках времени: положение интерполируется линейно,
ориентация — SLERP. Экстраполяция и промежутки больше `--max-gap` (0,3 с)
исключаются. Откаты меток анализируются отдельно в порядке получения.
`analysis.reconstructed_map_odom` вычисляет для каждого выхода MOLA
`T_map_lidar(t) * inverse(T_odom_lidar(t))`, исключая движение самого лидара.
Метки облака соответствуют началу скана, FAST-LIO — его концу: они
сохраняются без искусственного сдвига. TF `map -> odom` переиздаётся с
меткой в будущем, поэтому его отрицательная задержка не означает
отрицательную задержку локализации.

Если сырая поза стабильна, а сглаженная скачет, исследуем объединение
данных и временную обработку сглаживателя. Если скачет сырая поза —
локализацию по карте и deskew. Если обе позы стабильны, а TF скачет —
композицию TF в bridge. Высокое качество ICP само по себе не доказывает
правильность позы. После диагностики `diagnostics:=False` возвращает
обычный режим; никакие исправления extrinsics/ковариаций здесь не внесены.

### Общие проверки

```zsh
ros2 topic info /livox/lidar
ros2 topic info /livox/imu
ros2 run tf2_tools view_frames
```

`True` и `False` в командах launch должны иметь заглавную букву. Ошибка `bind failed` обычно означает неверный IP или второй запущенный драйвер. Если в `fields` нет `timestamp`, MOLA сможет работать, но deskew по отдельным точкам будет недоступен.
