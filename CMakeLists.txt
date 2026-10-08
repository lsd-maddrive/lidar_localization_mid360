cmake_minimum_required(VERSION 3.8)
project(lidar_localization_mid360)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config convert_pcd_to_nav2
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/close_loops.sh
  scripts/build_metric_map.sh
  scripts/fastlio_odometry_to_livox.py
  scripts/fastlio_mola_tf_audit.py
  scripts/map_odom_smoother.py
  scripts/lidar_map_path_publisher.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES README.md INSTALL.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(mola_yaml REQUIRED)
  add_executable(mola_yaml_diagnostic_probe test/mola_yaml_diagnostic_probe.cpp)
  target_link_libraries(mola_yaml_diagnostic_probe PRIVATE mola::mola_yaml)
  ament_add_pytest_test(fastlio_mola_diagnostics test/test_fastlio_mola_diagnostics.py
    ENV "LETI_MOLA_YAML_PROBE=$<TARGET_FILE:mola_yaml_diagnostic_probe>")
endif()

ament_package()
