cmake_minimum_required(VERSION 3.8)
project(vio)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpylint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  launch
  worlds
  rviz
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts with proper permissions
install(PROGRAMS
  scripts/circle_flight_controller.py
  scripts/imu_monitor.py
  scripts/pattern_flight_controller.py
  scripts/pattern_commander.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create symlinks for executables without .py extension
install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    circle_flight_controller.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/circle_flight_controller
  )
")

install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    imu_monitor.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/imu_monitor
  )
")

install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    pattern_flight_controller.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/pattern_flight_controller
  )
")

install(CODE "
  execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    pattern_commander.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/pattern_commander
  )
")

ament_package()
