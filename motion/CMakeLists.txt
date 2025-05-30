cmake_minimum_required(VERSION 3.8)
project(motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# Encontrar dependencias
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# Inclusiones
include_directories(
    include
)
# Definir los ejecutables
add_executable(controller src/controller.cpp src/main_controller.cpp)
add_executable(actuator src/actuator.cpp src/main_actuator.cpp)
add_executable(motion_model src/motion_model.cpp)
add_executable(motion_model_noise src/motion_model_noise.cpp)
# Declarar e instalar ejecutables
install(TARGETS
        controller
        actuator
        motion_model
        motion_model_noise
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_target_dependencies(controller rclcpp std_msgs)
ament_target_dependencies(actuator rclcpp std_msgs geometry_msgs)
ament_target_dependencies(motion_model rclcpp std_msgs geometry_msgs)
ament_target_dependencies(motion_model_noise rclcpp std_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
