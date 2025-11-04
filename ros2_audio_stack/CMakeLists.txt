cmake_minimum_required(VERSION 3.8)
project(ros2_audio_stack)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Instalacija Python modula i launch fajlova
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
