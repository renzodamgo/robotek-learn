cmake_minimum_required(VERSION 3.8)
project(robotek_gazebo)

find_package(ament_cmake REQUIRED)

# Install worlds and launch files into share/robotek_gazebo/
install(
  DIRECTORY worlds launch models config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
