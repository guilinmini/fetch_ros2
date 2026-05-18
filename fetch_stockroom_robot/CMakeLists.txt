cmake_minimum_required(VERSION 3.8)
project(fetch_stockroom_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models slam worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
