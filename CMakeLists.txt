cmake_minimum_required(VERSION 3.8)
project(felony_stop_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  models
  launch
  maps
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
