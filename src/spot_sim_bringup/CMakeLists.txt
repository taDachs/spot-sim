cmake_minimum_required(VERSION 3.16)
project(spot_sim_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
