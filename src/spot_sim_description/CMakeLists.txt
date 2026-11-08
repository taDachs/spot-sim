cmake_minimum_required(VERSION 3.16)
project(spot_sim_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
