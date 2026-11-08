cmake_minimum_required(VERSION 3.16)
project(spot_sim_gz CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(spot_sim_core REQUIRED)

add_library(SpotApiSystem SHARED src/spot_api_system.cpp)
target_link_libraries(SpotApiSystem
  gz-sim::gz-sim
  gz-plugin::register
  spot_sim_core::spot_sim_core)
target_compile_options(SpotApiSystem PRIVATE -Wall -Wextra)

install(TARGETS SpotApiSystem LIBRARY DESTINATION lib)

# Make the plugins discoverable by `gz sim` once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()
