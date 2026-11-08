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

# Header-only contract between SpotApiSystem and locomotion base plugins.
add_library(spot_sim_gz_components INTERFACE)
target_include_directories(spot_sim_gz_components INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(spot_sim_gz_components INTERFACE gz-sim::gz-sim spot_sim_core::spot_sim_core)

foreach(plugin SpotApiSystem KinematicBaseSystem)
  string(REGEX REPLACE "([a-z])([A-Z])" "\\1_\\2" source "${plugin}")
  string(TOLOWER "${source}" source)
  add_library(${plugin} SHARED src/${source}.cpp)
  target_link_libraries(${plugin} spot_sim_gz_components gz-plugin::register)
  target_compile_options(${plugin} PRIVATE -Wall -Wextra)
  install(TARGETS ${plugin} LIBRARY DESTINATION lib)
endforeach()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS spot_sim_gz_components EXPORT export_${PROJECT_NAME})

# Make the plugins discoverable by `gz sim` once the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(gz_sim_vendor gz-sim spot_sim_core)
ament_package()
