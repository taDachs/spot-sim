cmake_minimum_required(VERSION 3.16)
project(spot_sim_planning CXX)

# Local planning for the simulated Spot (map layers, MPPI). Pure C++ with no
# Gazebo or Spot API dependency, so it can be unit-tested in isolation; the
# LocalPlannerSystem plugin (spot_sim_gz) feeds it from the simulation.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/cost_to_go.cpp
  src/distance_field.cpp
  src/height_map.cpp
  src/mppi.cpp
  src/traversability.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_cost_to_go.cpp
    test/test_distance_field.cpp
    test/test_grid.cpp
    test/test_height_map.cpp
    test/test_mppi.cpp
    test/test_traversability.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
