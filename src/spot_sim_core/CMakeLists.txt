cmake_minimum_required(VERSION 3.16)
project(spot_sim_core CXX)

# The simulated robot's "firmware": Spot API semantics with no Gazebo
# dependency, so it can be unit-tested in isolation.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(spot_sim_api_protos REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/api_server.cpp
  src/commands.cpp
  src/directory.cpp
  src/estop.cpp
  src/gait.cpp
  src/lease.cpp
  src/leg_kinematics.cpp
  src/power.cpp
  src/robot.cpp
  src/robot_state.cpp
  src/time_sync.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC spot_sim_api_protos::spot_sim_api_protos)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # test_api_server needs SPOT_SIM_TEST_CERTS_DIR (spot_wrapper testing
  # credentials) in the environment; it skips otherwise.
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_api_server.cpp
    test/test_commands.cpp
    test/test_directory.cpp
    test/test_estop.cpp
    test/test_gait.cpp
    test/test_lease.cpp
    test/test_leg_kinematics.cpp
    test/test_power.cpp
    test/test_robot_state.cpp
    test/test_time_sync.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(spot_sim_api_protos)
ament_package()
