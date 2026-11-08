cmake_minimum_required(VERSION 3.16)
project(spot_sim_api_protos CXX)

# Compiles the Spot SDK .proto files (bosdyn/api/**) into one C++ library with
# protobuf messages and gRPC service stubs/base classes. Uses the system
# protobuf and gRPC so it can share a process with Gazebo's gz-msgs.

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPCPP REQUIRED IMPORTED_TARGET grpc++)
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin REQUIRED)

set(SPOT_SDK_PROTO_DIR "" CACHE PATH "Directory containing bosdyn/api/*.proto (spot-sdk/protos)")
if(NOT SPOT_SDK_PROTO_DIR AND DEFINED ENV{SPOT_SDK_PROTO_DIR})
  set(SPOT_SDK_PROTO_DIR "$ENV{SPOT_SDK_PROTO_DIR}")
endif()
if(NOT EXISTS "${SPOT_SDK_PROTO_DIR}/bosdyn/api/header.proto")
  message(FATAL_ERROR "SPOT_SDK_PROTO_DIR='${SPOT_SDK_PROTO_DIR}' does not contain bosdyn/api/header.proto")
endif()

file(GLOB_RECURSE PROTO_FILES RELATIVE "${SPOT_SDK_PROTO_DIR}" "${SPOT_SDK_PROTO_DIR}/bosdyn/api/*.proto")
list(SORT PROTO_FILES)

set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/gen")
file(MAKE_DIRECTORY "${GEN_DIR}")

set(GENERATED_SOURCES)
set(GENERATED_HEADERS)
foreach(proto ${PROTO_FILES})
  string(REGEX REPLACE "\\.proto$" "" stem "${proto}")
  set(outputs
    "${GEN_DIR}/${stem}.pb.cc" "${GEN_DIR}/${stem}.pb.h"
    "${GEN_DIR}/${stem}.grpc.pb.cc" "${GEN_DIR}/${stem}.grpc.pb.h")
  add_custom_command(
    OUTPUT ${outputs}
    COMMAND protobuf::protoc
      --proto_path=${SPOT_SDK_PROTO_DIR}
      --cpp_out=${GEN_DIR}
      --grpc_out=${GEN_DIR}
      --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
      ${proto}
    DEPENDS "${SPOT_SDK_PROTO_DIR}/${proto}"
    COMMENT "protoc ${proto}"
    VERBATIM)
  list(APPEND GENERATED_SOURCES "${GEN_DIR}/${stem}.pb.cc" "${GEN_DIR}/${stem}.grpc.pb.cc")
  list(APPEND GENERATED_HEADERS "${GEN_DIR}/${stem}.pb.h" "${GEN_DIR}/${stem}.grpc.pb.h")
endforeach()

add_library(${PROJECT_NAME} SHARED ${GENERATED_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${GEN_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC protobuf::libprotobuf PkgConfig::GRPCPP)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
# Generated code is noisy; keep the build log readable.
target_compile_options(${PROJECT_NAME} PRIVATE -w)

install(DIRECTORY "${GEN_DIR}/" DESTINATION include/${PROJECT_NAME} FILES_MATCHING PATTERN "*.h")
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Protobuf)
ament_package(CONFIG_EXTRAS "cmake/${PROJECT_NAME}-extras.cmake")
