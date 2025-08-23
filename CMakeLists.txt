cmake_minimum_required(VERSION 3.8)
project(vrobot_route)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===============================
# find dependencies
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  vrobot_local_planner
)

foreach(DEPENDENCY ${THIS_PACKAGE_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()


find_package(Eigen3 REQUIRED)

# ===============================
# Drogon
# ===============================
find_package(Drogon CONFIG REQUIRED)
# =======================================Interface definition=======================================
file(GLOB ${PROJECT_NAME}_interface_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
file(GLOB ${PROJECT_NAME}_action_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} action/*)
rosidl_generate_interfaces(
   ${PROJECT_NAME}
   ${${PROJECT_NAME}_interface_files}
   ${${PROJECT_NAME}_srv_files}
   ${${PROJECT_NAME}_action_files}
   DEPENDENCIES ${THIS_PACKAGE_DEPENDENCIES}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# ===================================End of Interface definition===================================
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

aux_source_directory(models MODELS_SRC)

# ===============================
# vrobot_route_lib
# ===============================
add_library(${PROJECT_NAME}_lib SHARED
  ${MODELS_SRC}
  src/db_client.cpp
  src/geometric_utils.cpp
  src/pathfinding.cpp
  src/graph_pose.cpp
  src/link_based_planner.cpp
)
target_include_directories(${PROJECT_NAME}_lib 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_lib 
  Drogon::Drogon
  ${EIGEN3_LIBRARIES}
  ${cpp_typesupport_target}
)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${THIS_PACKAGE_DEPENDENCIES}
)

# ===============================
# vrobot_route
# ===============================
add_executable(${PROJECT_NAME}_node
  src/vrobot_route.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}_lib
)

# ===============================
# vrobot_route_service
# ===============================
add_executable(${PROJECT_NAME}_service
  src/vrobot_route_service.cpp
)

target_include_directories(${PROJECT_NAME}_service PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${PROJECT_NAME}_service 
  ${PROJECT_NAME}_lib
)


# ===============================
# test
# ===============================
add_executable(test_action
  src/test/test_action.cpp
)
target_include_directories(test_action PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_action
  ${PROJECT_NAME}_lib
)

# ===============================
# install
# ===============================
install(TARGETS
  ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_service
  test_action
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})

ament_package()
