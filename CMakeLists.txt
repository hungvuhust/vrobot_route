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

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

aux_source_directory(models MODELS_SRC)

# ===============================
# vrobot_route_lib
# ===============================
add_library(vrobot_route_lib SHARED
  ${MODELS_SRC}
  src/db_client.cpp
  src/geometric_utils.cpp
  src/pathfinding.cpp
  src/graph_pose.cpp
  src/link_based_planner.cpp
)
target_include_directories(vrobot_route_lib 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/models
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(vrobot_route_lib 
  Drogon::Drogon
  ${EIGEN3_LIBRARIES}
)
ament_target_dependencies(vrobot_route_lib
  ${THIS_PACKAGE_DEPENDENCIES}
)

# ===============================
# vrobot_route
# ===============================
add_executable(vrobot_route
  src/vrobot_route.cpp
)
target_include_directories(vrobot_route PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(vrobot_route 
  vrobot_route_lib
)

# ===============================
# install
# ===============================
install(TARGETS
  vrobot_route_lib
  EXPORT vrobot_route_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  vrobot_route
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(vrobot_route_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})

ament_package()
