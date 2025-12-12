cmake_minimum_required(VERSION 3.12)
project(dua_pcl)

set(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(dua_math REQUIRED)
find_package(PCL 1.14 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pose_kit REQUIRED)

# DUA PCL library
add_library(dua_pcl INTERFACE)
target_compile_features(dua_pcl INTERFACE c_std_99 cxx_std_17)
target_compile_definitions(dua_pcl INTERFACE "DUA_PCL_BUILDING_LIBRARY")
target_include_directories(dua_pcl INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(dua_pcl INTERFACE
  dua_math
  pcl_conversions
  pose_kit
)

# Library installation
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS dua_pcl
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  dua_math
  PCL
  pcl_conversions
  pose_kit
)

ament_package()
