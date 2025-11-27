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
find_package(PCL 1.14 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pose_kit REQUIRED)

# DUA PCL library configuration
add_library(dua_pcl INTERFACE)
target_compile_features(dua_pcl INTERFACE c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_definitions(dua_pcl INTERFACE "DUA_PCL_BUILDING_LIBRARY")
target_include_directories(dua_pcl INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(dua_pcl INTERFACE
  ${PCL_LIBRARIES})
ament_target_dependencies(dua_pcl INTERFACE
  pose_kit)

# Library installation
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS dua_pcl
  EXPORT export_${PROJECT_NAME}
  INCLUDES DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export all dependencies and library targets for this package
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  PCL
  pcl_conversions
  pose_kit)

ament_package()
