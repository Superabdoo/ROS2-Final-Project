# ============================================================================
# CMakeLists.txt for edges_bringup package
# This file tells CMake (the build system) how to build and install the package.
# Students: Read carefully — almost every ROS 2 package has a CMakeLists like this.
# ============================================================================

# ------------------- MINIMUM CMAKE VERSION -------------------
cmake_minimum_required(VERSION 3.8)  
# The project requires CMake version 3.8 or higher to work.

# ------------------- PROJECT NAME -------------------
project(edges_bringup)  
# Defines the name of the project (and package).
# This should match the package.xml <name>.

# ------------------- COMPILER FLAGS -------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# If the compiler is GCC or Clang:
#   -Wall    → Enable most warning messages
#   -Wextra  → Enable extra warning messages
#   -Wpedantic → Enforce strict standard compliance
# This ensures cleaner, safer code.

# ------------------- DEPENDENCIES -------------------
find_package(ament_cmake REQUIRED)
# Finds and loads the "ament_cmake" build system (mandatory for ROS 2 packages).
# REQUIRED means it must exist, otherwise the build will fail.

# Uncomment and use this if your package depends on other ROS 2 libraries:
# find_package(<dependency> REQUIRED)
# Example: find_package(rclcpp REQUIRED)

# ------------------- INSTALL SECTION -------------------
install (
  DIRECTORY launch              # Install "launch" and "config" folders
  DESTINATION share/${PROJECT_NAME}/   # Copy them into install/share/edges_bringup/
)
# This ensures that your launch and config files are available after installation.

# ------------------- AMENT PACKAGE MACRO -------------------
ament_package()
# Must be called at the end of every CMakeLists.txt in a ROS 2 package.
# It finalizes the build setup and makes the package discoverable by colcon.
