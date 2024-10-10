cmake_minimum_required(VERSION 3.0.2)
project(qt_interface)

# Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

# Since we're using Qt6, make sure it's installed
find_package(Qt6 REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include/qt_interface
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  include/qt_interface/mainwindow.h
  resources/mainwindow.ui
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
)

# Qt6 specific executable setup without version checks
qt_add_executable(${PROJECT_NAME}_node
  MANUAL_FINALIZATION
  ${PROJECT_SOURCES}
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/resources"
)

# Link necessary libraries
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE Qt6::Widgets
  PRIVATE ${catkin_LIBRARIES}
)

# Finalize Qt6 executables
qt_finalize_executable(${PROJECT_NAME}_node)

# Set the CMake policy to avoid Qt policy warnings
cmake_policy(SET CMP0084 NEW)
