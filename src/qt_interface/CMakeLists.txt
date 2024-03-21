cmake_minimum_required(VERSION 3.0.2)
project(qt_interface)

# list(APPEND CMAKE_PREFIX_PATH "/home/chengjin/Qt/6.6.2/gcc_64/lib/cmake/Qt6")

# set(CMAKE_PREFIX_PATH "/home/chengjin/Qt/6.6.2")

# Find catkin macros and libraries, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  # Add other necessary ROS components here
)


# Since we're using Qt 6, make sure it's installed
find_package(QT NAMES Qt6 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


# Your package locations should be listed before other locations
include_directories(
  include/qt_interface
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_SOURCES
  src/main.cpp
  src/mainwindow.cpp
  include/qt_interface/mainwindow.h
  resources/mainwindow.ui  # Keep this path as is
)

catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES qt_interface
  CATKIN_DEPENDS roscpp
  # DEPENDS system_lib
)

# # Qt 6 specific executable setup
# qt_add_executable(${PROJECT_NAME}_node
#   MANUAL_FINALIZATION
#   ${PROJECT_SOURCES}
# )

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(${PROJECT_NAME}_node
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    if(ANDROID)
        add_library(${PROJECT_NAME}_node SHARED
            ${PROJECT_SOURCES}
        )
    else()
        add_executable(${PROJECT_NAME}_node
            ${PROJECT_SOURCES}
        )
    endif()
endif()

set_target_properties(${PROJECT_NAME}_node PROPERTIES
  AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/resources"
)

# Use the keyword signature for target_link_libraries
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE Qt6::Widgets
  PRIVATE ${catkin_LIBRARIES}
)

# Necessary for Qt 6 executables
if(QT_VERSION_MAJOR EQUAL 6)
  qt_finalize_executable(${PROJECT_NAME}_node)
endif()

# Install rules, if needed
# install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
