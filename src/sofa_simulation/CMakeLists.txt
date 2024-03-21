cmake_minimum_required(VERSION 3.0.2)
project(sofa_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sized-deallocation")


# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Declare a catkin package
catkin_package()

# Install Python scripts
catkin_install_python(PROGRAMS src/sofa_simulation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# export PATH=/home/chengjin/Software/cmake-3.18.0-Linux-x86_64/bin:$PATH
# export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
