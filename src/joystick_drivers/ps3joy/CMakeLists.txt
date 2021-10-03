cmake_minimum_required(VERSION 3.5)
project(ps3joy)

find_package(PkgConfig REQUIRED)
pkg_search_module(LIBUSB REQUIRED libusb)

if(LIBUSB_FOUND)
  include_directories(SYSTEM ${LIBUSB_INCLUDE_DIRS})
  link_directories(${LIBUSB_LIBRARY_DIRS})
else()
  message( FATAL_ERROR "Failed to find libusb" )
endif()

# Load catkin and all dependencies required for this package
set(CATKIN_DEPS diagnostic_msgs sensor_msgs rospy rosgraph roslint)
find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPS})
catkin_package(CATKIN_DEPENDS diagnostic_msgs sensor_msgs)

roslint_python(
  scripts/ps3joy.py
  scripts/ps3joy_node.py
  scripts/ps3joysim.py
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(sixpair src/sixpair.c)
target_link_libraries(sixpair -lusb ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  roslint_add_test()
endif()

# Install targets
install(TARGETS sixpair
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES diagnostics.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/ps3joy.py
  scripts/ps3joy_node.py
  scripts/ps3joysim.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
