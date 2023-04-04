cmake_minimum_required(VERSION 2.8.3)
project(ds4_driver)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
)

catkin_install_python(PROGRAMS
        node/ds4_driver
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)
