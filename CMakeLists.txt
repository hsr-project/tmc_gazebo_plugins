cmake_minimum_required(VERSION 2.8.3)
project(tmc_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  control_toolbox
  roscpp
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp gazebo_ros control_toolbox
  INCLUDE_DIRS
  LIBRARIES tmc_gazebo_mimic_plugin
  CFG_EXTRAS
)

if(CMAKE_VERSION VERSION_LESS "3.1")
    add_compile_options(-std=c++11)
else()
    set(CMAKE_CXX_STANDARD 11)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(tmc_gazebo_mimic_plugin src/mimic_plugin.cpp)
target_link_libraries(tmc_gazebo_mimic_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_mimic test/test_mimic.test test/test_mimic.cpp)
    target_link_libraries(test_mimic ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${GTEST_LIBRARIES})
endif()

install(
  TARGETS tmc_gazebo_mimic_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
