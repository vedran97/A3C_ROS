cmake_minimum_required(VERSION 2.8.3)

project(a3c_description)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch worlds rviz config xacro urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)