add_executable(a3c_kinematics_model src/kinematics_model_tutorial.cpp)
target_link_libraries(a3c_kinematics_model ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS a3c_kinematics_model DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
