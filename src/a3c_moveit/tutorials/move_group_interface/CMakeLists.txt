add_executable(a3c_move_group_interface_tutorial src/move_group_interface_tutorial.cpp)
target_link_libraries(a3c_move_group_interface_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS a3c_move_group_interface_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
