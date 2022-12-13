add_executable(a3c_pick_and_place_rviz_node src/pick_and_place_rviz.cpp)
target_link_libraries(a3c_pick_and_place_rviz_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS a3c_pick_and_place_rviz_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(a3c_pick_and_place_gaz_node src/pick_and_place_gaz.cpp)
target_link_libraries(a3c_pick_and_place_gaz_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS a3c_pick_and_place_gaz_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY urdf launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
