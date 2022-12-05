add_executable(a3c_pick_and_place_node src/pick_and_place.cpp)
target_link_libraries(a3c_pick_and_place_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS a3c_pick_and_place_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
add_executable(temp_node src/temp.cpp)
target_link_libraries(temp_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS temp_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})