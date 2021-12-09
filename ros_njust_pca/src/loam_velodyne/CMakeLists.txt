add_subdirectory(lib)

add_executable(multiScanRegistration multi_scan_registration_node.cpp)
target_link_libraries(multiScanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserOdometry laser_odometry_node.cpp)
target_link_libraries(laserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(laserMapping laser_mapping_node.cpp)
target_link_libraries(laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )

add_executable(transformMaintenance transform_maintenance_node.cpp)
target_link_libraries(transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES} loam )