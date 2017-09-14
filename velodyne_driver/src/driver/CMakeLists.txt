# build the driver node
add_executable(velodyne_node velodyne_node.cc driver.cc)
add_dependencies(velodyne_node velodyne_driver_gencfg)
target_link_libraries(velodyne_node
  velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

add_executable(velodyne_simulator simulator_node.cc simulator.cc obstacle_simulator.cc)
target_link_libraries(velodyne_simulator
  velodyne_output
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(driver_nodelet nodelet.cc driver.cc)
add_dependencies(driver_nodelet velodyne_driver_gencfg)
target_link_libraries(driver_nodelet
  velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(simulator_nodelet simulator_nodelet.cc simulator.cc obstacle_simulator.cc)
target_link_libraries(simulator_nodelet
  velodyne_output
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
# install runtime files
install(TARGETS velodyne_node velodyne_simulator
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS driver_nodelet simulator_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
