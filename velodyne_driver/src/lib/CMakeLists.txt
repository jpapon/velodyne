add_library(velodyne_input input.cc)
target_link_libraries(velodyne_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

add_library(velodyne_output output.cc)
target_link_libraries(velodyne_output
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(velodyne_input ${catkin_EXPORTED_TARGETS})
  add_dependencies(velodyne_output ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS velodyne_input velodyne_output
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
