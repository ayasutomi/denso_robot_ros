# ladders
add_subdirectory(table)
add_subdirectory(cafe_table)
add_subdirectory(beer)
add_subdirectory(coke_can)
add_subdirectory(wood_cube_10cm)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gazebo_models/environments/)

