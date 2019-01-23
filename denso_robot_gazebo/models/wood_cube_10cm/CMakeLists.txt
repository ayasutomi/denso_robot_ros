set(MODEL_NAME wood_cube_10cm)

set (files
  model.rsdf
  model.sdf
  model.config
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/${MODEL_NAME}/)

