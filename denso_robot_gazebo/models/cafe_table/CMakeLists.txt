set(MODEL_NAME cafe_table)

set (files
  model.sdf
  model-1_4.sdf
  model.config
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/${MODEL_NAME}/)

