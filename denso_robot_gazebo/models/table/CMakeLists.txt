set(MODEL_NAME table)

set (files
  model.sdf
  model-1_2.sdf
  model-1_3.sdf
  model-1_4.sdf
  model.config
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/${MODEL_NAME}/)

