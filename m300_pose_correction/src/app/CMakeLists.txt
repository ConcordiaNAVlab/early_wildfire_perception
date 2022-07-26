add_library(${PROJECT_NAME}_app SHARED pose_corrector.cc)
target_link_libraries(${PROJECT_NAME}_app ${THIRD_PARTY_LIBS}
                      ${PROJECT_NAME}_modules)
