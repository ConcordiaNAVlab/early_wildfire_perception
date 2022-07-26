add_executable(test_feature_matcher test_feature_matcher.cc)
target_link_libraries(test_feature_matcher ${PROJECT_NAME}_modules)

add_executable(test_pose_corrector test_pose_corrector.cc)
target_link_libraries(test_pose_corrector ${PROJECT_NAME}_app)

add_executable(test_dataset test_dataset.cc)
target_link_libraries(test_dataset ${PROJECT_NAME}_app)
