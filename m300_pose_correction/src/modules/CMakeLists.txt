add_library(
  ${PROJECT_NAME}_modules
  SHARED
  ORBextractor.cc
  frame.cc
  viewer.cc
  feature_matcher.cc
  dataset.cc
)

target_link_libraries(
  ${PROJECT_NAME}_modules
  ${THIRD_PARTY_LIBS}
)
