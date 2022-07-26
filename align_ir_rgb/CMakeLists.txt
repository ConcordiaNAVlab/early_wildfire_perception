cmake_minimum_required(VERSION 2.8)
project(align_ir_rgb)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -std=c++11 -O0 -Wall -ggdb -fopenmp")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -std=c++11 -O3 -fopenmp")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) set(LIBRARY_OUTPUT_PATH
# ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

# Eigen
include_directories("/usr/include/eigen3")

# Sophus
find_package(fmt REQUIRED)
find_package(Sophus REQUIRED)
include_directories("/usr/local/include/sophus")

# G2O
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

# boost
find_package(Boost REQUIRED COMPONENTS system filesystem)
include_directories(${Boost_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS
    g2o_core
    g2o_stuff
    g2o_types_sba
    g2o_solver_csparse
    g2o_csparse_extension
    ${CSPARSE_LIBRARY}
    ${Sophus_LIBRARIES}
    ${Boost_LIBRARIES})

set(${PROJECT_NAME}_tools)
add_library(${PROJECT_NAME}_tools src/tools.cc)
target_link_libraries(${PROJECT_NAME}_tools ${THIRD_PARTY_LIBS})

# for test
add_executable(test_files_reading src/test_files_reading.cc)
target_link_libraries(test_files_reading ${THIRD_PARTY_LIBS})
target_link_libraries(test_files_reading ${PROJECT_NAME}_tools)

add_executable(test_each_measurement src/test_each_measurement.cc)
target_link_libraries(test_each_measurement ${THIRD_PARTY_LIBS})
target_link_libraries(test_each_measurement ${PROJECT_NAME}_tools)


add_executable(align_ir_rgb_g2o src/align_ir_rgb_g2o.cc)
target_link_libraries(align_ir_rgb_g2o ${THIRD_PARTY_LIBS})
target_link_libraries(align_ir_rgb_g2o ${PROJECT_NAME}_tools)
