set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required(VERSION 3.7)

project(tmp_gtsam_link_err)


find_package(PCL REQUIRED)
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
find_package(Boost 1.61 COMPONENTS filesystem program_options iostreams timer REQUIRED)

include_directories(
 ${PCL_INCLUDE_DIRS}
 ${GTSAM_INCLUDE_DIR}
 ${Boost_INCLUDE_DIR}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(sources
    src/linkErr.cpp)

add_executable(linkErr ${sources})

#target_link_libraries(linkErr ${PCL_LIBRARIES}) # Code works fine
target_link_libraries(linkErr ${PCL_LIBRARIES} gtsam) # Code has issues
