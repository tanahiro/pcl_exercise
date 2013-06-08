set(TARGET polygon_mesh)

project(${TARGET})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PCL 1.7 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${Boost_INCLUDE_DIRS})

add_executable(${TARGET} ${TARGET}.cpp)
target_link_libraries(${TARGET} ${PCL_LIBRARIES})
target_link_libraries(${TARGET} ${Boost_LIBRARIES})
