cmake_minimum_required(VERSION 2.8.12)

project(osgMinitaur)

find_package(DART 6.1.1 REQUIRED COMPONENTS utils-urdf gui-osg)

# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
# find_package(libcmaes REQUIRED)

add_compile_options(-std=c++11)

include_directories(${DART_INCLUDE_DIRS})
# include_directories(${LIBCMAES_INCLUDE_DIR})
# set(LIBS ${DART_LIBRARIES} ${libcmaes_LIBRARY})

file(GLOB srcs "*.cpp" "*.hpp" ".h")
add_executable(${PROJECT_NAME} ${srcs})

# target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES} cmaes)
target_link_libraries(${PROJECT_NAME} ${DART_LIBRARIES})
