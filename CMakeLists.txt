cmake_minimum_required(VERSION 2.6)
project(TriangleMeshCollision)

include_directories(include/
  /usr/include/eigen3)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
find_package(LIBIGL REQUIRED QUIET)
if(LIBIGL_WITH_CGAL)
  find_package(CGAL REQUIRED)
  set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "CGAL's CMAKE Setup is super annoying ")
  include(${CGAL_USE_FILE})
endif()


if (NOT LIBIGL_FOUND)
   message(FATAL_ERROR "libigl not found --- You can download it using: \n git clone --recursive https://github.com/libigl/libigl.git ${PROJECT_SOURCE_DIR}/../libigl")
endif()

# Libigl requires a modern C++ compiler that supports c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

message("libigl includes: ${LIBIGL_INCLUDE_DIRS}")
message("libigl libraries: ${LIBIGL_LIBRARIES}")
message("libigl extra sources: ${LIBIGL_EXTRA_SOURCES}")
message("libigl extra libraries: ${LIBIGL_EXTRA_LIBRARIES}")
message("libigl definitions: ${LIBIGL_DEFINITIONS}")


add_library(${PROJECT_NAME} src/collision_detect.cpp src/bounding_box.cpp src/bvh.cpp)
target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw)

# FILE(GLOB SRCFILES src/*.cpp)
# add_executable(${PROJECT_NAME}_bin ${SRCFILES} ${LIBIGL_EXTRA_SOURCES})
# target_link_libraries(${PROJECT_NAME}_bin ${LIBIGL_LIBRARIES} ${LIBIGL_EXTRA_LIBRARIES} igl::core igl::opengl_glfw)
add_executable(collision_test src/main.cpp)
target_link_libraries(collision_test ${PROJECT_NAME} igl::core igl::opengl_glfw)


############################### Install
# headers
install(DIRECTORY include/
    DESTINATION /usr/local/include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
    )

# binaries
install(TARGETS ${PROJECT_NAME}
    DESTINATION /usr/local/lib/${PROJECT_NAME}
    )