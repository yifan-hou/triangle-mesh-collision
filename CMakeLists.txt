cmake_minimum_required(VERSION 2.6)
project(TriangleMeshCollision)

include_directories(include/
  /usr/include/eigen3)

# CGAL
find_package(CGAL)

##
## Build
##
add_library(${PROJECT_NAME} SHARED
	src/collision_detect.cpp
)
target_link_libraries(${PROJECT_NAME} CGAL::CGAL)

##
## Tests
##
add_executable(collision_detection_test src/collision_detection_test.cpp)
target_link_libraries(collision_detection_test ${PROJECT_NAME})

add_executable(ray_tracing_test src/ray_tracing_test.cpp)
target_link_libraries(ray_tracing_test ${PROJECT_NAME})

# ############################### Install
# headers
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.h"
    )

# binaries
install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
    )