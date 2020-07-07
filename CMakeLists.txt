cmake_minimum_required(VERSION 3.0.2)
project(bvp_exp)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  geometry_msgs
  tf2_ros
  roscpp
  visualization_msgs
)
catkin_package(
  LIBRARIES   bvp_exploration
  CATKIN_DEPENDS roscpp
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)

add_executable(${PROJECT_NAME}_node src/PotentialGrid.cpp src/main.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

