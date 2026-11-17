cmake_minimum_required(VERSION 3.8)
project(model_identification_usv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_executable(id_experiment src/experiment_node.cpp)
ament_target_dependencies(id_experiment rclcpp std_msgs geometry_msgs nav_msgs)

add_executable(id_fit src/fit_node.cpp)
ament_target_dependencies(id_fit rclcpp)
target_link_libraries(id_fit Eigen3::Eigen)

install(TARGETS id_experiment id_fit DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Gazebo (gz-sim 8, Harmonic) parts: the BodyWrench system and the identification worlds. Built
# only when gz-sim is found, so the package also builds on the vessel.
find_package(gz-sim8 QUIET)
find_package(gz-plugin2 QUIET COMPONENTS register)
find_package(gz-transport13 QUIET)
if(gz-sim8_FOUND AND gz-plugin2_FOUND AND gz-transport13_FOUND)
  add_library(${PROJECT_NAME}_body_wrench SHARED src/body_wrench_system.cpp)
  target_link_libraries(${PROJECT_NAME}_body_wrench
    gz-sim8::gz-sim8 gz-plugin2::register gz-transport13::gz-transport13)
  install(TARGETS ${PROJECT_NAME}_body_wrench DESTINATION lib)

  # one world per wind speed of the wave model (fft spectrum), worlds/id_wind<speed>.sdf
  foreach(WIND 0 2 4 6)
    configure_file(worlds/id_world.sdf.in ${CMAKE_CURRENT_BINARY_DIR}/worlds/id_wind${WIND}.sdf @ONLY)
  endforeach()
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/worlds DESTINATION share/${PROJECT_NAME})
else()
  message(STATUS "gz-sim8 not found: BodyWrench system and identification worlds not built")
endif()

ament_package()
