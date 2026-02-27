cmake_minimum_required(VERSION 3.8)
project(lidar_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Indica que puede encontrar los headers de las librerias que no son de ROS2 en el include
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Crea el ejecutable (luego crearemos el archivo .cpp)
add_executable(localizador_node src/localizador_node.cpp)


# Enlaza los binarios de las librerias (.cpp traducido) con el ejecutable (codigo final en binario con las librerias y todo pegado)
target_link_libraries(localizador_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# Librerias de ROS2. Como son de ROS2 el compilador ya sabe donde estan los headers, se puede hacer la lista
ament_target_dependencies(localizador_node 
  rclcpp
  sensor_msgs
  laser_geometry
  pcl_conversions
  pcl_ros
)

# Instala el ejecutable
install(TARGETS
  localizador_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()