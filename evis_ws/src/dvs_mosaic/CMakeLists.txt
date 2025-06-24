cmake_minimum_required(VERSION 3.8)
project(dvs_mosaic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dvs_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Adding all the source files containing other functions to the same executable
add_executable(mosaic_node src/mosaic_node.cpp src/image_util.cpp src/laplace.cpp src/poses_groundtruth.cpp src/geometry.cpp src/reconstruction.cpp src/publish.cpp src/mapping.cpp)
ament_target_dependencies(mosaic_node rclcpp std_msgs cv_bridge dvs_msgs sensor_msgs image_transport image_geometry ament_index_cpp)

target_include_directories(mosaic_node PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

include_directories(/usr/include)
link_directories(/usr/lib/x86_64-linux-gnu)
target_link_libraries(mosaic_node fftw3 fftw3_threads)
target_link_libraries(mosaic_node Eigen3::Eigen)

target_compile_features(mosaic_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS mosaic_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
