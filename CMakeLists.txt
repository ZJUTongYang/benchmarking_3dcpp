cmake_minimum_required(VERSION 3.18)
project(benchmarking_3dcpp)

set(CMAKE_CXX_STANDARD 23)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuc_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(spdlog REQUIRED)
find_package(std_srvs REQUIRED)

find_package(Open3D REQUIRED)

find_package(glfw3 REQUIRED) # required for open3d visualization
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)
find_package(HDF5 REQUIRED COMPONENTS C CXX)

# Enable CUDA if available
find_package(CUDAToolkit QUIET)

if(CMAKE_CUDA_COMPILER)

    add_compile_definitions(USE_CUDA)

    #enable cuda language
    enable_language(CUDA)

    set(USE_CUDA TRUE)
    message(STATUS "CUDA found, building with GPU acceleration")
    
    # Set CUDA architectures
    set(CMAKE_CUDA_ARCHITECTURES "70;75;80;86")
    
    # Set CUDA compiler flags
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++17 --expt-relaxed-constexpr -O3")
    set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -O3 -DNDEBUG")
    
else()
    set(USE_CUDA FALSE)
    message(STATUS "CUDA not found, building CPU-only version")
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

if(USE_CUDA)

    # Add CUDA source files
    set(CUDA_SOURCES src/cuda/cuda_kernels.cu)

    # Create library for CUDA kernels with explicit language specification
    add_library(cuda_kernels STATIC)
    target_sources(cuda_kernels PRIVATE ${CUDA_SOURCES})

    # Set properties for CUDA compilation
    set_target_properties(cuda_kernels PROPERTIES
        CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}"
        CUDA_RUNTIME_LIBRARY Shared
        POSITION_INDEPENDENT_CODE ON
        LINKER_LANGUAGE CUDA
    )

    target_include_directories(cuda_kernels PRIVATE
        ${Eigen3_INCLUDE_DIRS}
        include
        ${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES}
    )

    # Link CUDA libraries to the kernel library
    target_link_libraries(cuda_kernels
        CUDA::cudart
        Eigen3::Eigen
        TBB::tbb
    )
endif()

add_executable(benchmarking_3dcpp_node 
  src/benchmarking_3dcpp_node.cpp
  src/benchmarking_3dcpp.cpp
  src/eval/surface_sampler.cpp
  src/eval/coverage_evaluator.cpp
  src/viz/benchmarking_viz.cpp
  src/robot_model/robot_model.cpp
  src/robot_model/circular.cpp
  src/robot_model/line_lidar.cpp
  src/utils/h5_helper.cpp
  src/alg/nuc.cpp)

if(USE_CUDA)
    target_link_libraries(benchmarking_3dcpp_node
        cuda_kernels
    )
endif()

# Link CUDA libraries
target_link_libraries(benchmarking_3dcpp_node
    TBB::tbb
    yaml-cpp
    ${HDF5_LIBRARIES}
    ${HDF5_HL_LIBRARIES}
    HDF5::HDF5
)

ament_target_dependencies(benchmarking_3dcpp_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_eigen
  nuc_msgs
  Open3D
  spdlog
  std_srvs
)

add_executable(benchmarking_viz_node
  src/benchmarking_viz_node.cpp
  src/viz/benchmarking_viz.cpp
  src/utils/h5_helper.cpp
)

target_include_directories(benchmarking_viz_node PRIVATE
  include
)

target_link_libraries(benchmarking_viz_node
  ${HDF5_LIBRARIES}
  ${HDF5_HL_LIBRARIES}
  HDF5::HDF5
)

ament_target_dependencies(benchmarking_viz_node
  rclcpp
  tf2_eigen
  visualization_msgs
  std_srvs
  sensor_msgs
)

# Add compiler flags for OpenMP
target_link_libraries(benchmarking_3dcpp_node 
  OpenMP::OpenMP_CXX
)

install(TARGETS
  benchmarking_3dcpp_node
  benchmarking_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scene
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

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
