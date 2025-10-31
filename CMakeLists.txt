cmake_minimum_required(VERSION 3.18)
project(benchmarking_3dcpp)

set(CMAKE_CXX_STANDARD 23)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#enable cuda language
enable_language(CUDA)

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

find_package(Open3D REQUIRED)

find_package(CUDAToolkit REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED COMPONENTS tbb)

# Enable CUDA if available
if(CMAKE_CUDA_COMPILER)
    set(USE_CUDA TRUE)
    message(STATUS "CUDA found, building with GPU acceleration")
    
    # Set CUDA architectures
    set(CMAKE_CUDA_ARCHITECTURES "70;75;80;86")
    
    # Set CUDA compiler flags
    set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -std=c++17 --expt-relaxed-constexpr -O3")
    set(CMAKE_CUDA_FLAGS_RELEASE "${CMAKE_CUDA_FLAGS_RELEASE} -O3 -DNDEBUG")
    
# else()
#     set(USE_CUDA FALSE)
#     message(STATUS "CUDA not found, building CPU-only version")
endif()

include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

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

add_executable(benchmarking_3dcpp_node 
  src/benchmarking_3dcpp_node.cpp
  src/benchmarking_3dcpp.cpp
  src/eval/surface_sampler.cpp
  src/eval/coverage_evaluator.cpp
  src/alg/nuc.cpp)




# Link CUDA libraries
target_link_libraries(benchmarking_3dcpp_node
    cuda_kernels
    ${TBB_LIBRARIES}
    yaml-cpp
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
)

# Add compiler flags for OpenMP
target_link_libraries(benchmarking_3dcpp_node OpenMP::OpenMP_CXX)
# target_compile_options(benchmarking_3dcpp_node PRIVATE
#     $<$<COMPILE_LANGUAGE:CXX>:-fopenmp>
# )

install(TARGETS
  benchmarking_3dcpp_node
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
