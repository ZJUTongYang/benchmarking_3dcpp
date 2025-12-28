cmake_minimum_required(VERSION 3.8)
project(benchmarking_3dcpp_rviz_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/benchmarking_3dcpp_rviz_plugin/benchmarking_3dcpp_panel.hpp
)

add_library(benchmarking_3dcpp_panel src/benchmarking_3dcpp_panel.cpp ${MOC_FILES})
target_include_directories(benchmarking_3dcpp_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(benchmarking_3dcpp_panel
  pluginlib
  rviz_common
  ament_index_cpp
)
install(TARGETS benchmarking_3dcpp_panel
        EXPORT export_benchmarking_3dcpp_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES benchmarking_3dcpp_panel.xml
        DESTINATION share/${PROJECT_NAME}
)
install(FILES icons/classes/Benchmarking3DCPPPanel.svg
        DESTINATION share/${PROJECT_NAME}/icons/classes
)
ament_export_include_directories(include)
ament_export_targets(export_benchmarking_3dcpp_panel)
pluginlib_export_plugin_description_file(rviz_common benchmarking_3dcpp_panel.xml)

ament_package()
