cmake_minimum_required(VERSION 3.8)
project(my_demo_lib)

#=========================== 常规的编译器设置===============================================
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#=========================== 常规的编译器设置===============================================

# 查找依赖文件
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# 添加源文件，生成库
add_library(my_demo_lib SHARED
  include/my_demo_lib/my_demo_lib.hpp
  src/my_demo_lib.cpp
)

# 用于代替传统的target_link_libraries
ament_target_dependencies(my_demo_lib
  PUBLIC
  rclcpp
)

# 注意不能直接写rclcpp，只能用${rclcpp_LIBRARIES}
#target_link_libraries(my_demo_lib ${rclcpp_LIBRARIES})

# 添加头文件地址，允许通过以下方法查找头文件
target_include_directories(my_demo_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 导出一些信息（包括头文件和库），为了使下游文件可以访问
ament_export_targets(my_demo_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
)

# ament_export_include_directories(include)

# 注册 导出头文件
install(
  DIRECTORY include/
  DESTINATION include
)

# 注册 导出库文件
install(
  TARGETS my_demo_lib
  EXPORT my_demo_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()


# cmake_minimum_required(VERSION 3.5)
# project(rl2_logging)
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)

# add_library(rl2_logging src/log.cpp)

# target_link_libraries(rl2_logging ${rclcpp_LIBRARIES})

# target_include_directories(rl2_logging PUBLIC
#   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
#   "$<INSTALL_INTERFACE:include>"
#   ${rclcpp_INCLUDE_DIRS}
# )

# ament_target_dependencies(rl2_logging rclcpp)
# ament_export_dependencies(rclcpp)
# ament_export_interfaces(export_rl2_logging HAS_LIBRARY_TARGET)

# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}/
# )

# install(
#   TARGETS rl2_logging
#   EXPORT export_rl2_logging
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# ament_package()
