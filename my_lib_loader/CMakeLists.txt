cmake_minimum_required(VERSION 3.8)
project(my_lib_loader)

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

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED) # 可以不写此语句
find_package(my_demo_lib REQUIRED)

# 添加文件，只需要显示引用cpp文件即可，不必包含hpp
add_executable(my_lib_loader
  src/my_lib_loader.cpp
)

# 替代传统的target_link_libraries，将软件包链接到依赖项
ament_target_dependencies(my_lib_loader
  my_demo_lib
  rclcpp
)

# -----------------传统方法---------------------------
# include_directories(
#         ../../install/my_demo_lib/include
# )

# target_link_libraries(my_lib_loader
#   my_demo_lib::my_demo_lib ${rclcpp_LIBRARIES}
# )
# -----------------传统方法---------------------------

# 安装可执行文件
install(TARGETS my_lib_loader
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
