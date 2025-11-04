cmake_minimum_required(VERSION 3.16)
project(kf_gins_ros2_native)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 让所有静态库都可被共享库链接
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===== 仅使用“适配层 + 原仓库核心”方案 =====
# KF-GINS 源码所在根目录（确保该路径下有 src/kf-gins/gi_engine.cpp 等）
set(KF_GINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/KF-GINS)

# 编译并加入适配层（会顺带把 gi_engine/insmech 编进来）
add_subdirectory(external/kf_gins_adapter)

# 你的节点库
set(SRC src/kf_node.cpp src/geo.cpp)
set(HDR
  include/kf_gins_ros2_native/kf_core_interface.hpp
  include/kf_gins_ros2_native/geo.hpp
  include/kf_gins_ros2_native/node_factory.hpp
)
ament_auto_add_library(kf_gins_node_lib SHARED ${SRC} ${HDR})

# ★ 统一使用“无关键字”签名，避免与 ament 的内部调用冲突
target_link_libraries(kf_gins_node_lib kf_gins_core_adapter)

# 让节点能找到适配头文件（create_kf_core 等）
target_include_directories(kf_gins_node_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/external/kf_gins_adapter
)

# 让适配层也能看到本包的 include/
target_include_directories(kf_gins_core_adapter PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 可执行体
ament_auto_add_executable(kf_gins_node src/main.cpp)
target_link_libraries(kf_gins_node kf_gins_node_lib)

# 安装 Python 节点到 libexec（ros2 launch 会从这里找）
install(PROGRAMS
  scripts/imu_flu_to_frd.py
  scripts/odom_to_path.py
  # 若需要用 launch 起 hover_move，也一起装：
  # scripts/hover_move.py
  DESTINATION lib/${PROJECT_NAME}
)

# 安装
install(TARGETS kf_gins_node kf_gins_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION launch)
install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})

ament_package()
