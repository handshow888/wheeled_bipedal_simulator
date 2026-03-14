cmake_minimum_required(VERSION 3.8)
project(wheeled_bipedal_simulator)

find_package(ament_cmake REQUIRED)

# 安装配置文件、启动文件、网格文件和URDF文件
install(
  DIRECTORY config launch meshes urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

# 安装package.xml
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}/
)

# 检查依赖包是否可用
find_package(ament_cmake REQUIRED)

# 添加ament依赖
ament_package()