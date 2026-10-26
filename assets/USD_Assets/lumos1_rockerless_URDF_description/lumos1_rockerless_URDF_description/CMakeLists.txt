cmake_minimum_required(VERSION 3.8)
project(lumos1_rockerless_URDF_description)

find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes config worlds materials DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py")
install(PROGRAMS scripts/prepare_site20.py scripts/prepare_lunaryard.py scripts/respawn_rover.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
ament_package()
