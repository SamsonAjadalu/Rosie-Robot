cmake_minimum_required(VERSION 3.22)
project(rosie_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(PROGRAMS
  scripts/arm_control_from_UI.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/synchronized_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})