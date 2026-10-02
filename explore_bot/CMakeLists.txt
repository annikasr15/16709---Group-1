cmake_minimum_required(VERSION 3.8)
project(explore_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf worlds maps config launch scripts meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/send_goal_template.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY urdf worlds launch DESTINATION share/${PROJECT_NAME})

ament_package()
