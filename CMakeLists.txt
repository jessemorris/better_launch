cmake_minimum_required(VERSION 3.5)
project(better_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY example
        DESTINATION share/${PROJECT_NAME})

# Install the bl script that should be used instead of "ros2 launch/run"
install(PROGRAMS scripts/bl DESTINATION bin)

# Shell completion for the bl script
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/bl_shell_completion.bash")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/bl_shell_completion.zsh")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/bl_shell_completion.fish")

ament_python_install_package(${PROJECT_NAME})
ament_package()
