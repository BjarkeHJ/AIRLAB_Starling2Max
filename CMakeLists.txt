cmake_minimum_required(VERSION 3.8)
project(starling2max_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(starling2max)

install(
    DIRECTORY models px4 config
    DESTINATION share/${PROJECT_NAME}
)

# add 'launch' aboce once launch/view.launch.py exists

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/gz_resource_path.dsv.in")

ament_package()