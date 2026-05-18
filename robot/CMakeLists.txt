cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SpeechSrv.srv"
  "srv/SpeechNLUSrv.srv"
)

install(
  PROGRAMS
    scripts/label_publisher.py
    scripts/common.py
    scripts/spawn_objects_node.py
    scripts/speech_asr_node.py
    scripts/speech_nlu_node.py
    scripts/task_dispatcher.py
    scripts/iflytek_recognize.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
