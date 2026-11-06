cmake_minimum_required(VERSION 3.8)
project(gca_system_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommandContext.msg" "msg/MotionTarget.msg" "msg/MotionLimits.msg"
  "msg/StopStatus.msg" "msg/GcaState.msg" "msg/ManualInputQuality.msg"
  "msg/JogCommand.msg" "msg/TeleopState.msg"
  "srv/ControlSession.srv" "srv/ManageDevice.srv" "srv/ExecutionLease.srv"
  "srv/GetCommandStatus.srv" "srv/GetCapabilities.srv" "srv/ApplyConfig.srv"
  "srv/InjectFault.srv" "action/ExecuteMotion.action"
  DEPENDENCIES builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
