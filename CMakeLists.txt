cmake_minimum_required(VERSION 3.10)
project(opt_camera)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/opt_camera_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC optsdk/include)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC optsdk/lib/amd64 )
  install(
    DIRECTORY optsdk/lib/amd64/
    DESTINATION lib
  )
  target_link_libraries(${PROJECT_NAME} 
  avcodec
  avfilter
  avformat
  avutil
  ImageConvert
  log4cpp
  OPTSDK
  OPTSDKGuiQt
  postproc
  RecordVideo
  swresample
  swscale
  VideoRender
  x264
  GCBase_gcc421_v3_0
  GenApi_gcc421_v3_0
  Log_gcc421_v3_0
  log4cpp_gcc421_v3_0
  MathParser_gcc421_v3_0
  NodeMapData_gcc421_v3_0
  XmlParser_gcc421_v3_0
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC optsdk/lib/arm64)
  install(
    DIRECTORY optsdk/lib/arm64/
    DESTINATION lib
  )
  target_link_libraries(${PROJECT_NAME} 
  avcodec
  avfilter
  avformat
  avutil
  GCBase_gcc485_v3_0
  GenApi_gcc485_v3_0
  ImageConvert
  Log_gcc485_v3_0
  log4cpp
  log4cpp_gcc485_v3_0
  MathParser_gcc485_v3_0
  NodeMapData_gcc485_v3_0
  OPTSDK
  OPTSDKGuiQt
  postproc
  RecordVideo
  swresample
  swscale
  x264
  XmlParser_gcc485_v3_0
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()



rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN opt_camera::OPTCameraNode
  EXECUTABLE ${PROJECT_NAME}_node
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()

  set(_linter_excludes
    optsdk/include/OPTApi.h
    optsdk/include/OPTDefines.h
  )
  ament_cpplint(EXCLUDE ${_linter_excludes})
endif()

# ############
# # Install ##
# ############

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)