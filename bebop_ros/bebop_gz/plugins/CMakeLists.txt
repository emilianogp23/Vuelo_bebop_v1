cmake_minimum_required(VERSION 3.10)
project(bebop_gz_plugins)

# Los plugins compilan con dos versiones de Gazebo:
#   - Harmonic (gz-sim8)          → ROS 2 Jazzy  (Ubuntu 24.04, escritorio)
#   - Fortress (ignition-gazebo6) → ROS 2 Humble (Ubuntu 22.04, laptop)
# Se usa Harmonic si está instalado; si no, Fortress. gz_compat.hh define las
# macros gz*/GZ_* que no existen en Fortress.
find_package(gz-sim8 QUIET)

if(gz-sim8_FOUND)
  set(BEBOP_GZ_VERSION "harmonic")
  find_package(gz-cmake3 REQUIRED)
  find_package(gz-transport13 REQUIRED)
  find_package(gz-msgs10 REQUIRED)
  find_package(gz-plugin2 REQUIRED COMPONENTS register)
  find_package(gz-math7 REQUIRED)
  find_package(gz-common5 REQUIRED)

  set(GZ_PLUGIN_LIBS
    gz-sim8::gz-sim8
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-plugin2::register
    gz-math7::gz-math7
    gz-common5::gz-common5
  )
  set(GZ_PLUGIN_DEFINITIONS "")
else()
  find_package(ignition-gazebo6 QUIET)
  if(NOT ignition-gazebo6_FOUND)
    message(FATAL_ERROR
      "No se encontró Gazebo Harmonic (gz-sim8) ni Fortress (ignition-gazebo6). "
      "Instala gz-harmonic (Jazzy) o ignition-fortress (Humble).")
  endif()
  set(BEBOP_GZ_VERSION "fortress")
  find_package(ignition-cmake2 REQUIRED)
  find_package(ignition-transport11 REQUIRED)
  find_package(ignition-msgs8 REQUIRED)
  find_package(ignition-plugin1 REQUIRED COMPONENTS register)
  find_package(ignition-math6 REQUIRED)
  find_package(ignition-common4 REQUIRED)

  set(GZ_PLUGIN_LIBS
    ignition-gazebo6::ignition-gazebo6
    ignition-transport11::ignition-transport11
    ignition-msgs8::ignition-msgs8
    ignition-plugin1::register
    ignition-math6::ignition-math6
    ignition-common4::ignition-common4
  )
  set(GZ_PLUGIN_DEFINITIONS BEBOP_GZ_FORTRESS)
endif()

message(STATUS "bebop_gz: compilando plugins para Gazebo ${BEBOP_GZ_VERSION}")
# Disponible para el CMakeLists.txt padre (hook de entorno)
set(BEBOP_GZ_VERSION "${BEBOP_GZ_VERSION}" PARENT_SCOPE)

# Plugin 1: SetPosePlugin
add_library(SetPosePlugin SHARED SetPosePlugin.cc)
target_link_libraries(SetPosePlugin ${GZ_PLUGIN_LIBS})
target_compile_definitions(SetPosePlugin PRIVATE ${GZ_PLUGIN_DEFINITIONS})

# Plugin 2: RobotPosePublisher
add_library(RobotPosePublisher SHARED RobotPosePublisher.cc)
target_link_libraries(RobotPosePublisher ${GZ_PLUGIN_LIBS})
target_compile_definitions(RobotPosePublisher PRIVATE ${GZ_PLUGIN_DEFINITIONS})

# Installed next to the other bebop_gz assets so a single
# GZ_SIM_SYSTEM_PLUGIN_PATH entry (set by the bebop_gz environment hook)
# is enough to find them.
install(
  TARGETS SetPosePlugin RobotPosePublisher
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
