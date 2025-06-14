# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "RobotCar: 2 messages, 2 services")

set(MSG_I_FLAGS "-IRobotCar:/home/agrobot/Robot_ws/src/RobotCar/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(RobotCar_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_custom_target(_RobotCar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RobotCar" "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_custom_target(_RobotCar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RobotCar" "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_custom_target(_RobotCar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RobotCar" "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_custom_target(_RobotCar_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "RobotCar" "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
)
_generate_msg_cpp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
)

### Generating Services
_generate_srv_cpp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
)
_generate_srv_cpp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
)

### Generating Module File
_generate_module_cpp(RobotCar
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(RobotCar_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(RobotCar_generate_messages RobotCar_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_cpp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_cpp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_cpp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_cpp _RobotCar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotCar_gencpp)
add_dependencies(RobotCar_gencpp RobotCar_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotCar_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
)
_generate_msg_eus(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
)

### Generating Services
_generate_srv_eus(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
)
_generate_srv_eus(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
)

### Generating Module File
_generate_module_eus(RobotCar
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(RobotCar_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(RobotCar_generate_messages RobotCar_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_eus _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_eus _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_eus _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_eus _RobotCar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotCar_geneus)
add_dependencies(RobotCar_geneus RobotCar_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotCar_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
)
_generate_msg_lisp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
)

### Generating Services
_generate_srv_lisp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
)
_generate_srv_lisp(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
)

### Generating Module File
_generate_module_lisp(RobotCar
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(RobotCar_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(RobotCar_generate_messages RobotCar_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_lisp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_lisp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_lisp _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_lisp _RobotCar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotCar_genlisp)
add_dependencies(RobotCar_genlisp RobotCar_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotCar_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
)
_generate_msg_nodejs(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
)

### Generating Services
_generate_srv_nodejs(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
)
_generate_srv_nodejs(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
)

### Generating Module File
_generate_module_nodejs(RobotCar
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(RobotCar_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(RobotCar_generate_messages RobotCar_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_nodejs _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_nodejs _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_nodejs _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_nodejs _RobotCar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotCar_gennodejs)
add_dependencies(RobotCar_gennodejs RobotCar_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotCar_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
)
_generate_msg_py(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
)

### Generating Services
_generate_srv_py(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
)
_generate_srv_py(RobotCar
  "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
)

### Generating Module File
_generate_module_py(RobotCar
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(RobotCar_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(RobotCar_generate_messages RobotCar_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/carinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_py _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/msg/robotinfo.msg" NAME_WE)
add_dependencies(RobotCar_generate_messages_py _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/web2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_py _RobotCar_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/RobotCar/srv/voice2robot.srv" NAME_WE)
add_dependencies(RobotCar_generate_messages_py _RobotCar_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(RobotCar_genpy)
add_dependencies(RobotCar_genpy RobotCar_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS RobotCar_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/RobotCar
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(RobotCar_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/RobotCar
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(RobotCar_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/RobotCar
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(RobotCar_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/RobotCar
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(RobotCar_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/RobotCar
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(RobotCar_generate_messages_py std_msgs_generate_messages_py)
endif()
