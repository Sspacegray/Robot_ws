# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xf_mic_asr_offline: 2 messages, 7 services")

set(MSG_I_FLAGS "-Ixf_mic_asr_offline:/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xf_mic_asr_offline_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" ""
)

get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_custom_target(_xf_mic_asr_offline_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xf_mic_asr_offline" "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_msg_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Services
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_cpp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Module File
_generate_module_cpp(xf_mic_asr_offline
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xf_mic_asr_offline_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xf_mic_asr_offline_generate_messages xf_mic_asr_offline_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_cpp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_mic_asr_offline_gencpp)
add_dependencies(xf_mic_asr_offline_gencpp xf_mic_asr_offline_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_mic_asr_offline_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_msg_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Services
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_eus(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Module File
_generate_module_eus(xf_mic_asr_offline
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xf_mic_asr_offline_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xf_mic_asr_offline_generate_messages xf_mic_asr_offline_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_eus _xf_mic_asr_offline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_mic_asr_offline_geneus)
add_dependencies(xf_mic_asr_offline_geneus xf_mic_asr_offline_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_mic_asr_offline_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_msg_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Services
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_lisp(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Module File
_generate_module_lisp(xf_mic_asr_offline
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xf_mic_asr_offline_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xf_mic_asr_offline_generate_messages xf_mic_asr_offline_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_lisp _xf_mic_asr_offline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_mic_asr_offline_genlisp)
add_dependencies(xf_mic_asr_offline_genlisp xf_mic_asr_offline_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_mic_asr_offline_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_msg_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Services
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_nodejs(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Module File
_generate_module_nodejs(xf_mic_asr_offline
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xf_mic_asr_offline_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xf_mic_asr_offline_generate_messages xf_mic_asr_offline_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_nodejs _xf_mic_asr_offline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_mic_asr_offline_gennodejs)
add_dependencies(xf_mic_asr_offline_gennodejs xf_mic_asr_offline_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_mic_asr_offline_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_msg_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Services
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)
_generate_srv_py(xf_mic_asr_offline
  "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
)

### Generating Module File
_generate_module_py(xf_mic_asr_offline
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xf_mic_asr_offline_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xf_mic_asr_offline_generate_messages xf_mic_asr_offline_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/Pcm_Msg.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/msg/position.msg" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Offline_Result_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Major_Mic_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Start_Record_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Awake_Word_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Set_Led_On_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/agrobot/Robot_ws/src/xf_mic_asr_offline/srv/Get_Awake_Angle_srv.srv" NAME_WE)
add_dependencies(xf_mic_asr_offline_generate_messages_py _xf_mic_asr_offline_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xf_mic_asr_offline_genpy)
add_dependencies(xf_mic_asr_offline_genpy xf_mic_asr_offline_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xf_mic_asr_offline_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xf_mic_asr_offline
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xf_mic_asr_offline_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xf_mic_asr_offline
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xf_mic_asr_offline_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xf_mic_asr_offline
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xf_mic_asr_offline_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xf_mic_asr_offline
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xf_mic_asr_offline_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xf_mic_asr_offline
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xf_mic_asr_offline_generate_messages_py std_msgs_generate_messages_py)
endif()
