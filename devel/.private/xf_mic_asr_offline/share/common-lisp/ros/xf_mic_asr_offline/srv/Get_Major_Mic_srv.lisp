; Auto-generated. Do not edit!


(cl:in-package xf_mic_asr_offline-srv)


;//! \htmlinclude Get_Major_Mic_srv-request.msg.html

(cl:defclass <Get_Major_Mic_srv-request> (roslisp-msg-protocol:ros-message)
  ((get_major_id
    :reader get_major_id
    :initarg :get_major_id
    :type cl:integer
    :initform 0))
)

(cl:defclass Get_Major_Mic_srv-request (<Get_Major_Mic_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_Major_Mic_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_Major_Mic_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xf_mic_asr_offline-srv:<Get_Major_Mic_srv-request> is deprecated: use xf_mic_asr_offline-srv:Get_Major_Mic_srv-request instead.")))

(cl:ensure-generic-function 'get_major_id-val :lambda-list '(m))
(cl:defmethod get_major_id-val ((m <Get_Major_Mic_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:get_major_id-val is deprecated.  Use xf_mic_asr_offline-srv:get_major_id instead.")
  (get_major_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_Major_Mic_srv-request>) ostream)
  "Serializes a message object of type '<Get_Major_Mic_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'get_major_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_Major_Mic_srv-request>) istream)
  "Deserializes a message object of type '<Get_Major_Mic_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'get_major_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_Major_Mic_srv-request>)))
  "Returns string type for a service object of type '<Get_Major_Mic_srv-request>"
  "xf_mic_asr_offline/Get_Major_Mic_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_Major_Mic_srv-request)))
  "Returns string type for a service object of type 'Get_Major_Mic_srv-request"
  "xf_mic_asr_offline/Get_Major_Mic_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_Major_Mic_srv-request>)))
  "Returns md5sum for a message object of type '<Get_Major_Mic_srv-request>"
  "68ec108b7ba6ff9867de0e21598a80d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_Major_Mic_srv-request)))
  "Returns md5sum for a message object of type 'Get_Major_Mic_srv-request"
  "68ec108b7ba6ff9867de0e21598a80d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_Major_Mic_srv-request>)))
  "Returns full string definition for message of type '<Get_Major_Mic_srv-request>"
  (cl:format cl:nil "int32 get_major_id #1,0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_Major_Mic_srv-request)))
  "Returns full string definition for message of type 'Get_Major_Mic_srv-request"
  (cl:format cl:nil "int32 get_major_id #1,0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_Major_Mic_srv-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_Major_Mic_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_Major_Mic_srv-request
    (cl:cons ':get_major_id (get_major_id msg))
))
;//! \htmlinclude Get_Major_Mic_srv-response.msg.html

(cl:defclass <Get_Major_Mic_srv-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:string
    :initform "")
   (mic_id
    :reader mic_id
    :initarg :mic_id
    :type cl:fixnum
    :initform 0)
   (fail_reason
    :reader fail_reason
    :initarg :fail_reason
    :type cl:string
    :initform ""))
)

(cl:defclass Get_Major_Mic_srv-response (<Get_Major_Mic_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Get_Major_Mic_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Get_Major_Mic_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xf_mic_asr_offline-srv:<Get_Major_Mic_srv-response> is deprecated: use xf_mic_asr_offline-srv:Get_Major_Mic_srv-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <Get_Major_Mic_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:result-val is deprecated.  Use xf_mic_asr_offline-srv:result instead.")
  (result m))

(cl:ensure-generic-function 'mic_id-val :lambda-list '(m))
(cl:defmethod mic_id-val ((m <Get_Major_Mic_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:mic_id-val is deprecated.  Use xf_mic_asr_offline-srv:mic_id instead.")
  (mic_id m))

(cl:ensure-generic-function 'fail_reason-val :lambda-list '(m))
(cl:defmethod fail_reason-val ((m <Get_Major_Mic_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xf_mic_asr_offline-srv:fail_reason-val is deprecated.  Use xf_mic_asr_offline-srv:fail_reason instead.")
  (fail_reason m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Get_Major_Mic_srv-response>) ostream)
  "Serializes a message object of type '<Get_Major_Mic_srv-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'result))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'result))
  (cl:let* ((signed (cl:slot-value msg 'mic_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'fail_reason))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'fail_reason))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Get_Major_Mic_srv-response>) istream)
  "Deserializes a message object of type '<Get_Major_Mic_srv-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'result) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mic_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fail_reason) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'fail_reason) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Get_Major_Mic_srv-response>)))
  "Returns string type for a service object of type '<Get_Major_Mic_srv-response>"
  "xf_mic_asr_offline/Get_Major_Mic_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_Major_Mic_srv-response)))
  "Returns string type for a service object of type 'Get_Major_Mic_srv-response"
  "xf_mic_asr_offline/Get_Major_Mic_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Get_Major_Mic_srv-response>)))
  "Returns md5sum for a message object of type '<Get_Major_Mic_srv-response>"
  "68ec108b7ba6ff9867de0e21598a80d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Get_Major_Mic_srv-response)))
  "Returns md5sum for a message object of type 'Get_Major_Mic_srv-response"
  "68ec108b7ba6ff9867de0e21598a80d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Get_Major_Mic_srv-response>)))
  "Returns full string definition for message of type '<Get_Major_Mic_srv-response>"
  (cl:format cl:nil "string result~%int8 mic_id~%string fail_reason ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Get_Major_Mic_srv-response)))
  "Returns full string definition for message of type 'Get_Major_Mic_srv-response"
  (cl:format cl:nil "string result~%int8 mic_id~%string fail_reason ~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Get_Major_Mic_srv-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'result))
     1
     4 (cl:length (cl:slot-value msg 'fail_reason))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Get_Major_Mic_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Get_Major_Mic_srv-response
    (cl:cons ':result (result msg))
    (cl:cons ':mic_id (mic_id msg))
    (cl:cons ':fail_reason (fail_reason msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Get_Major_Mic_srv)))
  'Get_Major_Mic_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Get_Major_Mic_srv)))
  'Get_Major_Mic_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Get_Major_Mic_srv)))
  "Returns string type for a service object of type '<Get_Major_Mic_srv>"
  "xf_mic_asr_offline/Get_Major_Mic_srv")