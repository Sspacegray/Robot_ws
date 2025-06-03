; Auto-generated. Do not edit!


(cl:in-package RobotCar-srv)


;//! \htmlinclude voice2robot-request.msg.html

(cl:defclass <voice2robot-request> (roslisp-msg-protocol:ros-message)
  ((room_point
    :reader room_point
    :initarg :room_point
    :type cl:integer
    :initform 0))
)

(cl:defclass voice2robot-request (<voice2robot-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <voice2robot-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'voice2robot-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name RobotCar-srv:<voice2robot-request> is deprecated: use RobotCar-srv:voice2robot-request instead.")))

(cl:ensure-generic-function 'room_point-val :lambda-list '(m))
(cl:defmethod room_point-val ((m <voice2robot-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCar-srv:room_point-val is deprecated.  Use RobotCar-srv:room_point instead.")
  (room_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <voice2robot-request>) ostream)
  "Serializes a message object of type '<voice2robot-request>"
  (cl:let* ((signed (cl:slot-value msg 'room_point)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <voice2robot-request>) istream)
  "Deserializes a message object of type '<voice2robot-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'room_point) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<voice2robot-request>)))
  "Returns string type for a service object of type '<voice2robot-request>"
  "RobotCar/voice2robotRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'voice2robot-request)))
  "Returns string type for a service object of type 'voice2robot-request"
  "RobotCar/voice2robotRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<voice2robot-request>)))
  "Returns md5sum for a message object of type '<voice2robot-request>"
  "983f78df28f3a33759c7924630309e9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'voice2robot-request)))
  "Returns md5sum for a message object of type 'voice2robot-request"
  "983f78df28f3a33759c7924630309e9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<voice2robot-request>)))
  "Returns full string definition for message of type '<voice2robot-request>"
  (cl:format cl:nil "int32 room_point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'voice2robot-request)))
  "Returns full string definition for message of type 'voice2robot-request"
  (cl:format cl:nil "int32 room_point~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <voice2robot-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <voice2robot-request>))
  "Converts a ROS message object to a list"
  (cl:list 'voice2robot-request
    (cl:cons ':room_point (room_point msg))
))
;//! \htmlinclude voice2robot-response.msg.html

(cl:defclass <voice2robot-response> (roslisp-msg-protocol:ros-message)
  ((roompoint_check
    :reader roompoint_check
    :initarg :roompoint_check
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass voice2robot-response (<voice2robot-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <voice2robot-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'voice2robot-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name RobotCar-srv:<voice2robot-response> is deprecated: use RobotCar-srv:voice2robot-response instead.")))

(cl:ensure-generic-function 'roompoint_check-val :lambda-list '(m))
(cl:defmethod roompoint_check-val ((m <voice2robot-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCar-srv:roompoint_check-val is deprecated.  Use RobotCar-srv:roompoint_check instead.")
  (roompoint_check m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <voice2robot-response>) ostream)
  "Serializes a message object of type '<voice2robot-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'roompoint_check) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <voice2robot-response>) istream)
  "Deserializes a message object of type '<voice2robot-response>"
    (cl:setf (cl:slot-value msg 'roompoint_check) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<voice2robot-response>)))
  "Returns string type for a service object of type '<voice2robot-response>"
  "RobotCar/voice2robotResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'voice2robot-response)))
  "Returns string type for a service object of type 'voice2robot-response"
  "RobotCar/voice2robotResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<voice2robot-response>)))
  "Returns md5sum for a message object of type '<voice2robot-response>"
  "983f78df28f3a33759c7924630309e9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'voice2robot-response)))
  "Returns md5sum for a message object of type 'voice2robot-response"
  "983f78df28f3a33759c7924630309e9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<voice2robot-response>)))
  "Returns full string definition for message of type '<voice2robot-response>"
  (cl:format cl:nil "bool roompoint_check~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'voice2robot-response)))
  "Returns full string definition for message of type 'voice2robot-response"
  (cl:format cl:nil "bool roompoint_check~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <voice2robot-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <voice2robot-response>))
  "Converts a ROS message object to a list"
  (cl:list 'voice2robot-response
    (cl:cons ':roompoint_check (roompoint_check msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'voice2robot)))
  'voice2robot-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'voice2robot)))
  'voice2robot-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'voice2robot)))
  "Returns string type for a service object of type '<voice2robot>"
  "RobotCar/voice2robot")