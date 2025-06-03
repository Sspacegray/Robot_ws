; Auto-generated. Do not edit!


(cl:in-package RobotCar-msg)


;//! \htmlinclude robotinfo.msg.html

(cl:defclass <robotinfo> (roslisp-msg-protocol:ros-message)
  ((robotstate
    :reader robotstate
    :initarg :robotstate
    :type cl:integer
    :initform 0)
   (robotvoltage
    :reader robotvoltage
    :initarg :robotvoltage
    :type cl:integer
    :initform 0)
   (lastroompoint
    :reader lastroompoint
    :initarg :lastroompoint
    :type cl:integer
    :initform 0))
)

(cl:defclass robotinfo (<robotinfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <robotinfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'robotinfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name RobotCar-msg:<robotinfo> is deprecated: use RobotCar-msg:robotinfo instead.")))

(cl:ensure-generic-function 'robotstate-val :lambda-list '(m))
(cl:defmethod robotstate-val ((m <robotinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCar-msg:robotstate-val is deprecated.  Use RobotCar-msg:robotstate instead.")
  (robotstate m))

(cl:ensure-generic-function 'robotvoltage-val :lambda-list '(m))
(cl:defmethod robotvoltage-val ((m <robotinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCar-msg:robotvoltage-val is deprecated.  Use RobotCar-msg:robotvoltage instead.")
  (robotvoltage m))

(cl:ensure-generic-function 'lastroompoint-val :lambda-list '(m))
(cl:defmethod lastroompoint-val ((m <robotinfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobotCar-msg:lastroompoint-val is deprecated.  Use RobotCar-msg:lastroompoint instead.")
  (lastroompoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <robotinfo>) ostream)
  "Serializes a message object of type '<robotinfo>"
  (cl:let* ((signed (cl:slot-value msg 'robotstate)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'robotvoltage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'lastroompoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <robotinfo>) istream)
  "Deserializes a message object of type '<robotinfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotstate) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'robotvoltage) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'lastroompoint) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<robotinfo>)))
  "Returns string type for a message object of type '<robotinfo>"
  "RobotCar/robotinfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'robotinfo)))
  "Returns string type for a message object of type 'robotinfo"
  "RobotCar/robotinfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<robotinfo>)))
  "Returns md5sum for a message object of type '<robotinfo>"
  "00ba0d14f8e4b705d2bc45e5c870cf2b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'robotinfo)))
  "Returns md5sum for a message object of type 'robotinfo"
  "00ba0d14f8e4b705d2bc45e5c870cf2b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<robotinfo>)))
  "Returns full string definition for message of type '<robotinfo>"
  (cl:format cl:nil "int32 robotstate~%int32 robotvoltage~%int32 lastroompoint~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'robotinfo)))
  "Returns full string definition for message of type 'robotinfo"
  (cl:format cl:nil "int32 robotstate~%int32 robotvoltage~%int32 lastroompoint~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <robotinfo>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <robotinfo>))
  "Converts a ROS message object to a list"
  (cl:list 'robotinfo
    (cl:cons ':robotstate (robotstate msg))
    (cl:cons ':robotvoltage (robotvoltage msg))
    (cl:cons ':lastroompoint (lastroompoint msg))
))
