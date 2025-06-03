;; Auto-generated. Do not edit!


(when (boundp 'RobotCar::voice2robot)
  (if (not (find-package "ROBOTCAR"))
    (make-package "ROBOTCAR"))
  (shadow 'voice2robot (find-package "ROBOTCAR")))
(unless (find-package "ROBOTCAR::VOICE2ROBOT")
  (make-package "ROBOTCAR::VOICE2ROBOT"))
(unless (find-package "ROBOTCAR::VOICE2ROBOTREQUEST")
  (make-package "ROBOTCAR::VOICE2ROBOTREQUEST"))
(unless (find-package "ROBOTCAR::VOICE2ROBOTRESPONSE")
  (make-package "ROBOTCAR::VOICE2ROBOTRESPONSE"))

(in-package "ROS")





(defclass RobotCar::voice2robotRequest
  :super ros::object
  :slots (_room_point ))

(defmethod RobotCar::voice2robotRequest
  (:init
   (&key
    ((:room_point __room_point) 0)
    )
   (send-super :init)
   (setq _room_point (round __room_point))
   self)
  (:room_point
   (&optional __room_point)
   (if __room_point (setq _room_point __room_point)) _room_point)
  (:serialization-length
   ()
   (+
    ;; int32 _room_point
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _room_point
       (write-long _room_point s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _room_point
     (setq _room_point (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass RobotCar::voice2robotResponse
  :super ros::object
  :slots (_roompoint_check ))

(defmethod RobotCar::voice2robotResponse
  (:init
   (&key
    ((:roompoint_check __roompoint_check) nil)
    )
   (send-super :init)
   (setq _roompoint_check __roompoint_check)
   self)
  (:roompoint_check
   (&optional (__roompoint_check :null))
   (if (not (eq __roompoint_check :null)) (setq _roompoint_check __roompoint_check)) _roompoint_check)
  (:serialization-length
   ()
   (+
    ;; bool _roompoint_check
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _roompoint_check
       (if _roompoint_check (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _roompoint_check
     (setq _roompoint_check (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass RobotCar::voice2robot
  :super ros::object
  :slots ())

(setf (get RobotCar::voice2robot :md5sum-) "983f78df28f3a33759c7924630309e9f")
(setf (get RobotCar::voice2robot :datatype-) "RobotCar/voice2robot")
(setf (get RobotCar::voice2robot :request) RobotCar::voice2robotRequest)
(setf (get RobotCar::voice2robot :response) RobotCar::voice2robotResponse)

(defmethod RobotCar::voice2robotRequest
  (:response () (instance RobotCar::voice2robotResponse :init)))

(setf (get RobotCar::voice2robotRequest :md5sum-) "983f78df28f3a33759c7924630309e9f")
(setf (get RobotCar::voice2robotRequest :datatype-) "RobotCar/voice2robotRequest")
(setf (get RobotCar::voice2robotRequest :definition-)
      "int32 room_point

---
bool roompoint_check
")

(setf (get RobotCar::voice2robotResponse :md5sum-) "983f78df28f3a33759c7924630309e9f")
(setf (get RobotCar::voice2robotResponse :datatype-) "RobotCar/voice2robotResponse")
(setf (get RobotCar::voice2robotResponse :definition-)
      "int32 room_point

---
bool roompoint_check
")



(provide :RobotCar/voice2robot "983f78df28f3a33759c7924630309e9f")


