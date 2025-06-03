;; Auto-generated. Do not edit!


(when (boundp 'RobotCar::carinfo)
  (if (not (find-package "ROBOTCAR"))
    (make-package "ROBOTCAR"))
  (shadow 'carinfo (find-package "ROBOTCAR")))
(unless (find-package "ROBOTCAR::CARINFO")
  (make-package "ROBOTCAR::CARINFO"))

(in-package "ROS")
;;//! \htmlinclude carinfo.msg.html


(defclass RobotCar::carinfo
  :super ros::object
  :slots (_speed_x _speed_z _power ))

(defmethod RobotCar::carinfo
  (:init
   (&key
    ((:speed_x __speed_x) 0)
    ((:speed_z __speed_z) 0)
    ((:power __power) 0)
    )
   (send-super :init)
   (setq _speed_x (round __speed_x))
   (setq _speed_z (round __speed_z))
   (setq _power (round __power))
   self)
  (:speed_x
   (&optional __speed_x)
   (if __speed_x (setq _speed_x __speed_x)) _speed_x)
  (:speed_z
   (&optional __speed_z)
   (if __speed_z (setq _speed_z __speed_z)) _speed_z)
  (:power
   (&optional __power)
   (if __power (setq _power __power)) _power)
  (:serialization-length
   ()
   (+
    ;; int32 _speed_x
    4
    ;; int32 _speed_z
    4
    ;; int32 _power
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _speed_x
       (write-long _speed_x s)
     ;; int32 _speed_z
       (write-long _speed_z s)
     ;; int32 _power
       (write-long _power s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _speed_x
     (setq _speed_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _speed_z
     (setq _speed_z (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _power
     (setq _power (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get RobotCar::carinfo :md5sum-) "45c0e504be1d5153074c3d5b10d3b65d")
(setf (get RobotCar::carinfo :datatype-) "RobotCar/carinfo")
(setf (get RobotCar::carinfo :definition-)
      "int32 speed_x
int32 speed_z
int32 power


")



(provide :RobotCar/carinfo "45c0e504be1d5153074c3d5b10d3b65d")


