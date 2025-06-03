;; Auto-generated. Do not edit!


(when (boundp 'RobotCar::robotinfo)
  (if (not (find-package "ROBOTCAR"))
    (make-package "ROBOTCAR"))
  (shadow 'robotinfo (find-package "ROBOTCAR")))
(unless (find-package "ROBOTCAR::ROBOTINFO")
  (make-package "ROBOTCAR::ROBOTINFO"))

(in-package "ROS")
;;//! \htmlinclude robotinfo.msg.html


(defclass RobotCar::robotinfo
  :super ros::object
  :slots (_robotstate _robotvoltage _lastroompoint ))

(defmethod RobotCar::robotinfo
  (:init
   (&key
    ((:robotstate __robotstate) 0)
    ((:robotvoltage __robotvoltage) 0)
    ((:lastroompoint __lastroompoint) 0)
    )
   (send-super :init)
   (setq _robotstate (round __robotstate))
   (setq _robotvoltage (round __robotvoltage))
   (setq _lastroompoint (round __lastroompoint))
   self)
  (:robotstate
   (&optional __robotstate)
   (if __robotstate (setq _robotstate __robotstate)) _robotstate)
  (:robotvoltage
   (&optional __robotvoltage)
   (if __robotvoltage (setq _robotvoltage __robotvoltage)) _robotvoltage)
  (:lastroompoint
   (&optional __lastroompoint)
   (if __lastroompoint (setq _lastroompoint __lastroompoint)) _lastroompoint)
  (:serialization-length
   ()
   (+
    ;; int32 _robotstate
    4
    ;; int32 _robotvoltage
    4
    ;; int32 _lastroompoint
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _robotstate
       (write-long _robotstate s)
     ;; int32 _robotvoltage
       (write-long _robotvoltage s)
     ;; int32 _lastroompoint
       (write-long _lastroompoint s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _robotstate
     (setq _robotstate (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _robotvoltage
     (setq _robotvoltage (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _lastroompoint
     (setq _lastroompoint (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get RobotCar::robotinfo :md5sum-) "00ba0d14f8e4b705d2bc45e5c870cf2b")
(setf (get RobotCar::robotinfo :datatype-) "RobotCar/robotinfo")
(setf (get RobotCar::robotinfo :definition-)
      "int32 robotstate
int32 robotvoltage
int32 lastroompoint
")



(provide :RobotCar/robotinfo "00ba0d14f8e4b705d2bc45e5c870cf2b")


