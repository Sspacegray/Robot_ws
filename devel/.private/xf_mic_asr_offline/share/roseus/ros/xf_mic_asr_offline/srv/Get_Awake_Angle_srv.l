;; Auto-generated. Do not edit!


(when (boundp 'xf_mic_asr_offline::Get_Awake_Angle_srv)
  (if (not (find-package "XF_MIC_ASR_OFFLINE"))
    (make-package "XF_MIC_ASR_OFFLINE"))
  (shadow 'Get_Awake_Angle_srv (find-package "XF_MIC_ASR_OFFLINE")))
(unless (find-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRV")
  (make-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRV"))
(unless (find-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRVREQUEST")
  (make-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRVREQUEST"))
(unless (find-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRVRESPONSE")
  (make-package "XF_MIC_ASR_OFFLINE::GET_AWAKE_ANGLE_SRVRESPONSE"))

(in-package "ROS")





(defclass xf_mic_asr_offline::Get_Awake_Angle_srvRequest
  :super ros::object
  :slots (_get_awake_angle ))

(defmethod xf_mic_asr_offline::Get_Awake_Angle_srvRequest
  (:init
   (&key
    ((:get_awake_angle __get_awake_angle) 0)
    )
   (send-super :init)
   (setq _get_awake_angle (round __get_awake_angle))
   self)
  (:get_awake_angle
   (&optional __get_awake_angle)
   (if __get_awake_angle (setq _get_awake_angle __get_awake_angle)) _get_awake_angle)
  (:serialization-length
   ()
   (+
    ;; int32 _get_awake_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _get_awake_angle
       (write-long _get_awake_angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _get_awake_angle
     (setq _get_awake_angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass xf_mic_asr_offline::Get_Awake_Angle_srvResponse
  :super ros::object
  :slots (_result _awake_angle _fail_reason ))

(defmethod xf_mic_asr_offline::Get_Awake_Angle_srvResponse
  (:init
   (&key
    ((:result __result) "")
    ((:awake_angle __awake_angle) 0)
    ((:fail_reason __fail_reason) "")
    )
   (send-super :init)
   (setq _result (string __result))
   (setq _awake_angle (round __awake_angle))
   (setq _fail_reason (string __fail_reason))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:awake_angle
   (&optional __awake_angle)
   (if __awake_angle (setq _awake_angle __awake_angle)) _awake_angle)
  (:fail_reason
   (&optional __fail_reason)
   (if __fail_reason (setq _fail_reason __fail_reason)) _fail_reason)
  (:serialization-length
   ()
   (+
    ;; string _result
    4 (length _result)
    ;; int32 _awake_angle
    4
    ;; string _fail_reason
    4 (length _fail_reason)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _result
       (write-long (length _result) s) (princ _result s)
     ;; int32 _awake_angle
       (write-long _awake_angle s)
     ;; string _fail_reason
       (write-long (length _fail_reason) s) (princ _fail_reason s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _result
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _result (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _awake_angle
     (setq _awake_angle (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _fail_reason
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _fail_reason (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass xf_mic_asr_offline::Get_Awake_Angle_srv
  :super ros::object
  :slots ())

(setf (get xf_mic_asr_offline::Get_Awake_Angle_srv :md5sum-) "9a96c781963794bda6b872fc7788b6bb")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srv :datatype-) "xf_mic_asr_offline/Get_Awake_Angle_srv")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srv :request) xf_mic_asr_offline::Get_Awake_Angle_srvRequest)
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srv :response) xf_mic_asr_offline::Get_Awake_Angle_srvResponse)

(defmethod xf_mic_asr_offline::Get_Awake_Angle_srvRequest
  (:response () (instance xf_mic_asr_offline::Get_Awake_Angle_srvResponse :init)))

(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvRequest :md5sum-) "9a96c781963794bda6b872fc7788b6bb")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvRequest :datatype-) "xf_mic_asr_offline/Get_Awake_Angle_srvRequest")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvRequest :definition-)
      "int32 get_awake_angle #1,0
---
string result
int32 awake_angle
string fail_reason 

")

(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvResponse :md5sum-) "9a96c781963794bda6b872fc7788b6bb")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvResponse :datatype-) "xf_mic_asr_offline/Get_Awake_Angle_srvResponse")
(setf (get xf_mic_asr_offline::Get_Awake_Angle_srvResponse :definition-)
      "int32 get_awake_angle #1,0
---
string result
int32 awake_angle
string fail_reason 

")



(provide :xf_mic_asr_offline/Get_Awake_Angle_srv "9a96c781963794bda6b872fc7788b6bb")


