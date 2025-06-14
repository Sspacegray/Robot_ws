
(cl:in-package :asdf)

(defsystem "RobotCar-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "carinfo" :depends-on ("_package_carinfo"))
    (:file "_package_carinfo" :depends-on ("_package"))
    (:file "robotinfo" :depends-on ("_package_robotinfo"))
    (:file "_package_robotinfo" :depends-on ("_package"))
  ))