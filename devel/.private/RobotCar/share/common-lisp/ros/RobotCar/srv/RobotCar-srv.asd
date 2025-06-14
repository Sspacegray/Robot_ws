
(cl:in-package :asdf)

(defsystem "RobotCar-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "voice2robot" :depends-on ("_package_voice2robot"))
    (:file "_package_voice2robot" :depends-on ("_package"))
    (:file "web2robot" :depends-on ("_package_web2robot"))
    (:file "_package_web2robot" :depends-on ("_package"))
  ))