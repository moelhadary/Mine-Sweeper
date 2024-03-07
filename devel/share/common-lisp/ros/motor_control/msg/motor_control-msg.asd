
(cl:in-package :asdf)

(defsystem "motor_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PwmArray" :depends-on ("_package_PwmArray"))
    (:file "_package_PwmArray" :depends-on ("_package"))
  ))