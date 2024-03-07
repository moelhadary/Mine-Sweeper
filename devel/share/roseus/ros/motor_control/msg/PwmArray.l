;; Auto-generated. Do not edit!


(when (boundp 'motor_control::PwmArray)
  (if (not (find-package "MOTOR_CONTROL"))
    (make-package "MOTOR_CONTROL"))
  (shadow 'PwmArray (find-package "MOTOR_CONTROL")))
(unless (find-package "MOTOR_CONTROL::PWMARRAY")
  (make-package "MOTOR_CONTROL::PWMARRAY"))

(in-package "ROS")
;;//! \htmlinclude PwmArray.msg.html


(defclass motor_control::PwmArray
  :super ros::object
  :slots (_pwm_values ))

(defmethod motor_control::PwmArray
  (:init
   (&key
    ((:pwm_values __pwm_values) (make-array 6 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _pwm_values __pwm_values)
   self)
  (:pwm_values
   (&optional __pwm_values)
   (if __pwm_values (setq _pwm_values __pwm_values)) _pwm_values)
  (:serialization-length
   ()
   (+
    ;; int32[6] _pwm_values
    (* 4    6)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[6] _pwm_values
     (dotimes (i 6)
       (write-long (elt _pwm_values i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[6] _pwm_values
   (dotimes (i (length _pwm_values))
     (setf (elt _pwm_values i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     )
   ;;
   self)
  )

(setf (get motor_control::PwmArray :md5sum-) "bd3f5c6582467ef6ecc8c0b440e1fb77")
(setf (get motor_control::PwmArray :datatype-) "motor_control/PwmArray")
(setf (get motor_control::PwmArray :definition-)
      "int32[6] pwm_values

")



(provide :motor_control/PwmArray "bd3f5c6582467ef6ecc8c0b440e1fb77")


