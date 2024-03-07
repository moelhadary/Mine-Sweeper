; Auto-generated. Do not edit!


(cl:in-package motor_control-msg)


;//! \htmlinclude PwmArray.msg.html

(cl:defclass <PwmArray> (roslisp-msg-protocol:ros-message)
  ((pwm_values
    :reader pwm_values
    :initarg :pwm_values
    :type (cl:vector cl:integer)
   :initform (cl:make-array 6 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass PwmArray (<PwmArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PwmArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PwmArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motor_control-msg:<PwmArray> is deprecated: use motor_control-msg:PwmArray instead.")))

(cl:ensure-generic-function 'pwm_values-val :lambda-list '(m))
(cl:defmethod pwm_values-val ((m <PwmArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motor_control-msg:pwm_values-val is deprecated.  Use motor_control-msg:pwm_values instead.")
  (pwm_values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PwmArray>) ostream)
  "Serializes a message object of type '<PwmArray>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'pwm_values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PwmArray>) istream)
  "Deserializes a message object of type '<PwmArray>"
  (cl:setf (cl:slot-value msg 'pwm_values) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'pwm_values)))
    (cl:dotimes (i 6)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PwmArray>)))
  "Returns string type for a message object of type '<PwmArray>"
  "motor_control/PwmArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PwmArray)))
  "Returns string type for a message object of type 'PwmArray"
  "motor_control/PwmArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PwmArray>)))
  "Returns md5sum for a message object of type '<PwmArray>"
  "bd3f5c6582467ef6ecc8c0b440e1fb77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PwmArray)))
  "Returns md5sum for a message object of type 'PwmArray"
  "bd3f5c6582467ef6ecc8c0b440e1fb77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PwmArray>)))
  "Returns full string definition for message of type '<PwmArray>"
  (cl:format cl:nil "int32[6] pwm_values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PwmArray)))
  "Returns full string definition for message of type 'PwmArray"
  (cl:format cl:nil "int32[6] pwm_values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PwmArray>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'pwm_values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PwmArray>))
  "Converts a ROS message object to a list"
  (cl:list 'PwmArray
    (cl:cons ':pwm_values (pwm_values msg))
))
