; Auto-generated. Do not edit!


(cl:in-package me212bot-msg)


;//! \htmlinclude mobile_step.msg.html

(cl:defclass <mobile_step> (roslisp-msg-protocol:ros-message)
  ((step
    :reader step
    :initarg :step
    :type cl:integer
    :initform 0))
)

(cl:defclass mobile_step (<mobile_step>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mobile_step>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mobile_step)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name me212bot-msg:<mobile_step> is deprecated: use me212bot-msg:mobile_step instead.")))

(cl:ensure-generic-function 'step-val :lambda-list '(m))
(cl:defmethod step-val ((m <mobile_step>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader me212bot-msg:step-val is deprecated.  Use me212bot-msg:step instead.")
  (step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mobile_step>) ostream)
  "Serializes a message object of type '<mobile_step>"
  (cl:let* ((signed (cl:slot-value msg 'step)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mobile_step>) istream)
  "Deserializes a message object of type '<mobile_step>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'step) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mobile_step>)))
  "Returns string type for a message object of type '<mobile_step>"
  "me212bot/mobile_step")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mobile_step)))
  "Returns string type for a message object of type 'mobile_step"
  "me212bot/mobile_step")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mobile_step>)))
  "Returns md5sum for a message object of type '<mobile_step>"
  "99174260c0c07917ce2b7a46302ab7a8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mobile_step)))
  "Returns md5sum for a message object of type 'mobile_step"
  "99174260c0c07917ce2b7a46302ab7a8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mobile_step>)))
  "Returns full string definition for message of type '<mobile_step>"
  (cl:format cl:nil "int32 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mobile_step)))
  "Returns full string definition for message of type 'mobile_step"
  (cl:format cl:nil "int32 step~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mobile_step>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mobile_step>))
  "Converts a ROS message object to a list"
  (cl:list 'mobile_step
    (cl:cons ':step (step msg))
))
