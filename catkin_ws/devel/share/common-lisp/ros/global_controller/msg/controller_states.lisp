; Auto-generated. Do not edit!


(cl:in-package global_controller-msg)


;//! \htmlinclude controller_states.msg.html

(cl:defclass <controller_states> (roslisp-msg-protocol:ros-message)
  ((is_driving_automatically
    :reader is_driving_automatically
    :initarg :is_driving_automatically
    :type cl:boolean
    :initform cl:nil)
   (is_allowed_to_drive
    :reader is_allowed_to_drive
    :initarg :is_allowed_to_drive
    :type cl:boolean
    :initform cl:nil)
   (tiz_is_boosted
    :reader tiz_is_boosted
    :initarg :tiz_is_boosted
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass controller_states (<controller_states>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <controller_states>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'controller_states)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name global_controller-msg:<controller_states> is deprecated: use global_controller-msg:controller_states instead.")))

(cl:ensure-generic-function 'is_driving_automatically-val :lambda-list '(m))
(cl:defmethod is_driving_automatically-val ((m <controller_states>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:is_driving_automatically-val is deprecated.  Use global_controller-msg:is_driving_automatically instead.")
  (is_driving_automatically m))

(cl:ensure-generic-function 'is_allowed_to_drive-val :lambda-list '(m))
(cl:defmethod is_allowed_to_drive-val ((m <controller_states>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:is_allowed_to_drive-val is deprecated.  Use global_controller-msg:is_allowed_to_drive instead.")
  (is_allowed_to_drive m))

(cl:ensure-generic-function 'tiz_is_boosted-val :lambda-list '(m))
(cl:defmethod tiz_is_boosted-val ((m <controller_states>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:tiz_is_boosted-val is deprecated.  Use global_controller-msg:tiz_is_boosted instead.")
  (tiz_is_boosted m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <controller_states>) ostream)
  "Serializes a message object of type '<controller_states>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_driving_automatically) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_allowed_to_drive) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'tiz_is_boosted) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <controller_states>) istream)
  "Deserializes a message object of type '<controller_states>"
    (cl:setf (cl:slot-value msg 'is_driving_automatically) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'is_allowed_to_drive) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'tiz_is_boosted) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<controller_states>)))
  "Returns string type for a message object of type '<controller_states>"
  "global_controller/controller_states")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'controller_states)))
  "Returns string type for a message object of type 'controller_states"
  "global_controller/controller_states")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<controller_states>)))
  "Returns md5sum for a message object of type '<controller_states>"
  "656034952bcd2a7d6b51db3480a3adeb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'controller_states)))
  "Returns md5sum for a message object of type 'controller_states"
  "656034952bcd2a7d6b51db3480a3adeb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<controller_states>)))
  "Returns full string definition for message of type '<controller_states>"
  (cl:format cl:nil "bool is_driving_automatically~%bool is_allowed_to_drive~%bool tiz_is_boosted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'controller_states)))
  "Returns full string definition for message of type 'controller_states"
  (cl:format cl:nil "bool is_driving_automatically~%bool is_allowed_to_drive~%bool tiz_is_boosted~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <controller_states>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <controller_states>))
  "Converts a ROS message object to a list"
  (cl:list 'controller_states
    (cl:cons ':is_driving_automatically (is_driving_automatically msg))
    (cl:cons ':is_allowed_to_drive (is_allowed_to_drive msg))
    (cl:cons ':tiz_is_boosted (tiz_is_boosted msg))
))
