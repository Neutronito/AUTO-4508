; Auto-generated. Do not edit!


(cl:in-package global_controller-msg)


;//! \htmlinclude gps_points.msg.html

(cl:defclass <gps_points> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (reached_status
    :reader reached_status
    :initarg :reached_status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gps_points (<gps_points>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gps_points>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gps_points)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name global_controller-msg:<gps_points> is deprecated: use global_controller-msg:gps_points instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <gps_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:latitude-val is deprecated.  Use global_controller-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <gps_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:longitude-val is deprecated.  Use global_controller-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'reached_status-val :lambda-list '(m))
(cl:defmethod reached_status-val ((m <gps_points>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader global_controller-msg:reached_status-val is deprecated.  Use global_controller-msg:reached_status instead.")
  (reached_status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gps_points>) ostream)
  "Serializes a message object of type '<gps_points>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'reached_status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gps_points>) istream)
  "Deserializes a message object of type '<gps_points>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'reached_status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gps_points>)))
  "Returns string type for a message object of type '<gps_points>"
  "global_controller/gps_points")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gps_points)))
  "Returns string type for a message object of type 'gps_points"
  "global_controller/gps_points")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gps_points>)))
  "Returns md5sum for a message object of type '<gps_points>"
  "560569517c52fdcf12619013f5dcbac6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gps_points)))
  "Returns md5sum for a message object of type 'gps_points"
  "560569517c52fdcf12619013f5dcbac6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gps_points>)))
  "Returns full string definition for message of type '<gps_points>"
  (cl:format cl:nil "float64 latitude~%float64 longitude~%bool reached_status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gps_points)))
  "Returns full string definition for message of type 'gps_points"
  (cl:format cl:nil "float64 latitude~%float64 longitude~%bool reached_status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gps_points>))
  (cl:+ 0
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gps_points>))
  "Converts a ROS message object to a list"
  (cl:list 'gps_points
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':reached_status (reached_status msg))
))
