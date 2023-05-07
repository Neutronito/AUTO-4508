;; Auto-generated. Do not edit!


(when (boundp 'waypoint_driver::gps_points)
  (if (not (find-package "WAYPOINT_DRIVER"))
    (make-package "WAYPOINT_DRIVER"))
  (shadow 'gps_points (find-package "WAYPOINT_DRIVER")))
(unless (find-package "WAYPOINT_DRIVER::GPS_POINTS")
  (make-package "WAYPOINT_DRIVER::GPS_POINTS"))
(unless (find-package "WAYPOINT_DRIVER::GPS_POINTSREQUEST")
  (make-package "WAYPOINT_DRIVER::GPS_POINTSREQUEST"))
(unless (find-package "WAYPOINT_DRIVER::GPS_POINTSRESPONSE")
  (make-package "WAYPOINT_DRIVER::GPS_POINTSRESPONSE"))

(in-package "ROS")





(defclass waypoint_driver::gps_pointsRequest
  :super ros::object
  :slots (_latitude _longitude ))

(defmethod waypoint_driver::gps_pointsRequest
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass waypoint_driver::gps_pointsResponse
  :super ros::object
  :slots (_reached_status ))

(defmethod waypoint_driver::gps_pointsResponse
  (:init
   (&key
    ((:reached_status __reached_status) nil)
    )
   (send-super :init)
   (setq _reached_status __reached_status)
   self)
  (:reached_status
   (&optional (__reached_status :null))
   (if (not (eq __reached_status :null)) (setq _reached_status __reached_status)) _reached_status)
  (:serialization-length
   ()
   (+
    ;; bool _reached_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _reached_status
       (if _reached_status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _reached_status
     (setq _reached_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass waypoint_driver::gps_points
  :super ros::object
  :slots ())

(setf (get waypoint_driver::gps_points :md5sum-) "b5bec956a12980a00e59b3e48c455bdc")
(setf (get waypoint_driver::gps_points :datatype-) "waypoint_driver/gps_points")
(setf (get waypoint_driver::gps_points :request) waypoint_driver::gps_pointsRequest)
(setf (get waypoint_driver::gps_points :response) waypoint_driver::gps_pointsResponse)

(defmethod waypoint_driver::gps_pointsRequest
  (:response () (instance waypoint_driver::gps_pointsResponse :init)))

(setf (get waypoint_driver::gps_pointsRequest :md5sum-) "b5bec956a12980a00e59b3e48c455bdc")
(setf (get waypoint_driver::gps_pointsRequest :datatype-) "waypoint_driver/gps_pointsRequest")
(setf (get waypoint_driver::gps_pointsRequest :definition-)
      "float64 latitude
float64 longitude
---
bool reached_status
")

(setf (get waypoint_driver::gps_pointsResponse :md5sum-) "b5bec956a12980a00e59b3e48c455bdc")
(setf (get waypoint_driver::gps_pointsResponse :datatype-) "waypoint_driver/gps_pointsResponse")
(setf (get waypoint_driver::gps_pointsResponse :definition-)
      "float64 latitude
float64 longitude
---
bool reached_status
")



(provide :waypoint_driver/gps_points "b5bec956a12980a00e59b3e48c455bdc")


