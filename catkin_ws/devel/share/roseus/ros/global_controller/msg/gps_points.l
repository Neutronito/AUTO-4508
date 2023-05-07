;; Auto-generated. Do not edit!


(when (boundp 'global_controller::gps_points)
  (if (not (find-package "GLOBAL_CONTROLLER"))
    (make-package "GLOBAL_CONTROLLER"))
  (shadow 'gps_points (find-package "GLOBAL_CONTROLLER")))
(unless (find-package "GLOBAL_CONTROLLER::GPS_POINTS")
  (make-package "GLOBAL_CONTROLLER::GPS_POINTS"))

(in-package "ROS")
;;//! \htmlinclude gps_points.msg.html


(defclass global_controller::gps_points
  :super ros::object
  :slots (_latitude _longitude _reached_status ))

(defmethod global_controller::gps_points
  (:init
   (&key
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:reached_status __reached_status) nil)
    )
   (send-super :init)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _reached_status __reached_status)
   self)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:reached_status
   (&optional (__reached_status :null))
   (if (not (eq __reached_status :null)) (setq _reached_status __reached_status)) _reached_status)
  (:serialization-length
   ()
   (+
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; bool _reached_status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _reached_status
       (if _reached_status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _reached_status
     (setq _reached_status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get global_controller::gps_points :md5sum-) "560569517c52fdcf12619013f5dcbac6")
(setf (get global_controller::gps_points :datatype-) "global_controller/gps_points")
(setf (get global_controller::gps_points :definition-)
      "float64 latitude
float64 longitude
bool reached_status
")



(provide :global_controller/gps_points "560569517c52fdcf12619013f5dcbac6")


