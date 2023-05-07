;; Auto-generated. Do not edit!


(when (boundp 'global_controller::controller_states)
  (if (not (find-package "GLOBAL_CONTROLLER"))
    (make-package "GLOBAL_CONTROLLER"))
  (shadow 'controller_states (find-package "GLOBAL_CONTROLLER")))
(unless (find-package "GLOBAL_CONTROLLER::CONTROLLER_STATES")
  (make-package "GLOBAL_CONTROLLER::CONTROLLER_STATES"))

(in-package "ROS")
;;//! \htmlinclude controller_states.msg.html


(defclass global_controller::controller_states
  :super ros::object
  :slots (_is_driving_automatically _is_allowed_to_drive _tiz_is_boosted ))

(defmethod global_controller::controller_states
  (:init
   (&key
    ((:is_driving_automatically __is_driving_automatically) nil)
    ((:is_allowed_to_drive __is_allowed_to_drive) nil)
    ((:tiz_is_boosted __tiz_is_boosted) nil)
    )
   (send-super :init)
   (setq _is_driving_automatically __is_driving_automatically)
   (setq _is_allowed_to_drive __is_allowed_to_drive)
   (setq _tiz_is_boosted __tiz_is_boosted)
   self)
  (:is_driving_automatically
   (&optional (__is_driving_automatically :null))
   (if (not (eq __is_driving_automatically :null)) (setq _is_driving_automatically __is_driving_automatically)) _is_driving_automatically)
  (:is_allowed_to_drive
   (&optional (__is_allowed_to_drive :null))
   (if (not (eq __is_allowed_to_drive :null)) (setq _is_allowed_to_drive __is_allowed_to_drive)) _is_allowed_to_drive)
  (:tiz_is_boosted
   (&optional (__tiz_is_boosted :null))
   (if (not (eq __tiz_is_boosted :null)) (setq _tiz_is_boosted __tiz_is_boosted)) _tiz_is_boosted)
  (:serialization-length
   ()
   (+
    ;; bool _is_driving_automatically
    1
    ;; bool _is_allowed_to_drive
    1
    ;; bool _tiz_is_boosted
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_driving_automatically
       (if _is_driving_automatically (write-byte -1 s) (write-byte 0 s))
     ;; bool _is_allowed_to_drive
       (if _is_allowed_to_drive (write-byte -1 s) (write-byte 0 s))
     ;; bool _tiz_is_boosted
       (if _tiz_is_boosted (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_driving_automatically
     (setq _is_driving_automatically (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _is_allowed_to_drive
     (setq _is_allowed_to_drive (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _tiz_is_boosted
     (setq _tiz_is_boosted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get global_controller::controller_states :md5sum-) "656034952bcd2a7d6b51db3480a3adeb")
(setf (get global_controller::controller_states :datatype-) "global_controller/controller_states")
(setf (get global_controller::controller_states :definition-)
      "bool is_driving_automatically
bool is_allowed_to_drive
bool tiz_is_boosted

")



(provide :global_controller/controller_states "656034952bcd2a7d6b51db3480a3adeb")


