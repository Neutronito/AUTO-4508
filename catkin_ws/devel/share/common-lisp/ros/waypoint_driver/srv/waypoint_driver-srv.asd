
(cl:in-package :asdf)

(defsystem "waypoint_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "gps_points" :depends-on ("_package_gps_points"))
    (:file "_package_gps_points" :depends-on ("_package"))
  ))