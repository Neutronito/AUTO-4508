
(cl:in-package :asdf)

(defsystem "global_controller-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "controller_states" :depends-on ("_package_controller_states"))
    (:file "_package_controller_states" :depends-on ("_package"))
  ))