
(cl:in-package :asdf)

(defsystem "remote_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "set_controller" :depends-on ("_package_set_controller"))
    (:file "_package_set_controller" :depends-on ("_package"))
  ))