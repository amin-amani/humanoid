
(cl:in-package :asdf)

(defsystem "surena_usb-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "active_csp" :depends-on ("_package_active_csp"))
    (:file "_package_active_csp" :depends-on ("_package"))
    (:file "reset_node" :depends-on ("_package_reset_node"))
    (:file "_package_reset_node" :depends-on ("_package"))
  ))