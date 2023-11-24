
(cl:in-package :asdf)

(defsystem "tomm_hardware_real-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DriverState" :depends-on ("_package_DriverState"))
    (:file "_package_DriverState" :depends-on ("_package"))
    (:file "OmniBaseState" :depends-on ("_package_OmniBaseState"))
    (:file "_package_OmniBaseState" :depends-on ("_package"))
  ))