
(cl:in-package :asdf)

(defsystem "walking_core_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanFootsteps" :depends-on ("_package_PlanFootsteps"))
    (:file "_package_PlanFootsteps" :depends-on ("_package"))
  ))