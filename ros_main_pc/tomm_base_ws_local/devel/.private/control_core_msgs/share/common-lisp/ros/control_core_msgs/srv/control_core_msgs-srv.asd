
(cl:in-package :asdf)

(defsystem "control_core_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :control_core_msgs-msg
               :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "ComputeControl" :depends-on ("_package_ComputeControl"))
    (:file "_package_ComputeControl" :depends-on ("_package"))
    (:file "SetCartesianPositionGoal" :depends-on ("_package_SetCartesianPositionGoal"))
    (:file "_package_SetCartesianPositionGoal" :depends-on ("_package"))
    (:file "SetCartesianStateGoal" :depends-on ("_package_SetCartesianStateGoal"))
    (:file "_package_SetCartesianStateGoal" :depends-on ("_package"))
    (:file "StartControl" :depends-on ("_package_StartControl"))
    (:file "_package_StartControl" :depends-on ("_package"))
  ))