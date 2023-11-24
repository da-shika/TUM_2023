
(cl:in-package :asdf)

(defsystem "walking_core_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :control_core_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FootStep" :depends-on ("_package_FootStep"))
    (:file "_package_FootStep" :depends-on ("_package"))
    (:file "FootSteps" :depends-on ("_package_FootSteps"))
    (:file "_package_FootSteps" :depends-on ("_package"))
    (:file "WalkingPhase" :depends-on ("_package_WalkingPhase"))
    (:file "_package_WalkingPhase" :depends-on ("_package"))
    (:file "WalkingStates" :depends-on ("_package_WalkingStates"))
    (:file "_package_WalkingStates" :depends-on ("_package"))
  ))