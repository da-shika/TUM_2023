
(cl:in-package :asdf)

(defsystem "behavior_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ChangeBehavior" :depends-on ("_package_ChangeBehavior"))
    (:file "_package_ChangeBehavior" :depends-on ("_package"))
    (:file "ChangeContact" :depends-on ("_package_ChangeContact"))
    (:file "_package_ChangeContact" :depends-on ("_package"))
    (:file "ListBehavior" :depends-on ("_package_ListBehavior"))
    (:file "_package_ListBehavior" :depends-on ("_package"))
  ))