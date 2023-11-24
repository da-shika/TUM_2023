
(cl:in-package :asdf)

(defsystem "ur_script_manager-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getScriptManagerStates" :depends-on ("_package_getScriptManagerStates"))
    (:file "_package_getScriptManagerStates" :depends-on ("_package"))
    (:file "setScriptManagerState" :depends-on ("_package_setScriptManagerState"))
    (:file "_package_setScriptManagerState" :depends-on ("_package"))
  ))