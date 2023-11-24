
(cl:in-package :asdf)

(defsystem "skin_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SkinPatchData" :depends-on ("_package_SkinPatchData"))
    (:file "_package_SkinPatchData" :depends-on ("_package"))
  ))