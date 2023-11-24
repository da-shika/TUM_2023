;; Auto-generated. Do not edit!


(when (boundp 'ur_script_manager::setScriptManagerState)
  (if (not (find-package "UR_SCRIPT_MANAGER"))
    (make-package "UR_SCRIPT_MANAGER"))
  (shadow 'setScriptManagerState (find-package "UR_SCRIPT_MANAGER")))
(unless (find-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATE")
  (make-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATE"))
(unless (find-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATEREQUEST")
  (make-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATEREQUEST"))
(unless (find-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATERESPONSE")
  (make-package "UR_SCRIPT_MANAGER::SETSCRIPTMANAGERSTATERESPONSE"))

(in-package "ROS")





(defclass ur_script_manager::setScriptManagerStateRequest
  :super ros::object
  :slots (_name _enable ))

(defmethod ur_script_manager::setScriptManagerStateRequest
  (:init
   (&key
    ((:name __name) "")
    ((:enable __enable) nil)
    )
   (send-super :init)
   (setq _name (string __name))
   (setq _enable __enable)
   self)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:enable
   (&optional (__enable :null))
   (if (not (eq __enable :null)) (setq _enable __enable)) _enable)
  (:serialization-length
   ()
   (+
    ;; string _name
    4 (length _name)
    ;; bool _enable
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; bool _enable
       (if _enable (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _enable
     (setq _enable (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_script_manager::setScriptManagerStateResponse
  :super ros::object
  :slots (_ok ))

(defmethod ur_script_manager::setScriptManagerStateResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass ur_script_manager::setScriptManagerState
  :super ros::object
  :slots ())

(setf (get ur_script_manager::setScriptManagerState :md5sum-) "f7e852bc0e000ce749738a3d7c423da7")
(setf (get ur_script_manager::setScriptManagerState :datatype-) "ur_script_manager/setScriptManagerState")
(setf (get ur_script_manager::setScriptManagerState :request) ur_script_manager::setScriptManagerStateRequest)
(setf (get ur_script_manager::setScriptManagerState :response) ur_script_manager::setScriptManagerStateResponse)

(defmethod ur_script_manager::setScriptManagerStateRequest
  (:response () (instance ur_script_manager::setScriptManagerStateResponse :init)))

(setf (get ur_script_manager::setScriptManagerStateRequest :md5sum-) "f7e852bc0e000ce749738a3d7c423da7")
(setf (get ur_script_manager::setScriptManagerStateRequest :datatype-) "ur_script_manager/setScriptManagerStateRequest")
(setf (get ur_script_manager::setScriptManagerStateRequest :definition-)
      "string name	# the name of the script sub program 
bool enable	# enable/disable program
---
bool ok		# change of the state was successfull/failed

")

(setf (get ur_script_manager::setScriptManagerStateResponse :md5sum-) "f7e852bc0e000ce749738a3d7c423da7")
(setf (get ur_script_manager::setScriptManagerStateResponse :datatype-) "ur_script_manager/setScriptManagerStateResponse")
(setf (get ur_script_manager::setScriptManagerStateResponse :definition-)
      "string name	# the name of the script sub program 
bool enable	# enable/disable program
---
bool ok		# change of the state was successfull/failed

")



(provide :ur_script_manager/setScriptManagerState "f7e852bc0e000ce749738a3d7c423da7")


