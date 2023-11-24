;; Auto-generated. Do not edit!


(when (boundp 'ur_script_manager::getScriptManagerStates)
  (if (not (find-package "UR_SCRIPT_MANAGER"))
    (make-package "UR_SCRIPT_MANAGER"))
  (shadow 'getScriptManagerStates (find-package "UR_SCRIPT_MANAGER")))
(unless (find-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATES")
  (make-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATES"))
(unless (find-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATESREQUEST")
  (make-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATESREQUEST"))
(unless (find-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATESRESPONSE")
  (make-package "UR_SCRIPT_MANAGER::GETSCRIPTMANAGERSTATESRESPONSE"))

(in-package "ROS")





(defclass ur_script_manager::getScriptManagerStatesRequest
  :super ros::object
  :slots ())

(defmethod ur_script_manager::getScriptManagerStatesRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ur_script_manager::getScriptManagerStatesResponse
  :super ros::object
  :slots (_names _enabled ))

(defmethod ur_script_manager::getScriptManagerStatesResponse
  (:init
   (&key
    ((:names __names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:enabled __enabled) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _names __names)
   (setq _enabled __enabled)
   self)
  (:names
   (&optional __names)
   (if __names (setq _names __names)) _names)
  (:enabled
   (&optional __enabled)
   (if __enabled (setq _enabled __enabled)) _enabled)
  (:serialization-length
   ()
   (+
    ;; string[] _names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _names)) 4
    ;; bool[] _enabled
    (* 1    (length _enabled)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _names
     (write-long (length _names) s)
     (dolist (elem _names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool[] _enabled
     (write-long (length _enabled) s)
     (dotimes (i (length _enabled))
       (if (elt _enabled i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool[] _enabled
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _enabled (make-list n))
     (dotimes (i n)
     (setf (elt _enabled i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(defclass ur_script_manager::getScriptManagerStates
  :super ros::object
  :slots ())

(setf (get ur_script_manager::getScriptManagerStates :md5sum-) "44bb719088af7b4a0bc3023901924bc8")
(setf (get ur_script_manager::getScriptManagerStates :datatype-) "ur_script_manager/getScriptManagerStates")
(setf (get ur_script_manager::getScriptManagerStates :request) ur_script_manager::getScriptManagerStatesRequest)
(setf (get ur_script_manager::getScriptManagerStates :response) ur_script_manager::getScriptManagerStatesResponse)

(defmethod ur_script_manager::getScriptManagerStatesRequest
  (:response () (instance ur_script_manager::getScriptManagerStatesResponse :init)))

(setf (get ur_script_manager::getScriptManagerStatesRequest :md5sum-) "44bb719088af7b4a0bc3023901924bc8")
(setf (get ur_script_manager::getScriptManagerStatesRequest :datatype-) "ur_script_manager/getScriptManagerStatesRequest")
(setf (get ur_script_manager::getScriptManagerStatesRequest :definition-)
      "---
string[] names		# the names of the script sub programs
bool[] enabled		# the states of the script sub programs

")

(setf (get ur_script_manager::getScriptManagerStatesResponse :md5sum-) "44bb719088af7b4a0bc3023901924bc8")
(setf (get ur_script_manager::getScriptManagerStatesResponse :datatype-) "ur_script_manager/getScriptManagerStatesResponse")
(setf (get ur_script_manager::getScriptManagerStatesResponse :definition-)
      "---
string[] names		# the names of the script sub programs
bool[] enabled		# the states of the script sub programs

")



(provide :ur_script_manager/getScriptManagerStates "44bb719088af7b4a0bc3023901924bc8")


