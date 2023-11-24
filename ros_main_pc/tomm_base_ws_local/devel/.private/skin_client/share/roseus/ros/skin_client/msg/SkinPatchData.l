;; Auto-generated. Do not edit!


(when (boundp 'skin_client::SkinPatchData)
  (if (not (find-package "SKIN_CLIENT"))
    (make-package "SKIN_CLIENT"))
  (shadow 'SkinPatchData (find-package "SKIN_CLIENT")))
(unless (find-package "SKIN_CLIENT::SKINPATCHDATA")
  (make-package "SKIN_CLIENT::SKINPATCHDATA"))

(in-package "ROS")
;;//! \htmlinclude SkinPatchData.msg.html


(defclass skin_client::SkinPatchData
  :super ros::object
  :slots (_prox _force _dist ))

(defmethod skin_client::SkinPatchData
  (:init
   (&key
    ((:prox __prox) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:force __force) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:dist __dist) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _prox __prox)
   (setq _force __force)
   (setq _dist __dist)
   self)
  (:prox
   (&optional __prox)
   (if __prox (setq _prox __prox)) _prox)
  (:force
   (&optional __force)
   (if __force (setq _force __force)) _force)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:serialization-length
   ()
   (+
    ;; float64[] _prox
    (* 8    (length _prox)) 4
    ;; float64[] _force
    (* 8    (length _force)) 4
    ;; float64[] _dist
    (* 8    (length _dist)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _prox
     (write-long (length _prox) s)
     (dotimes (i (length _prox))
       (sys::poke (elt _prox i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _force
     (write-long (length _force) s)
     (dotimes (i (length _force))
       (sys::poke (elt _force i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _dist
     (write-long (length _dist) s)
     (dotimes (i (length _dist))
       (sys::poke (elt _dist i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _prox
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _prox (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _prox i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _force
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _force (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _force i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _dist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dist i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get skin_client::SkinPatchData :md5sum-) "179ab063081e8357cb8a2a1174181125")
(setf (get skin_client::SkinPatchData :datatype-) "skin_client/SkinPatchData")
(setf (get skin_client::SkinPatchData :definition-)
      "float64[] prox
float64[] force
float64[] dist
")



(provide :skin_client/SkinPatchData "179ab063081e8357cb8a2a1174181125")


