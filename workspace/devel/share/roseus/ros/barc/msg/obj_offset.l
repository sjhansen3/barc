;; Auto-generated. Do not edit!


(when (boundp 'barc::obj_offset)
  (if (not (find-package "BARC"))
    (make-package "BARC"))
  (shadow 'obj_offset (find-package "BARC")))
(unless (find-package "BARC::OBJ_OFFSET")
  (make-package "BARC::OBJ_OFFSET"))

(in-package "ROS")
;;//! \htmlinclude obj_offset.msg.html


(defclass barc::obj_offset
  :super ros::object
  :slots (_pixel_center_offset _est_dist _est_center_offset ))

(defmethod barc::obj_offset
  (:init
   (&key
    ((:pixel_center_offset __pixel_center_offset) 0.0)
    ((:est_dist __est_dist) 0.0)
    ((:est_center_offset __est_center_offset) 0.0)
    )
   (send-super :init)
   (setq _pixel_center_offset (float __pixel_center_offset))
   (setq _est_dist (float __est_dist))
   (setq _est_center_offset (float __est_center_offset))
   self)
  (:pixel_center_offset
   (&optional __pixel_center_offset)
   (if __pixel_center_offset (setq _pixel_center_offset __pixel_center_offset)) _pixel_center_offset)
  (:est_dist
   (&optional __est_dist)
   (if __est_dist (setq _est_dist __est_dist)) _est_dist)
  (:est_center_offset
   (&optional __est_center_offset)
   (if __est_center_offset (setq _est_center_offset __est_center_offset)) _est_center_offset)
  (:serialization-length
   ()
   (+
    ;; float32 _pixel_center_offset
    4
    ;; float32 _est_dist
    4
    ;; float32 _est_center_offset
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _pixel_center_offset
       (sys::poke _pixel_center_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_dist
       (sys::poke _est_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _est_center_offset
       (sys::poke _est_center_offset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _pixel_center_offset
     (setq _pixel_center_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_dist
     (setq _est_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _est_center_offset
     (setq _est_center_offset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get barc::obj_offset :md5sum-) "906dd9894d5ba5a23af5c11934a5eac3")
(setf (get barc::obj_offset :datatype-) "barc/obj_offset")
(setf (get barc::obj_offset :definition-)
      "float32 pixel_center_offset
float32 est_dist
float32 est_center_offset

")



(provide :barc/obj_offset "906dd9894d5ba5a23af5c11934a5eac3")


