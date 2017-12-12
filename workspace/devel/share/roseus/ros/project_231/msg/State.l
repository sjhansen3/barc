;; Auto-generated. Do not edit!


(when (boundp 'project_231::State)
  (if (not (find-package "PROJECT_231"))
    (make-package "PROJECT_231"))
  (shadow 'State (find-package "PROJECT_231")))
(unless (find-package "PROJECT_231::STATE")
  (make-package "PROJECT_231::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html


(defclass project_231::State
  :super ros::object
  :slots (_car_vel _car_accl _us_dist _us_rate _us_accl _obj_psi ))

(defmethod project_231::State
  (:init
   (&key
    ((:car_vel __car_vel) 0.0)
    ((:car_accl __car_accl) 0.0)
    ((:us_dist __us_dist) 0.0)
    ((:us_rate __us_rate) 0.0)
    ((:us_accl __us_accl) 0.0)
    ((:obj_psi __obj_psi) 0.0)
    )
   (send-super :init)
   (setq _car_vel (float __car_vel))
   (setq _car_accl (float __car_accl))
   (setq _us_dist (float __us_dist))
   (setq _us_rate (float __us_rate))
   (setq _us_accl (float __us_accl))
   (setq _obj_psi (float __obj_psi))
   self)
  (:car_vel
   (&optional __car_vel)
   (if __car_vel (setq _car_vel __car_vel)) _car_vel)
  (:car_accl
   (&optional __car_accl)
   (if __car_accl (setq _car_accl __car_accl)) _car_accl)
  (:us_dist
   (&optional __us_dist)
   (if __us_dist (setq _us_dist __us_dist)) _us_dist)
  (:us_rate
   (&optional __us_rate)
   (if __us_rate (setq _us_rate __us_rate)) _us_rate)
  (:us_accl
   (&optional __us_accl)
   (if __us_accl (setq _us_accl __us_accl)) _us_accl)
  (:obj_psi
   (&optional __obj_psi)
   (if __obj_psi (setq _obj_psi __obj_psi)) _obj_psi)
  (:serialization-length
   ()
   (+
    ;; float32 _car_vel
    4
    ;; float32 _car_accl
    4
    ;; float32 _us_dist
    4
    ;; float32 _us_rate
    4
    ;; float32 _us_accl
    4
    ;; float32 _obj_psi
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _car_vel
       (sys::poke _car_vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _car_accl
       (sys::poke _car_accl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _us_dist
       (sys::poke _us_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _us_rate
       (sys::poke _us_rate (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _us_accl
       (sys::poke _us_accl (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _obj_psi
       (sys::poke _obj_psi (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _car_vel
     (setq _car_vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _car_accl
     (setq _car_accl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _us_dist
     (setq _us_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _us_rate
     (setq _us_rate (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _us_accl
     (setq _us_accl (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _obj_psi
     (setq _obj_psi (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get project_231::State :md5sum-) "99ffbc9b2ce9f309bebfb0d24dae7716")
(setf (get project_231::State :datatype-) "project_231/State")
(setf (get project_231::State :definition-)
      "float32 car_vel
float32 car_accl
float32 us_dist
float32 us_rate
float32 us_accl
float32 obj_psi

")



(provide :project_231/State "99ffbc9b2ce9f309bebfb0d24dae7716")


