; Auto-generated. Do not edit!


(cl:in-package project_231-msg)


;//! \htmlinclude State.msg.html

(cl:defclass <State> (roslisp-msg-protocol:ros-message)
  ((car_vel
    :reader car_vel
    :initarg :car_vel
    :type cl:float
    :initform 0.0)
   (car_accl
    :reader car_accl
    :initarg :car_accl
    :type cl:float
    :initform 0.0)
   (us_dist
    :reader us_dist
    :initarg :us_dist
    :type cl:float
    :initform 0.0)
   (us_rate
    :reader us_rate
    :initarg :us_rate
    :type cl:float
    :initform 0.0)
   (us_accl
    :reader us_accl
    :initarg :us_accl
    :type cl:float
    :initform 0.0)
   (obj_psi
    :reader obj_psi
    :initarg :obj_psi
    :type cl:float
    :initform 0.0))
)

(cl:defclass State (<State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name project_231-msg:<State> is deprecated: use project_231-msg:State instead.")))

(cl:ensure-generic-function 'car_vel-val :lambda-list '(m))
(cl:defmethod car_vel-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:car_vel-val is deprecated.  Use project_231-msg:car_vel instead.")
  (car_vel m))

(cl:ensure-generic-function 'car_accl-val :lambda-list '(m))
(cl:defmethod car_accl-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:car_accl-val is deprecated.  Use project_231-msg:car_accl instead.")
  (car_accl m))

(cl:ensure-generic-function 'us_dist-val :lambda-list '(m))
(cl:defmethod us_dist-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:us_dist-val is deprecated.  Use project_231-msg:us_dist instead.")
  (us_dist m))

(cl:ensure-generic-function 'us_rate-val :lambda-list '(m))
(cl:defmethod us_rate-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:us_rate-val is deprecated.  Use project_231-msg:us_rate instead.")
  (us_rate m))

(cl:ensure-generic-function 'us_accl-val :lambda-list '(m))
(cl:defmethod us_accl-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:us_accl-val is deprecated.  Use project_231-msg:us_accl instead.")
  (us_accl m))

(cl:ensure-generic-function 'obj_psi-val :lambda-list '(m))
(cl:defmethod obj_psi-val ((m <State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader project_231-msg:obj_psi-val is deprecated.  Use project_231-msg:obj_psi instead.")
  (obj_psi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <State>) ostream)
  "Serializes a message object of type '<State>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car_accl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'us_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'us_rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'us_accl))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'obj_psi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <State>) istream)
  "Deserializes a message object of type '<State>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car_accl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'us_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'us_rate) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'us_accl) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'obj_psi) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<State>)))
  "Returns string type for a message object of type '<State>"
  "project_231/State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'State)))
  "Returns string type for a message object of type 'State"
  "project_231/State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<State>)))
  "Returns md5sum for a message object of type '<State>"
  "99ffbc9b2ce9f309bebfb0d24dae7716")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'State)))
  "Returns md5sum for a message object of type 'State"
  "99ffbc9b2ce9f309bebfb0d24dae7716")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<State>)))
  "Returns full string definition for message of type '<State>"
  (cl:format cl:nil "float32 car_vel~%float32 car_accl~%float32 us_dist~%float32 us_rate~%float32 us_accl~%float32 obj_psi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'State)))
  "Returns full string definition for message of type 'State"
  (cl:format cl:nil "float32 car_vel~%float32 car_accl~%float32 us_dist~%float32 us_rate~%float32 us_accl~%float32 obj_psi~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <State>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <State>))
  "Converts a ROS message object to a list"
  (cl:list 'State
    (cl:cons ':car_vel (car_vel msg))
    (cl:cons ':car_accl (car_accl msg))
    (cl:cons ':us_dist (us_dist msg))
    (cl:cons ':us_rate (us_rate msg))
    (cl:cons ':us_accl (us_accl msg))
    (cl:cons ':obj_psi (obj_psi msg))
))
