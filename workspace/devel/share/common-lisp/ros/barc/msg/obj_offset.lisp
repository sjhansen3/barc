; Auto-generated. Do not edit!


(cl:in-package barc-msg)


;//! \htmlinclude obj_offset.msg.html

(cl:defclass <obj_offset> (roslisp-msg-protocol:ros-message)
  ((pixel_center_offset
    :reader pixel_center_offset
    :initarg :pixel_center_offset
    :type cl:float
    :initform 0.0)
   (est_dist
    :reader est_dist
    :initarg :est_dist
    :type cl:float
    :initform 0.0)
   (est_center_offset
    :reader est_center_offset
    :initarg :est_center_offset
    :type cl:float
    :initform 0.0))
)

(cl:defclass obj_offset (<obj_offset>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obj_offset>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obj_offset)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name barc-msg:<obj_offset> is deprecated: use barc-msg:obj_offset instead.")))

(cl:ensure-generic-function 'pixel_center_offset-val :lambda-list '(m))
(cl:defmethod pixel_center_offset-val ((m <obj_offset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:pixel_center_offset-val is deprecated.  Use barc-msg:pixel_center_offset instead.")
  (pixel_center_offset m))

(cl:ensure-generic-function 'est_dist-val :lambda-list '(m))
(cl:defmethod est_dist-val ((m <obj_offset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:est_dist-val is deprecated.  Use barc-msg:est_dist instead.")
  (est_dist m))

(cl:ensure-generic-function 'est_center_offset-val :lambda-list '(m))
(cl:defmethod est_center_offset-val ((m <obj_offset>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader barc-msg:est_center_offset-val is deprecated.  Use barc-msg:est_center_offset instead.")
  (est_center_offset m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obj_offset>) ostream)
  "Serializes a message object of type '<obj_offset>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pixel_center_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'est_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'est_center_offset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obj_offset>) istream)
  "Deserializes a message object of type '<obj_offset>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pixel_center_offset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'est_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'est_center_offset) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obj_offset>)))
  "Returns string type for a message object of type '<obj_offset>"
  "barc/obj_offset")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obj_offset)))
  "Returns string type for a message object of type 'obj_offset"
  "barc/obj_offset")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obj_offset>)))
  "Returns md5sum for a message object of type '<obj_offset>"
  "906dd9894d5ba5a23af5c11934a5eac3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obj_offset)))
  "Returns md5sum for a message object of type 'obj_offset"
  "906dd9894d5ba5a23af5c11934a5eac3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obj_offset>)))
  "Returns full string definition for message of type '<obj_offset>"
  (cl:format cl:nil "float32 pixel_center_offset~%float32 est_dist~%float32 est_center_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obj_offset)))
  "Returns full string definition for message of type 'obj_offset"
  (cl:format cl:nil "float32 pixel_center_offset~%float32 est_dist~%float32 est_center_offset~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obj_offset>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obj_offset>))
  "Converts a ROS message object to a list"
  (cl:list 'obj_offset
    (cl:cons ':pixel_center_offset (pixel_center_offset msg))
    (cl:cons ':est_dist (est_dist msg))
    (cl:cons ':est_center_offset (est_center_offset msg))
))
