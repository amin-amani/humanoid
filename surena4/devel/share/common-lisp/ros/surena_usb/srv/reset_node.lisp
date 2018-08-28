; Auto-generated. Do not edit!


(cl:in-package surena_usb-srv)


;//! \htmlinclude reset_node-request.msg.html

(cl:defclass <reset_node-request> (roslisp-msg-protocol:ros-message)
  ((nodeID
    :reader nodeID
    :initarg :nodeID
    :type cl:integer
    :initform 0))
)

(cl:defclass reset_node-request (<reset_node-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_node-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_node-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surena_usb-srv:<reset_node-request> is deprecated: use surena_usb-srv:reset_node-request instead.")))

(cl:ensure-generic-function 'nodeID-val :lambda-list '(m))
(cl:defmethod nodeID-val ((m <reset_node-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surena_usb-srv:nodeID-val is deprecated.  Use surena_usb-srv:nodeID instead.")
  (nodeID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_node-request>) ostream)
  "Serializes a message object of type '<reset_node-request>"
  (cl:let* ((signed (cl:slot-value msg 'nodeID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_node-request>) istream)
  "Deserializes a message object of type '<reset_node-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_node-request>)))
  "Returns string type for a service object of type '<reset_node-request>"
  "surena_usb/reset_nodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_node-request)))
  "Returns string type for a service object of type 'reset_node-request"
  "surena_usb/reset_nodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_node-request>)))
  "Returns md5sum for a message object of type '<reset_node-request>"
  "0c471dfb6b81ef71ddb24300538963e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_node-request)))
  "Returns md5sum for a message object of type 'reset_node-request"
  "0c471dfb6b81ef71ddb24300538963e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_node-request>)))
  "Returns full string definition for message of type '<reset_node-request>"
  (cl:format cl:nil "int32 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_node-request)))
  "Returns full string definition for message of type 'reset_node-request"
  (cl:format cl:nil "int32 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_node-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_node-request>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_node-request
    (cl:cons ':nodeID (nodeID msg))
))
;//! \htmlinclude reset_node-response.msg.html

(cl:defclass <reset_node-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass reset_node-response (<reset_node-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <reset_node-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'reset_node-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surena_usb-srv:<reset_node-response> is deprecated: use surena_usb-srv:reset_node-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <reset_node-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surena_usb-srv:result-val is deprecated.  Use surena_usb-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <reset_node-response>) ostream)
  "Serializes a message object of type '<reset_node-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <reset_node-response>) istream)
  "Deserializes a message object of type '<reset_node-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<reset_node-response>)))
  "Returns string type for a service object of type '<reset_node-response>"
  "surena_usb/reset_nodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_node-response)))
  "Returns string type for a service object of type 'reset_node-response"
  "surena_usb/reset_nodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<reset_node-response>)))
  "Returns md5sum for a message object of type '<reset_node-response>"
  "0c471dfb6b81ef71ddb24300538963e0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'reset_node-response)))
  "Returns md5sum for a message object of type 'reset_node-response"
  "0c471dfb6b81ef71ddb24300538963e0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<reset_node-response>)))
  "Returns full string definition for message of type '<reset_node-response>"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'reset_node-response)))
  "Returns full string definition for message of type 'reset_node-response"
  (cl:format cl:nil "int32 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <reset_node-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <reset_node-response>))
  "Converts a ROS message object to a list"
  (cl:list 'reset_node-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'reset_node)))
  'reset_node-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'reset_node)))
  'reset_node-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'reset_node)))
  "Returns string type for a service object of type '<reset_node>"
  "surena_usb/reset_node")