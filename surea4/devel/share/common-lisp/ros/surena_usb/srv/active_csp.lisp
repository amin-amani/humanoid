; Auto-generated. Do not edit!


(cl:in-package surena_usb-srv)


;//! \htmlinclude active_csp-request.msg.html

(cl:defclass <active_csp-request> (roslisp-msg-protocol:ros-message)
  ((nodeID
    :reader nodeID
    :initarg :nodeID
    :type cl:fixnum
    :initform 0))
)

(cl:defclass active_csp-request (<active_csp-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <active_csp-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'active_csp-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surena_usb-srv:<active_csp-request> is deprecated: use surena_usb-srv:active_csp-request instead.")))

(cl:ensure-generic-function 'nodeID-val :lambda-list '(m))
(cl:defmethod nodeID-val ((m <active_csp-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surena_usb-srv:nodeID-val is deprecated.  Use surena_usb-srv:nodeID instead.")
  (nodeID m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <active_csp-request>) ostream)
  "Serializes a message object of type '<active_csp-request>"
  (cl:let* ((signed (cl:slot-value msg 'nodeID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <active_csp-request>) istream)
  "Deserializes a message object of type '<active_csp-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'nodeID) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<active_csp-request>)))
  "Returns string type for a service object of type '<active_csp-request>"
  "surena_usb/active_cspRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'active_csp-request)))
  "Returns string type for a service object of type 'active_csp-request"
  "surena_usb/active_cspRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<active_csp-request>)))
  "Returns md5sum for a message object of type '<active_csp-request>"
  "9d17c7a086f295f3ddeb8ac344bfc6ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'active_csp-request)))
  "Returns md5sum for a message object of type 'active_csp-request"
  "9d17c7a086f295f3ddeb8ac344bfc6ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<active_csp-request>)))
  "Returns full string definition for message of type '<active_csp-request>"
  (cl:format cl:nil "int16 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'active_csp-request)))
  "Returns full string definition for message of type 'active_csp-request"
  (cl:format cl:nil "int16 nodeID~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <active_csp-request>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <active_csp-request>))
  "Converts a ROS message object to a list"
  (cl:list 'active_csp-request
    (cl:cons ':nodeID (nodeID msg))
))
;//! \htmlinclude active_csp-response.msg.html

(cl:defclass <active_csp-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:fixnum
    :initform 0))
)

(cl:defclass active_csp-response (<active_csp-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <active_csp-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'active_csp-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name surena_usb-srv:<active_csp-response> is deprecated: use surena_usb-srv:active_csp-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <active_csp-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader surena_usb-srv:result-val is deprecated.  Use surena_usb-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <active_csp-response>) ostream)
  "Serializes a message object of type '<active_csp-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <active_csp-response>) istream)
  "Deserializes a message object of type '<active_csp-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<active_csp-response>)))
  "Returns string type for a service object of type '<active_csp-response>"
  "surena_usb/active_cspResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'active_csp-response)))
  "Returns string type for a service object of type 'active_csp-response"
  "surena_usb/active_cspResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<active_csp-response>)))
  "Returns md5sum for a message object of type '<active_csp-response>"
  "9d17c7a086f295f3ddeb8ac344bfc6ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'active_csp-response)))
  "Returns md5sum for a message object of type 'active_csp-response"
  "9d17c7a086f295f3ddeb8ac344bfc6ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<active_csp-response>)))
  "Returns full string definition for message of type '<active_csp-response>"
  (cl:format cl:nil "int16 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'active_csp-response)))
  "Returns full string definition for message of type 'active_csp-response"
  (cl:format cl:nil "int16 result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <active_csp-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <active_csp-response>))
  "Converts a ROS message object to a list"
  (cl:list 'active_csp-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'active_csp)))
  'active_csp-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'active_csp)))
  'active_csp-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'active_csp)))
  "Returns string type for a service object of type '<active_csp>"
  "surena_usb/active_csp")