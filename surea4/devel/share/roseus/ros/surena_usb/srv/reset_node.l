;; Auto-generated. Do not edit!


(when (boundp 'surena_usb::reset_node)
  (if (not (find-package "SURENA_USB"))
    (make-package "SURENA_USB"))
  (shadow 'reset_node (find-package "SURENA_USB")))
(unless (find-package "SURENA_USB::RESET_NODE")
  (make-package "SURENA_USB::RESET_NODE"))
(unless (find-package "SURENA_USB::RESET_NODEREQUEST")
  (make-package "SURENA_USB::RESET_NODEREQUEST"))
(unless (find-package "SURENA_USB::RESET_NODERESPONSE")
  (make-package "SURENA_USB::RESET_NODERESPONSE"))

(in-package "ROS")





(defclass surena_usb::reset_nodeRequest
  :super ros::object
  :slots (_nodeID ))

(defmethod surena_usb::reset_nodeRequest
  (:init
   (&key
    ((:nodeID __nodeID) 0)
    )
   (send-super :init)
   (setq _nodeID (round __nodeID))
   self)
  (:nodeID
   (&optional __nodeID)
   (if __nodeID (setq _nodeID __nodeID)) _nodeID)
  (:serialization-length
   ()
   (+
    ;; int32 _nodeID
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _nodeID
       (write-long _nodeID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _nodeID
     (setq _nodeID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass surena_usb::reset_nodeResponse
  :super ros::object
  :slots (_result ))

(defmethod surena_usb::reset_nodeResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass surena_usb::reset_node
  :super ros::object
  :slots ())

(setf (get surena_usb::reset_node :md5sum-) "0c471dfb6b81ef71ddb24300538963e0")
(setf (get surena_usb::reset_node :datatype-) "surena_usb/reset_node")
(setf (get surena_usb::reset_node :request) surena_usb::reset_nodeRequest)
(setf (get surena_usb::reset_node :response) surena_usb::reset_nodeResponse)

(defmethod surena_usb::reset_nodeRequest
  (:response () (instance surena_usb::reset_nodeResponse :init)))

(setf (get surena_usb::reset_nodeRequest :md5sum-) "0c471dfb6b81ef71ddb24300538963e0")
(setf (get surena_usb::reset_nodeRequest :datatype-) "surena_usb/reset_nodeRequest")
(setf (get surena_usb::reset_nodeRequest :definition-)
      "int32 nodeID
---
int32 result

")

(setf (get surena_usb::reset_nodeResponse :md5sum-) "0c471dfb6b81ef71ddb24300538963e0")
(setf (get surena_usb::reset_nodeResponse :datatype-) "surena_usb/reset_nodeResponse")
(setf (get surena_usb::reset_nodeResponse :definition-)
      "int32 nodeID
---
int32 result

")



(provide :surena_usb/reset_node "0c471dfb6b81ef71ddb24300538963e0")


