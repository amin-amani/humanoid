;; Auto-generated. Do not edit!


(when (boundp 'surena_usb::active_csp)
  (if (not (find-package "SURENA_USB"))
    (make-package "SURENA_USB"))
  (shadow 'active_csp (find-package "SURENA_USB")))
(unless (find-package "SURENA_USB::ACTIVE_CSP")
  (make-package "SURENA_USB::ACTIVE_CSP"))
(unless (find-package "SURENA_USB::ACTIVE_CSPREQUEST")
  (make-package "SURENA_USB::ACTIVE_CSPREQUEST"))
(unless (find-package "SURENA_USB::ACTIVE_CSPRESPONSE")
  (make-package "SURENA_USB::ACTIVE_CSPRESPONSE"))

(in-package "ROS")





(defclass surena_usb::active_cspRequest
  :super ros::object
  :slots (_nodeID ))

(defmethod surena_usb::active_cspRequest
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
    ;; int16 _nodeID
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _nodeID
       (write-word _nodeID s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _nodeID
     (setq _nodeID (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass surena_usb::active_cspResponse
  :super ros::object
  :slots (_result ))

(defmethod surena_usb::active_cspResponse
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
    ;; int16 _result
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _result
       (write-word _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _result
     (setq _result (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(defclass surena_usb::active_csp
  :super ros::object
  :slots ())

(setf (get surena_usb::active_csp :md5sum-) "9d17c7a086f295f3ddeb8ac344bfc6ca")
(setf (get surena_usb::active_csp :datatype-) "surena_usb/active_csp")
(setf (get surena_usb::active_csp :request) surena_usb::active_cspRequest)
(setf (get surena_usb::active_csp :response) surena_usb::active_cspResponse)

(defmethod surena_usb::active_cspRequest
  (:response () (instance surena_usb::active_cspResponse :init)))

(setf (get surena_usb::active_cspRequest :md5sum-) "9d17c7a086f295f3ddeb8ac344bfc6ca")
(setf (get surena_usb::active_cspRequest :datatype-) "surena_usb/active_cspRequest")
(setf (get surena_usb::active_cspRequest :definition-)
      "int16 nodeID
---
int16 result

")

(setf (get surena_usb::active_cspResponse :md5sum-) "9d17c7a086f295f3ddeb8ac344bfc6ca")
(setf (get surena_usb::active_cspResponse :datatype-) "surena_usb/active_cspResponse")
(setf (get surena_usb::active_cspResponse :definition-)
      "int16 nodeID
---
int16 result

")



(provide :surena_usb/active_csp "9d17c7a086f295f3ddeb8ac344bfc6ca")


