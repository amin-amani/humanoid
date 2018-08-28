;; Auto-generated. Do not edit!


(when (boundp 'offlinepublisher::num)
  (if (not (find-package "OFFLINEPUBLISHER"))
    (make-package "OFFLINEPUBLISHER"))
  (shadow 'num (find-package "OFFLINEPUBLISHER")))
(unless (find-package "OFFLINEPUBLISHER::NUM")
  (make-package "OFFLINEPUBLISHER::NUM"))

(in-package "ROS")
;;//! \htmlinclude num.msg.html


(defclass offlinepublisher::num
  :super ros::object
  :slots (_first_name _last_name _age _score ))

(defmethod offlinepublisher::num
  (:init
   (&key
    ((:first_name __first_name) "")
    ((:last_name __last_name) "")
    ((:age __age) 0)
    ((:score __score) 0)
    )
   (send-super :init)
   (setq _first_name (string __first_name))
   (setq _last_name (string __last_name))
   (setq _age (round __age))
   (setq _score (round __score))
   self)
  (:first_name
   (&optional __first_name)
   (if __first_name (setq _first_name __first_name)) _first_name)
  (:last_name
   (&optional __last_name)
   (if __last_name (setq _last_name __last_name)) _last_name)
  (:age
   (&optional __age)
   (if __age (setq _age __age)) _age)
  (:score
   (&optional __score)
   (if __score (setq _score __score)) _score)
  (:serialization-length
   ()
   (+
    ;; string _first_name
    4 (length _first_name)
    ;; string _last_name
    4 (length _last_name)
    ;; uint8 _age
    1
    ;; uint32 _score
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _first_name
       (write-long (length _first_name) s) (princ _first_name s)
     ;; string _last_name
       (write-long (length _last_name) s) (princ _last_name s)
     ;; uint8 _age
       (write-byte _age s)
     ;; uint32 _score
       (write-long _score s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _first_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _first_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _last_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _last_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint8 _age
     (setq _age (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint32 _score
     (setq _score (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get offlinepublisher::num :md5sum-) "f8bfa80ae3c7a93455596d9622ad33a9")
(setf (get offlinepublisher::num :datatype-) "offlinepublisher/num")
(setf (get offlinepublisher::num :definition-)
      "string first_name
string last_name
uint8 age
uint32 score

")



(provide :offlinepublisher/num "f8bfa80ae3c7a93455596d9622ad33a9")


