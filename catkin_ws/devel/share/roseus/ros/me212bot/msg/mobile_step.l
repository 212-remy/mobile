;; Auto-generated. Do not edit!


(when (boundp 'me212bot::mobile_step)
  (if (not (find-package "ME212BOT"))
    (make-package "ME212BOT"))
  (shadow 'mobile_step (find-package "ME212BOT")))
(unless (find-package "ME212BOT::MOBILE_STEP")
  (make-package "ME212BOT::MOBILE_STEP"))

(in-package "ROS")
;;//! \htmlinclude mobile_step.msg.html


(defclass me212bot::mobile_step
  :super ros::object
  :slots (_step ))

(defmethod me212bot::mobile_step
  (:init
   (&key
    ((:step __step) 0)
    )
   (send-super :init)
   (setq _step (round __step))
   self)
  (:step
   (&optional __step)
   (if __step (setq _step __step)) _step)
  (:serialization-length
   ()
   (+
    ;; int32 _step
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _step
       (write-long _step s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _step
     (setq _step (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get me212bot::mobile_step :md5sum-) "99174260c0c07917ce2b7a46302ab7a8")
(setf (get me212bot::mobile_step :datatype-) "me212bot/mobile_step")
(setf (get me212bot::mobile_step :definition-)
      "int32 step

")



(provide :me212bot/mobile_step "99174260c0c07917ce2b7a46302ab7a8")


