;; Auto-generated. Do not edit!


(when (boundp 'marker_publisher::Marker)
  (if (not (find-package "MARKER_PUBLISHER"))
    (make-package "MARKER_PUBLISHER"))
  (shadow 'Marker (find-package "MARKER_PUBLISHER")))
(unless (find-package "MARKER_PUBLISHER::MARKER")
  (make-package "MARKER_PUBLISHER::MARKER"))

(in-package "ROS")
;;//! \htmlinclude Marker.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass marker_publisher::Marker
  :super ros::object
  :slots (_idx _pose ))

(defmethod marker_publisher::Marker
  (:init
   (&key
    ((:idx __idx) 0)
    ((:pose __pose) (instance geometry_msgs::PoseWithCovariance :init))
    )
   (send-super :init)
   (setq _idx (round __idx))
   (setq _pose __pose)
   self)
  (:idx
   (&optional __idx)
   (if __idx (setq _idx __idx)) _idx)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; uint32 _idx
    4
    ;; geometry_msgs/PoseWithCovariance _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _idx
       (write-long _idx s)
     ;; geometry_msgs/PoseWithCovariance _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _idx
     (setq _idx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; geometry_msgs/PoseWithCovariance _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get marker_publisher::Marker :md5sum-) "dbd1a350bd65d90cf1c2af9ba7d01fe6")
(setf (get marker_publisher::Marker :datatype-) "marker_publisher/Marker")
(setf (get marker_publisher::Marker :definition-)
      "uint32 idx
geometry_msgs/PoseWithCovariance pose

================================================================================
MSG: geometry_msgs/PoseWithCovariance
# This represents a pose in free space with uncertainty.

Pose pose

# Row-major representation of the 6x6 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
float64[36] covariance

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :marker_publisher/Marker "dbd1a350bd65d90cf1c2af9ba7d01fe6")


