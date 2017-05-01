; Auto-generated. Do not edit!


(cl:in-package pal_detection_msgs-msg)


;//! \htmlinclude PersonDetections.msg.html

(cl:defclass <PersonDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (persons
    :reader persons
    :initarg :persons
    :type (cl:vector pal_detection_msgs-msg:PersonDetection)
   :initform (cl:make-array 0 :element-type 'pal_detection_msgs-msg:PersonDetection :initial-element (cl:make-instance 'pal_detection_msgs-msg:PersonDetection)))
   (camera_pose
    :reader camera_pose
    :initarg :camera_pose
    :type geometry_msgs-msg:TransformStamped
    :initform (cl:make-instance 'geometry_msgs-msg:TransformStamped)))
)

(cl:defclass PersonDetections (<PersonDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PersonDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PersonDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_detection_msgs-msg:<PersonDetections> is deprecated: use pal_detection_msgs-msg:PersonDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <PersonDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_detection_msgs-msg:header-val is deprecated.  Use pal_detection_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'persons-val :lambda-list '(m))
(cl:defmethod persons-val ((m <PersonDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_detection_msgs-msg:persons-val is deprecated.  Use pal_detection_msgs-msg:persons instead.")
  (persons m))

(cl:ensure-generic-function 'camera_pose-val :lambda-list '(m))
(cl:defmethod camera_pose-val ((m <PersonDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader pal_detection_msgs-msg:camera_pose-val is deprecated.  Use pal_detection_msgs-msg:camera_pose instead.")
  (camera_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PersonDetections>) ostream)
  "Serializes a message object of type '<PersonDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'persons))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'persons))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PersonDetections>) istream)
  "Deserializes a message object of type '<PersonDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'persons) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'persons)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'pal_detection_msgs-msg:PersonDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PersonDetections>)))
  "Returns string type for a message object of type '<PersonDetections>"
  "pal_detection_msgs/PersonDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PersonDetections)))
  "Returns string type for a message object of type 'PersonDetections"
  "pal_detection_msgs/PersonDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PersonDetections>)))
  "Returns md5sum for a message object of type '<PersonDetections>"
  "2e9459d911994b1f8ae4e54431f0f631")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PersonDetections)))
  "Returns md5sum for a message object of type 'PersonDetections"
  "2e9459d911994b1f8ae4e54431f0f631")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PersonDetections>)))
  "Returns full string definition for message of type '<PersonDetections>"
  (cl:format cl:nil "Header header~%~%pal_detection_msgs/PersonDetection[] persons~%~%# Optional transformation between the camera frame and a certain parent frame~%geometry_msgs/TransformStamped camera_pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pal_detection_msgs/PersonDetection~%## Contains detection correspondences among the different person detectors available~%~%# full body image detection. If width == height == 0 then no full body detection is available~%pal_detection_msgs/Detection2d full_body~%~%# face image detection. If width == height == 0 then the face has not been detected~%pal_detection_msgs/FaceDetection face~%~%# leg detection in laser scan. If Point is (0,0,0) then no legs have been detected~%pal_detection_msgs/LegDetections legs~%~%# 3D position of the person with respect to a given frame specified in the header of the PointStamped.~%# If the position is not available then (0,0,0) is set.~%geometry_msgs/PointStamped position3D ~%~%~%================================================================================~%MSG: pal_detection_msgs/Detection2d~%## Rectangle defined by a point, its width and height~%# corresponds to the openCV struct : CvRect~%~%# coordinates of the top left corner of the box~%int64 x~%int64 y~%~%# width of the box~%int64 width~%~%# height of the box~%int64 height~%~%~%================================================================================~%MSG: pal_detection_msgs/FaceDetection~%~%##########################################~%#~%# Face detection data~%#~%##########################################~%~%#####################~%# Face bounding box~%#####################~%# coordinates of the top left corner of the box~%int32 x~%int32 y~%~%# width of the box~%int32 width~%~%# height of the box~%int32 height~%~%############################~%# Eyes position (if found)~%############################~%~%bool eyesLocated~%~%int32 leftEyeX~%int32 leftEyeY~%int32 rightEyeX~%int32 rightEyeY~%~%#############################~%# Centre of eyes 3D estimate~%#############################~%geometry_msgs/Point32 position~%~%~%############################~%# Person recognition~%############################~%~%string name~%float32 confidence~%~%############################~%# Facial expression~%############################~%string EXPRESSION_NEUTRAL=\"neutral\"~%string EXPRESSION_SMILE=\"smile\"~%string EXPRESSION_RAISED_BROWS=\"raised brows\"~%string EXPRESSION_EYES_AWAY=\"eyes away\"~%string EXPRESSION_SQUINTING=\"squinting\"~%string EXPRESSION_FROWNING=\"frowning\"~%string expression~%float32 expression_confidence~%~%############################~%# Facial emotion~%############################~%float32 emotion_anger_confidence~%float32 emotion_disgust_confidence~%float32 emotion_fear_confidence~%float32 emotion_happiness_confidence~%float32 emotion_neutral_confidence~%float32 emotion_sadness_confidence~%float32 emotion_surprise_confidence~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: pal_detection_msgs/LegDetections~%## Contains data relative to the detection of the legs of persons in a laser scan~%~%Header header~%~%geometry_msgs/Point[]  position3D    # 3D position of the persons' legs in a given frame~%~%# Optional transformation between the laser frame and a certain parent frame~%geometry_msgs/TransformStamped laser_pose~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PersonDetections)))
  "Returns full string definition for message of type 'PersonDetections"
  (cl:format cl:nil "Header header~%~%pal_detection_msgs/PersonDetection[] persons~%~%# Optional transformation between the camera frame and a certain parent frame~%geometry_msgs/TransformStamped camera_pose~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: pal_detection_msgs/PersonDetection~%## Contains detection correspondences among the different person detectors available~%~%# full body image detection. If width == height == 0 then no full body detection is available~%pal_detection_msgs/Detection2d full_body~%~%# face image detection. If width == height == 0 then the face has not been detected~%pal_detection_msgs/FaceDetection face~%~%# leg detection in laser scan. If Point is (0,0,0) then no legs have been detected~%pal_detection_msgs/LegDetections legs~%~%# 3D position of the person with respect to a given frame specified in the header of the PointStamped.~%# If the position is not available then (0,0,0) is set.~%geometry_msgs/PointStamped position3D ~%~%~%================================================================================~%MSG: pal_detection_msgs/Detection2d~%## Rectangle defined by a point, its width and height~%# corresponds to the openCV struct : CvRect~%~%# coordinates of the top left corner of the box~%int64 x~%int64 y~%~%# width of the box~%int64 width~%~%# height of the box~%int64 height~%~%~%================================================================================~%MSG: pal_detection_msgs/FaceDetection~%~%##########################################~%#~%# Face detection data~%#~%##########################################~%~%#####################~%# Face bounding box~%#####################~%# coordinates of the top left corner of the box~%int32 x~%int32 y~%~%# width of the box~%int32 width~%~%# height of the box~%int32 height~%~%############################~%# Eyes position (if found)~%############################~%~%bool eyesLocated~%~%int32 leftEyeX~%int32 leftEyeY~%int32 rightEyeX~%int32 rightEyeY~%~%#############################~%# Centre of eyes 3D estimate~%#############################~%geometry_msgs/Point32 position~%~%~%############################~%# Person recognition~%############################~%~%string name~%float32 confidence~%~%############################~%# Facial expression~%############################~%string EXPRESSION_NEUTRAL=\"neutral\"~%string EXPRESSION_SMILE=\"smile\"~%string EXPRESSION_RAISED_BROWS=\"raised brows\"~%string EXPRESSION_EYES_AWAY=\"eyes away\"~%string EXPRESSION_SQUINTING=\"squinting\"~%string EXPRESSION_FROWNING=\"frowning\"~%string expression~%float32 expression_confidence~%~%############################~%# Facial emotion~%############################~%float32 emotion_anger_confidence~%float32 emotion_disgust_confidence~%float32 emotion_fear_confidence~%float32 emotion_happiness_confidence~%float32 emotion_neutral_confidence~%float32 emotion_sadness_confidence~%float32 emotion_surprise_confidence~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: pal_detection_msgs/LegDetections~%## Contains data relative to the detection of the legs of persons in a laser scan~%~%Header header~%~%geometry_msgs/Point[]  position3D    # 3D position of the persons' legs in a given frame~%~%# Optional transformation between the laser frame and a certain parent frame~%geometry_msgs/TransformStamped laser_pose~%~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TransformStamped~%# This expresses a transform from coordinate frame header.frame_id~%# to the coordinate frame child_frame_id~%#~%# This message is mostly used by the ~%# <a href=\"http://www.ros.org/wiki/tf\">tf</a> package. ~%# See its documentation for more information.~%~%Header header~%string child_frame_id # the frame id of the child frame~%Transform transform~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/PointStamped~%# This represents a Point with reference coordinate frame and timestamp~%Header header~%Point point~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PersonDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'persons) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PersonDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'PersonDetections
    (cl:cons ':header (header msg))
    (cl:cons ':persons (persons msg))
    (cl:cons ':camera_pose (camera_pose msg))
))
