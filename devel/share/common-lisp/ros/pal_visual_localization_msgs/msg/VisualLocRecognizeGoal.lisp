; Auto-generated. Do not edit!


(cl:in-package pal_visual_localization_msgs-msg)


;//! \htmlinclude VisualLocRecognizeGoal.msg.html

(cl:defclass <VisualLocRecognizeGoal> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass VisualLocRecognizeGoal (<VisualLocRecognizeGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualLocRecognizeGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualLocRecognizeGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name pal_visual_localization_msgs-msg:<VisualLocRecognizeGoal> is deprecated: use pal_visual_localization_msgs-msg:VisualLocRecognizeGoal instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualLocRecognizeGoal>) ostream)
  "Serializes a message object of type '<VisualLocRecognizeGoal>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualLocRecognizeGoal>) istream)
  "Deserializes a message object of type '<VisualLocRecognizeGoal>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualLocRecognizeGoal>)))
  "Returns string type for a message object of type '<VisualLocRecognizeGoal>"
  "pal_visual_localization_msgs/VisualLocRecognizeGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualLocRecognizeGoal)))
  "Returns string type for a message object of type 'VisualLocRecognizeGoal"
  "pal_visual_localization_msgs/VisualLocRecognizeGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualLocRecognizeGoal>)))
  "Returns md5sum for a message object of type '<VisualLocRecognizeGoal>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualLocRecognizeGoal)))
  "Returns md5sum for a message object of type 'VisualLocRecognizeGoal"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualLocRecognizeGoal>)))
  "Returns full string definition for message of type '<VisualLocRecognizeGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualLocRecognizeGoal)))
  "Returns full string definition for message of type 'VisualLocRecognizeGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualLocRecognizeGoal>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualLocRecognizeGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualLocRecognizeGoal
))
