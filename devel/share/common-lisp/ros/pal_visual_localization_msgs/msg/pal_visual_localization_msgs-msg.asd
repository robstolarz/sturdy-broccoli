
(cl:in-package :asdf)

(defsystem "pal_visual_localization_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "VisualLocAddPlaceFeedback" :depends-on ("_package_VisualLocAddPlaceFeedback"))
    (:file "_package_VisualLocAddPlaceFeedback" :depends-on ("_package"))
    (:file "VisualLocRecognizeActionFeedback" :depends-on ("_package_VisualLocRecognizeActionFeedback"))
    (:file "_package_VisualLocRecognizeActionFeedback" :depends-on ("_package"))
    (:file "VisualLocAddPlaceAction" :depends-on ("_package_VisualLocAddPlaceAction"))
    (:file "_package_VisualLocAddPlaceAction" :depends-on ("_package"))
    (:file "VisualLocRecognizeGoal" :depends-on ("_package_VisualLocRecognizeGoal"))
    (:file "_package_VisualLocRecognizeGoal" :depends-on ("_package"))
    (:file "VisualLocRecognizeActionResult" :depends-on ("_package_VisualLocRecognizeActionResult"))
    (:file "_package_VisualLocRecognizeActionResult" :depends-on ("_package"))
    (:file "VisualLocAddPlaceActionResult" :depends-on ("_package_VisualLocAddPlaceActionResult"))
    (:file "_package_VisualLocAddPlaceActionResult" :depends-on ("_package"))
    (:file "VisualLocRecognizeFeedback" :depends-on ("_package_VisualLocRecognizeFeedback"))
    (:file "_package_VisualLocRecognizeFeedback" :depends-on ("_package"))
    (:file "VisualLocAddPlaceActionGoal" :depends-on ("_package_VisualLocAddPlaceActionGoal"))
    (:file "_package_VisualLocAddPlaceActionGoal" :depends-on ("_package"))
    (:file "VisualLocAddPlaceResult" :depends-on ("_package_VisualLocAddPlaceResult"))
    (:file "_package_VisualLocAddPlaceResult" :depends-on ("_package"))
    (:file "VisualLocRecognizeAction" :depends-on ("_package_VisualLocRecognizeAction"))
    (:file "_package_VisualLocRecognizeAction" :depends-on ("_package"))
    (:file "VisualLocAddPlaceActionFeedback" :depends-on ("_package_VisualLocAddPlaceActionFeedback"))
    (:file "_package_VisualLocAddPlaceActionFeedback" :depends-on ("_package"))
    (:file "VisualLocRecognizeResult" :depends-on ("_package_VisualLocRecognizeResult"))
    (:file "_package_VisualLocRecognizeResult" :depends-on ("_package"))
    (:file "VisualLocAddPlaceGoal" :depends-on ("_package_VisualLocAddPlaceGoal"))
    (:file "_package_VisualLocAddPlaceGoal" :depends-on ("_package"))
    (:file "VisualLocRecognizeActionGoal" :depends-on ("_package_VisualLocRecognizeActionGoal"))
    (:file "_package_VisualLocRecognizeActionGoal" :depends-on ("_package"))
  ))