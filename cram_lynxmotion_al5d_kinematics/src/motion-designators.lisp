(in-package :al5d)

(def-fact-group robotic-arm-motion-designators (motion-grounding)
    ;; Check for and extract necessary information for each motion

(<- (desig:motion-grounding ?desig (move ?goal))
    (desig-prop ?desig (:type :moving))
    (desig-prop ?desig (:position ?position))
    (desig-prop ?desig (:yaw ?yaw))
    (desig-prop ?desig (:pitch ?pitch))
    (lisp-fun apply cl-transforms:make-3d-vector ?position ?point)
    ;; A 180 degrees rotation is always applied to the y-axis in the original solution
    (lisp-fun cl-transforms:euler->quaternion :ax ?pitch :az ?yaw ?quaternion)
    (lisp-fun cl-transforms:make-pose ?point ?quaternion ?goal))

(<- (desig:motion-grounding ?desig (grasp ?distance))
    (desig-prop ?desig (:type :grasping))
    (desig-prop ?desig (:distance ?distance))))
