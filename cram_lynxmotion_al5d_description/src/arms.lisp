(in-package :al5d-descr)

(defparameter *tcp-in-ee-pose*
  (cl-transforms:make-pose
   (cl-transforms:make-3d-vector 0 0 0.3191d0)
   (cl-transforms:make-identity-rotation)))

(defparameter *standard-to-al5d-gripper-transform*
  (cl-transforms-stamped:make-identity-transform))

(defparameter *left-parking-joint-states*
  '(("Joint1" 1.57)
    ("Joint2" 1.57)
    ("Joint3" -1.57)
    ("Joint4" 0.00)
    ("Joint5" 0.00)
    ("Gripper" 0.00)))
    ;("GripperBis" 0.00)))

(defparameter *right-parking-joint-states*
   '(("Joint1" -1.57)
    ("Joint2" 1.57)
    ("Joint3" -1.57)
    ("Joint4" 0.00)
    ("Joint5" 0.00)
    ("Gripper" 0.00)))
    ;("GripperBis" 0.00)))

(defparameter *straight-pointing-joint-states*
    '(("Joint1" 0.00)
    ("Joint2" 1.57)
    ("Joint3" -1.57)
    ("Joint4" 0.00)
    ("Joint5" 0.00)
    ("Gripper" 0.00)))
    ;("GripperBis" 0.00)))


(def-fact-group al5d-arm-facts (end-effector-link
                                robot-tool-frame
                                arm-joints arm-links
                                gripper-joint gripper-link
                                gripper-meter-to-joint-multiplier
                                standard-to-particular-gripper-transform
                                robot-joint-states)

  (<- (end-effector-link al5d :left "left_finger"))

  (<- (robot-tool-frame al5d :left "gripper"))

  (<- (arm-joints al5d :right ("world_to_robot"
                              "camera_joint"
                              "base_to_cylinder"
                              "Joint1"
                              "Joint2"
                              "Joint3"
                              "Joint4"
                              "Joint5"
                              "Gripper"
                              "right_finger_joint"
                              "left_finger_joint")))

  (<- (arm-links al5d :right ("robot_base_cylinder"
                             "robot_support"
                             "robot_base_top_box"
                             "shoulder"
                             "arm"
                             "wrist"
                             "gripper"
                             "left_finger"
                             "right_finger"
                             "gripper_mov" )))

  (<- (gripper-joint al5d :right "Gripper"))
  ;(<- (gripper-joint al5d :right "GripperBis"))

  (<- (gripper-link al5d :right ?link)
    (bound ?link)
    (lisp-fun search "Gripper" ?link ?pos)
    (lisp-pred identity ?pos))
  #| (<- (gripper-link al5d :right ?link)
    (bound ?link)
    (lisp-fun search "GripperBis" ?link ?pos)
    (lisp-pred identity ?pos)) |#

  (<- (gripper-meter-to-joint-multiplier al5d 0.002))

  (<- (standard-to-particular-gripper-transform al5d ?transform)
    (symbol-value *standard-to-al5d-gripper-transform* ?transform))

  (<- (robot-joint-states al5d :arm :right :point-ahead ?joint-states)
    (symbol-value *straight-pointing-joint-states* ?joint-states))

  (<- (robot-joint-states al5d :arm :right :park ?joint-states)
    (symbol-value *left-parking-joint-states* ?joint-states))

  (<- (robot-joint-states al5d :arm :right :park ?joint-states)
    (symbol-value *right-parking-joint-states* ?joint-states)))
