(in-package :al5d)

(defparameter *gripper-open* 0.03d0 "Gripper opening distance in m")
(defparameter *gripper-closed* 0.0d0 "Gripper closed distance")
(defparameter *object-approach-distance* 0.100d0 "Approched distance in the -z direction")
(defparameter *grasp-theta* (* -1 pi) "Grasp angle in the y direction")
(defparameter *end-effector-length* 0.100d0 "The length of the end effector")

(defun pick (?destination)
        ; First we open the gripper
        (let  ((?goal (cl-transforms:make-pose
                        (cl-transforms:v+ (cl-transforms:origin ?destination) 
                            (cl-transforms:make-3d-vector 0 0 *end-effector-length*))
                        (cl-transforms:orientation ?destination))))

            ; Open the end effector
            (exe:perform (a motion (type grasping) (distance *gripper-open*)))
            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (destination ?goal)))
            ; Close the fingers
            (exe:perform (a motion (type grasping) (distance *gripper-closed*)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (at ?goal)))))

(defun place (?destination)
        ; First we open the gripper
        (let  ((?goal (cl-transforms:make-pose
                        (cl-transforms:v+ (cl-transforms:origin ?destination) 
                            (cl-transforms:make-3d-vector 0 0 *end-effector-length*))
                        (cl-transforms:orientation ?destination))))

            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (destination ?goal)))
            ; Open the end effector
            (exe:perform (a motion (type grasping) (distance *gripper-open*)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (at ?goal)))
            ; Close the fingers
            (exe:perform (a motion (type grasping) (distance *gripper-closed*)))))

(defun approach (?target)
    ; Sends the robot to a specific approach pose
    (let ((?approach-pose (cl-transforms:make-pose 
                            (cl-transforms:v+ (cl-transforms:origin ?target)
                                (cl-transforms:make-3d-vector 0 0 *object-approach-distance*))
                            (cl-transforms:orientation ?target))))
        (exe:perform (a motion (type moving) (destination ?approach-pose)))))

(defun pick-and-place (?from ?to)
    ; First pick
    (exe:perform (an action (type picking) (from ?from)))
    ; Then place
    (exe:perform (an action (type placing) (to ?to))))

(defun demo()
    (go-home)
    ; Let us run the whole thing from here
    (let ((?goal-1 (reference (a location (step-no 1)))))
        (exe:perform (desig:a motion (type now-moving) (destination ?goal-1)))))
