(in-package :al5d)

(defparameter *gripper-open* 0.03d0 "Gripper opening distance in m")
(defparameter *gripper-closed* 0.0d0 "Gripper closed distance")
(defparameter *object-approach-distance* 0.100d0 "Approched distance in the -z direction")
(defparameter *grasp-theta* (* -1 pi) "Grasp angle in the y direction")
(defparameter *end-effector-length* 0.100d0 "The length of the end effector")

(defun pick (?position ?orientation)
        ; First we open the gripper
        (let ((?end-effector-position (append (butlast ?position) (list (+ *end-effector-length* (car (cdr (cdr ?position)))))))
              (?neg-orientation (* -1 ?orientation)))
            (exe:perform (a motion (type grasping) (distance *gripper-open*)))
            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (position ?end-effector-position) (orientation ?neg-orientation)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (position ?end-effector-position) (yaw ?neg-orientation) (pitch *grasp-theta*)))
            ; Close the fingers
            (exe:perform (a motion (type grasping) (distance *gripper-closed*)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (position ?end-effector-position) (orientation ?neg-orientation)))))

(defun place (?position ?orientation)
        ; We assume that the gripper is open already. So we just move to the approach and place
        (let ((?end-effector-position (append (butlast ?position) (list (+ *end-effector-length* (car (cdr (cdr ?position)))))))
              (?neg-orientation (* -1 ?orientation)))
            ; Move to the robot approach pose
            (exe:perform (an action (type approaching) (position ?end-effector-position) (orientation ?neg-orientation)))
            ; Now we go to the grasp pose
            (exe:perform (a motion (type moving) (position ?end-effector-position) (yaw ?neg-orientation) (pitch *grasp-theta*)))
            ; Open the fingers
            (exe:perform (a motion (type grasping) (distance *gripper-open*)))
            ; Go back to the approach pose
            (exe:perform (an action (type approaching) (position ?end-effector-position) (orientation ?neg-orientation)))
            ; Now close the fingers
            (exe:perform (a motion (type grasping) (distance *gripper-closed*)))))

(defun approach (?position ?orientation)
    ; Sends the robot to a specific approach pose
    (let ((?approach-pose ?position))
          ; Need to substract 100 mm from the z
          (setf ?approach-pose (append   (butlast ?approach-pose)
                    (list (+ *object-approach-distance* (car (cdr (cdr ?approach-pose)))))))
          (exe:perform (a motion (type moving) (position ?approach-pose) (yaw ?orientation) (pitch *grasp-theta*)))))

(defun pick-and-place   (?from-position ?from-orientation
                         ?to-position ?to-orientation)
    ; First pick
    (exe:perform (an action (type picking) (position ?from-position) (orientation ?from-orientation)))
    ; Then place
    (exe:perform (an action (type placing) (position ?to-position) (orientation ?to-orientation))))


(defun demo ()
	(go-home)
    (let* ((?object-position '(0 0.187 0))
            (?object-theta (* -1 (/ pi 2)))
            (?example-position (append (butlast ?object-position) (list 0.216)))
            (?tray-position '(0.150 0.100 0.100)))
        (exe:perform (a motion (type moving) (position (append (butlast ?example-position) (list (+ *end-effector-length* (last ?example-position))))) (yaw 0)))
        (exe:perform (a motion (type moving) (position (append (butlast ?example-position) (list (+ *end-effector-length* (last ?example-position))))) (yaw (/ pi 2))))
	(exe:perform (a motion (type moving) (position '((car ?example-position) (+ (cadr ?example-position) *end-effector-length*) (last ?example-position))) (yaw (/ pi 2)) (pitch (/ pi 2))))
	(exe:perform (a motion (type moving) (position '((car ?example-position) (+ (cadr ?example-position) *end-effector-length*) (last ?example-position))) (pitch (/ pi 2)) (roll (* -1 (/ pi 2)))))
	(exe:perform (a motion (type moving) (position '((car ?example-position) (+ (cadr ?example-position) *end-effector-position*) 0.02)) (yaw (/ pi 2)) (pitch (/ pi 2))))
	(exe:perform (an action (type approaching) (position ?object-position) (orientation ?object-theta)))
	(exe:perform (a motion (type moving) (position ?object-position) (pitch pi) (yaw ?object-theta)))))
