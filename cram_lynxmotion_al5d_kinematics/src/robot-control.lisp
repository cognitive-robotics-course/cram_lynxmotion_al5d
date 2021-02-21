(in-package :al5d)

;;; Kinematics: arm dimensions (metres) for AL5D arm
(defparameter *d1* 0.070 "Base height to X/Y plane")
(defparameter *a3* 0.146d0 "Shoulder-to-elbow \"bone\"")
(defparameter *a4* 0.187d0 "Elbow-to-wrist \"bone\"")
(defparameter *ez* 0.100d0 "Gripper length")

;;; Get the current positions from the subscriber. Values are set to zero
(defvar *current-positions* '(0.0 0.0 0.0 0.0 0.0 0.0)' "Current joint positions")
(defvar *home-position* '(0.0 1.57 -1.57 0.0 0.0 0.03175)' "Joint values for the home positions")

(defun to-degrees (angle)
	"Returns the value of a radian angle in degrees"
	(/ (* angle 180) pi))

(defun to-radians (angle)
	"Returns the value of a degrees angle in radians"
	(/ (* angle pi) 180))

(defun is-nan-or-infinity (value)
	"Returns a boolean specifying whether the value passed is not a number or 
		is the infinity number"
	(or 
		(not value) 
		(or 
			(<= value MOST-NEGATIVE-DOUBLE-FLOAT) 
					(>= value MOST-POSITIVE-DOUBLE-FLOAT))))

(defun compute-joint-angles (?object-pose)
	"Transform from wrist pose to joint angles using the inverse kinematics
	 If resulting arm position is physically unreachable, return false.
	 Otherwise, return the corresponding joint angles in radians."
	
    (let* ((?position (cl-transforms:origin ?object-pose))
           (?orientation (cl-transforms:orientation ?object-pose))
           (angles (cl-transforms:quaternion->euler ?orientation))
           (x (cl-transforms:x ?position))
           (y (cl-transforms:y ?position))
           (z (cl-transforms:z ?position))
           (pitch (fourth angles))
           (yaw (sixth angles)))
        ; Print the values to the terminal if the debug flag is set
        (if *debug*
            (format t "computeJointAngles(): x ~,2F, y ~,2F, z ~,2F, pitch ~,2F, yaw ~,2F~%" x y z pitch yaw))

            (let* (
                (hum_sq (* *a3* *a3*))
                (uln_sq (* *a4* *a4*))
                ; Grip angles in randians for computations
                (bas_angle (atan x y))
                (rdist (sqrt (+ (* x x) (* y y))))
                ; Wrist position
                (wrist_z (- z *d1*))
                (wrist_y rdist)
                ; Shoulder to wrist distance (aka sw)
                (sw (+ (* wrist_z wrist_z) (* wrist_y wrist_y)))
                (sw_sqrt (sqrt sw))
                ; sw angle to ground
                (a1 (atan wrist_z wrist_y))
                ; sw angle to A3
                (a2 (acos (/ (+ (- hum_sq uln_sq) sw) (* 2 *a3* sw_sqrt))))
                ; shoulder angle
                (shl_angle (+ a1 a2)))

                (if (is-nan-or-infinity shl_angle)
                    nil
                (let* (
                    (elb_angle (acos 
                                (/ (- (+ hum_sq uln_sq) sw)
                                (* 2 *a3* *a4*)))))
                (if (is-nan-or-infinity elb_angle)
                    nil
                    (let* (
                        (elb_angle (* -1 (- pi elb_angle)))
                        (wri_yaw_angle nil)
                        (wri_roll_angle (- (+ (/ pi 2) pitch) (+ elb_angle shl_angle))))

				(if (equal 0 (round (to-degrees pitch)))
					(setf wri_yaw_angle (+ yaw bas_angle (/ pi 2)))
					; else here
					(if (equal 0 (mod (round (to-degrees pitch)) 180))
						(setf wri_yaw_angle (+ (- yaw bas_angle) (/ pi 2)))
						; else again
						(setf wri_yaw_angle (+ (/ pi 2) yaw))))

                ; Because cl-tranforms tries to change the angles as appropriate,
                ; -pi can be interpreted as 0 leading to some errors in computation.
                ; This provides a temporary fix.
                (when (> wri_yaw_angle pi)
                    (setf wri_roll_angle (- wri_roll_angle pi))
                    (setf wri_yaw_angle (- pi wri_yaw_angle)))
					
					(if *debug*
						(progn
							(format t "Joint 1 : ~,2F ~%" bas_angle)
							(format t "Joint 2 : ~,2F ~%" shl_angle)
							(format t "Joint 3 : ~,2F ~%" elb_angle)
							(format t "Joint 4 : ~,2F ~%" wri_roll_angle)
							(format t "Joint 5 : ~,2F ~%~%" wri_yaw_angle)))
					;;; Return the list of computed angle
					(list 
						bas_angle 
						shl_angle 
						elb_angle  
						wri_roll_angle
						wri_yaw_angle))))))))


(defun set-joint-positions (joint-positions)
	"Receives the joint positions and publishes them to the robot"
	(publish-joint-positions joint-positions)
	(sleep *default-sleep-time*))

(defun go-home ()
    "Sends the robotic arm to home"
    (set-joint-positions *home-position*))

(def-cram-function grasp (distance)
	"Takes a grasp distance in metres and sends the joints positions to 
		apply the opening to the robot."
	(let ((joint-positions (append (butlast *current-positions*) 
			(list distance ))))
        (when *debug*
            (format t "Sending joints ~A~%" joint-positions))
        (set-joint-positions joint-positions)))

(def-cram-function move-to (destination)
    "Takes a cl-transforms:pose object and sends the robot at the
     specified position"

    (set-joint-positions
        (compute-joint-angles destination)))

