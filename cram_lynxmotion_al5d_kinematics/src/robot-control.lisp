(in-package :al5d)

;;; Setting parameters to be used in the program to control the physical robot

(defparameter *max-filename-length* 80 "Maximum length of the configuration file")
(defparameter *string-length* 200 "Maximum length of a string")
(defparameter *key-length* 20 "Length of a key in the configuration file")
(defparameter *number-of-keys* 9 "Number of keys expected in the configuration file")
    
;;; Parameters for the servo control
(defparameter *comand-size* 200 "size of the comand")
(defparameter *max-servos* 32 "maximum number of servos")
(defparameter *min-pw* 750 "lowest pulse width")
(defparameter *max-pw* 2250 "highest pulse width")

;;; Working envelope in m
(defparameter *min-x* -0.130d0)
(defparameter *max-x* 0.130d0)
(defparameter *min-y* 0.080d0)
(defparameter *max-y* 0.330d0)
(defparameter *min-z* 0.00d0)
(defparameter *max-z* 0.380d0)

;;; Kinematics: arm dimensions (metres) for AL5D arm
(defparameter *d1* 0.070 "Base height to X/Y plane")
(defparameter *a3* 0.146d0 "Shoulder-to-elbow \"bone\"")
(defparameter *a4* 0.187d0 "Elbow-to-wrist \"bone\"")
(defparameter *ez* 0.100d0 "Gripper length")


;;; Get the current positions from the subscriber. Values are set to zero
(defvar *current-positions* '(0.0 0.0 0.0 0.0 0.0 0.0)' "Current joint positions")

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

(defun compute-joint-angles (x y z pitch yaw)
	"Transform from wrist pose (x, y, z, pitch, and yaw in degrees) to joint angles using the inverse kinematics
	 If resulting arm position is physically unreachable, return false.
	 Otherwise, return the corresponding joint angles in radians."
	
	; Print the values to the terminal if the debug flag is set
	(if *debug*
		(format t "computeJointAngles(): x ~,2F, y ~,2F, z ~,2F, pitch ~,2F, yaw ~,2F~%" x y z pitch yaw))

	(let* (
		(hum_sq (* *a3* *a3*))
		(uln_sq (* *a4* *a4*))

		; Grip angles in randians for computations
		(pitch_r (to-radians pitch))
		(yaw_r (to-radians yaw))
		(bas_angle_r (atan x y))
		(bas_angle_d (to-degrees bas_angle_r))
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
		(shl_angle_r (+ a1 a2)))

		(if (is-nan-or-infinity shl_angle_r)
			nil
			(let* (
				(shl_angle_d (to-degrees shl_angle_r))
				(a1_d (to-degrees a1))
				(a2_d (to-degrees a2))
				(elb_angle_r (acos 
							(/ (- (+ hum_sq uln_sq) sw)
							(* 2 *a3* *a4*)))))
			(if (is-nan-or-infinity elb_angle_r)
				nil
				(let* (
					(elb_angle_r (* -1 elb_angle_r))
                    (elb_angle_r (* -1 (+ pi elb_angle_r)))
					(elb_angle_d (to-degrees elb_angle_r))
					(wri_yaw_angle_d nil)
					(wri_pitch_angle_d (- (+ 90 pitch) (+ elb_angle_d shl_angle_d))))

				(if (equal 0 (round pitch))
					(setf wri_yaw_angle_d (+ yaw bas_angle_d 90))
					; else here
					(if (or (equal -180 (round pitch)) (equal 180 (round pitch)))
						(setf wri_yaw_angle_d (+ (- yaw bas_angle_d) 90))
						; else again
						(setq wri_yaw_angle_d (+ 90 yaw))))
					
					(if *debug*
						(progn
							(format t "Joint 1 (degrees): ~,2F ~%" bas_angle_d)
							(format t "Joint 2 (degrees): ~,2F ~%" shl_angle_d)
							(format t "Joint 3 (degrees): ~,2F ~%" elb_angle_d)
							(format t "Joint 4 (degrees): ~,2F ~%" wri_pitch_angle_d)
							(format t "Joint 5 (degrees): ~,2F ~%~%" wri_yaw_angle_d)

							(format t "Joint 1 (radians): ~,2F ~%" bas_angle_r)
							(format t "Joint 2 (radians): ~,2F ~%" shl_angle_r)
							(format t "Joint 3 (radians): ~,2F ~%" elb_angle_r)
							(format t "Joint 4 (radians): ~,2F ~%" (to-radians wri_pitch_angle_d))
							(format t "Joint 5 (radians): ~,2F ~%~%" (to-radians wri_yaw_angle_d))))
					;;; Return the list of computed angle
					(list 
						bas_angle_r 
						shl_angle_r 
						elb_angle_r  
						(to-radians wri_pitch_angle_d)
						(to-radians wri_yaw_angle_d))))))))


(defun set-joint-positions (joint-positions)
	"Receives the joint positions and publishes them to the robot"
    ;;; The value for Joint3 should be altered before publishing to the robot
	(publish-joint-positions joint-positions)
	(sleep *default-sleep-time*))

(def-cram-function move-to (goal)
	"Receives a goal destination as a pose, gets the necessary joint angles from the robot
		and publishes them on the topic."
	; Retrieves the values from the pose and set them 
    
	(let ((position (cl-transforms:origin goal))
		  (rpy-orientation 
			(cl-transforms:quaternion->euler 
				(cl-transforms:orientation goal) :just-values t)))
		
		; Retrieve the values and send them to the function
		(set-joint-positions
			(compute-joint-angles 
				; XYZ values
				(cl-transforms:x position)
				(cl-transforms:y position)
				(cl-transforms:z position)
				
				; We are using rotations around x and z
				(to-degrees (car rpy-orientation))
                (to-degrees (third rpy-orientation))))
	t))

(def-cram-function grasp (distance)
	"Takes a grasp distance in metres and sends the joints positions to 
		apply the opening to the robot."
	(let ((joint-positions (append (butlast *current-positions*) 
			(list distance ))))
        (when *debug*
            (format t "Sending joints ~A~%" joint-positions))
        (set-joint-positions joint-positions)))
