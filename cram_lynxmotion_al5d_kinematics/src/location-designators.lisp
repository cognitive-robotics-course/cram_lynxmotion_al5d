(in-package :al5d)

(defparameter *end-effector-length* 0.100d0 "The length of the end effector")

(defun robot-programming-goals-generator (designator)
    (declare (type location-designator designator))
    ;; Step no so that we know where we are
    (with-desig-props (step-no) designator
        (let* ((?object-position '(0 0.187 0))
                (?object-theta (* -1 (/ pi 2)))
                (?example-position (append (butlast ?object-position) (list 0.216)))
                (?tray-position '(0.150 0.100 0.100)))
            (format t "~f" (second ?object-position))
            (ecase step-no
                (1 
                    (loop repeat 1 
                            collect (cl-transforms:make-pose 
                                        (cl-transforms:make-3d-vector
                                             (first ?object-position)
                                             (second ?object-position)
                                             (+ *end-effector-length* (third ?object-position)))
                                        (cl-transforms:euler->quaternion :az ?object-theta))))))))

(register-location-generator 5 robot-programming-goals-generator)

