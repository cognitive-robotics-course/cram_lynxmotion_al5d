(in-package :al5d-descr)

(def-fact-group al5d-metadata (robot
                               robot-base-frame 
                               robot-odom-frame
                               arm
                               camera-frame
                               camera-minimal-height
                               camera-maximal-height)
  (<- (robot al5d))

  (<- (robot-odom-frame al5d "world"))
  (<- (robot-base-frame al5d "robot_support"))

  (<- (arm al5d :right))

  (<- (camera-frame al5d "camera_link"))

  (<- (camera-minimal-height al5d 0.10))
  (<- (camera-maximal-height al5d 1.00))
  )
  

; Gaussian distribution configuration
(def-fact-group location-costmap-metadata ( costmap:costmap-size
                                            costmap:costmap-origin
                                            costmap:costmap-resolution
                                            costmap:orientation-samples
                                            costmap:orientation-sample-step
                                            costmap:costmap-padding
                                            costmap:costmap-manipulation-padding
                                            costmap:costmap-in-reach-distance
                                            costmap:costmap-reach-minimal-distance
                                            costmap:visibility-costmap-size)
    (<- (costmap:costmap-size 12 12))
    (<- (costmap:costmap-origin -6 -6))
    (<- (costmap:costmap-resolution 0.04))
 
    (<- (costmap:costmap-padding 0.3))
    (<- (costmap:costmap-manipulation-padding 0.4))
    (<- (costmap:costmap-in-reach-distance 0.7))
    (<- (costmap:costmap-reach-minimal-distance 0.2))
    (<- (costmap:visibility-costmap-size 2))
    (<- (costmap:orientation-samples 2))
    (<- (costmap:orientation-sample-step 0.1)))