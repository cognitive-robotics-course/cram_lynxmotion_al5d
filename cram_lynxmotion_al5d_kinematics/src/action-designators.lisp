(in-package :al5d)

(def-fact-group lynxmotion-al5d-action-designators (action-grounding)
    ; Demo/robotProgramming action designator
    (<- (desig:action-grounding ?desig (demo))
        (desig-prop ?desig (:type :demoing)))	

    ; Action designator for picking an object
    (<- (desig:action-grounding ?desig (pick ?position ?orientation))
        (desig-prop ?desig (:type :picking))
        (desig-prop ?desig (:position ?position))
        (desig-prop ?desig (:orientation ?orientation)))

    ; Action designator for placing an object
    (<- (desig:action-grounding ?desig (place ?position ?orientation))
        (desig-prop ?desig (:type :placing))
        (desig-prop ?desig (:position ?position))
        (desig-prop ?desig (:orientation ?orientation)))

    ; Action designator for approaching an object
    (<- (desig:action-grounding ?desig (approach ?position ?orientation))
        (desig-prop ?desig (:type :approaching))
        (desig-prop ?desig (:position ?position))
        (desig-prop ?desig (:orientation ?orientation)))

    ; Action designator for a complete pick and place exercice
    (<- (desig:action-grounding ?desig (pick-and-place ?source-position 
                                        ?source-orientation ?dest-position 
                                        ?dest-orientation))
        (desig-prop ?desig (:type :picking-and-placing))
        (desig-prop ?desig (:from ?source-position))
        (desig-prop ?desig (:is-heading ?source-orientation))
        (desig-prop ?desig (:to ?dest-position))
        (desig-prop ?desig (:will-head ?dest-orientation))))
