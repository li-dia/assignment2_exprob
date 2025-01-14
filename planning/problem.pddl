(define (problem visit_and_detect) (:domain marker_detection)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
   
    robot1 - robot
)
(:init
    (robot_at robot1 wp4)          ; The robot starts at waypoint wp0
    (marker_detected wp4)
    (visited wp4)
  )
  (:goal
    (and
      (all_markers_detected)
    )
  )


)
