(define (problem task)
(:domain marker_detection)
(:objects
    robot1 - robot
    wp0 wp1 wp2 wp3 wp4 - waypoint
)
(:init
    (robot_at robot1 wp4)

    (visited wp4)

    (marker_detected wp4)


)
(:goal (and
    (all_markers_detected)
))
)
