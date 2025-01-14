(define (domain marker_detection)


(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types 
    robot
    waypoint
)



(:predicates 
    (robot_at ?r - robot ?wp - waypoint)  ; The robot is at a specific waypoint
    (visited ?wp - waypoint)              ; The robot has visited a waypoint
    (marker_detected ?wp - waypoint)      ; The marker at the waypoint has been detected
    (all_markers_detected)                ; All markers have been detected
    
)




;; Move to a waypoint
  (:action move_to
    :parameters (?r - robot ?from ?to - waypoint)
    :precondition (and
      (robot_at ?r ?from)                ; The robot is at the source waypoint

    )
    :effect (and
      (not (robot_at ?r ?from))          ; The robot leaves the source waypoint
      (robot_at ?r ?to)                  ; The robot reaches the target waypoint
    )
  )

 ;; Detect the marker at the current waypoint
  (:action detect_marker
    :parameters (?r - robot ?wp - waypoint)
    :precondition (and
      (robot_at ?r ?wp)                  ; The robot is at the waypoint
    )
    :effect (and
      (marker_detected ?wp)              ; The marker at the waypoint is detected
      (visited ?wp)                      ; The waypoint is marked as visited
    )
  )

(:action go_to_least_id
      :parameters ()
      :precondition (and
        (forall (?w - waypoint) (marker_detected ?w))  ; All markers must be detected
        (forall (?w - waypoint) (visited ?w))         ; All waypoints must be visited
      )
      :effect (and
        (all_markers_detected)                        ; Set the flag indicating all markers are detected
      )
    )


)
