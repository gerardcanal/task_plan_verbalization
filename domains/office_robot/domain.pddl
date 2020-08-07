(define (domain warehouse_domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)

(:types
	waypoint robot person - locatable
        object

)

(:predicates
	(robot_at ?v - robot ?wp - waypoint)
        (person_nexto ?v - person ?wp - waypoint)
	(connected ?from ?to - waypoint)
	(visited ?wp - waypoint)
	(not_occupied ?wp - waypoint)
	(scanned_place ?place - waypoint)
	(object_at ?p - object ?l - locatable)
	(not_holding_object ?v - robot)
        (graspable ?p - object)
        (person_found ?p - person)
        (person_not_found ?p - person)
        (person_asked ?p - person)
)

(:functions

	(distance ?wp1 ?wp2 - waypoint) 
)

(:durative-action goto_waypoint
  :parameters (?v - robot ?from ?to - waypoint)
  :duration(= ?duration (distance ?from ?to))
  :condition (and
    (at start (robot_at ?v ?from))
    (at start (not_occupied ?to))
    (over all (connected ?from ?to)))
  :effect (and
    (at start (not (not_occupied ?to)))
    (at end (not_occupied ?from))
    (at start (not (robot_at ?v ?from)))
    (at end (robot_at ?v ?to)))
)

(:durative-action scan_place
  :parameters (?v - robot ?place - waypoint)
  :duration(= ?duration 1)
  :condition (and 
    (over all (robot_at ?v ?place)))
  :effect (and 
    (at end (scanned_place ?place)))
)

(:durative-action find_person
  :parameters (?v - robot ?p - person)
  :duration(= ?duration 20)
  :condition (and 
    (at start (person_not_found ?p)))
  :effect (and
    (at end (person_found ?p))
    (at end (not (person_not_found ?p))))
)

(:durative-action ask_person
  :parameters (?v - robot ?p - person ?wp - waypoint)
  :duration(= ?duration 2)
  :condition (and 
    (over all (robot_at ?v ?wp))
    (over all (person_nexto ?p ?wp))
    (over all (person_found ?p)))
  :effect (and (at end (person_asked ?p)))
)

(:durative-action take_object
  :parameters (?v - robot ?o - object ?place - waypoint ?p - person)
  :duration(= ?duration 2)
  :condition (and 
    (over all (robot_at ?v ?place))
    (over all (person_nexto ?p ?place))
    (over all (graspable ?o))
    (over all (person_found ?p))
    (at start (object_at ?o ?p))
    (at start (not_holding_object ?v)))
  :effect (and
    (at start (not (object_at ?o ?p)))
    (at start (not (not_holding_object ?v)))
    (at end (object_at ?o ?v)))
)

(:durative-action grasp_object
  :parameters (?v - robot ?p - object ?place - waypoint)
  :duration(= ?duration 2)
  :condition (and 
    (over all (robot_at ?v ?place))
    (over all (graspable ?p))
    (at start (object_at ?p ?place))
    (at start (not_holding_object ?v)))
  :effect (and
    (at start (not (object_at ?p ?place)))
    (at start (not (not_holding_object ?v)))
    (at end (object_at ?p ?v)))
)

(:durative-action place_object
  :parameters (?v - robot ?p - object ?place - waypoint)
  :duration(= ?duration 2.5)
  :condition (and
    (at start (object_at ?p ?v))
    (over all (robot_at ?v ?place))
    (over all (scanned_place ?place)))
  :effect (and
    (at start (not (object_at ?p ?v)))
    (at end (object_at ?p ?place))
    (at end (not_holding_object ?v)))
)

(:durative-action give_object
  :parameters (?v - robot ?o - object ?place - waypoint ?p - person)
  :duration(= ?duration 2)
  :condition (and
    (at start (object_at ?o ?v))
    (over all (robot_at ?v ?place))
    (over all (person_nexto ?p ?place))
    (over all (person_found ?p)))
  :effect (and
    (at start (object_at ?o ?p))
    (at end (not (object_at ?o ?v)))
    (at end (not_holding_object ?v))))
)

)
