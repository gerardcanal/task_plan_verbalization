(define (domain Rover)
(:requirements :typing :durative-actions)
(:types rover waypoint store camera mode lander objective)

(:predicates 
  ; The rover ?x is at the waypoint ?y
  ; verb = be
  ; subject = ?x
  ; prep = at ?y
  (at ?x - rover ?y - waypoint) 

  ; The lander ?x is at the waypoint ?y
  ; verb = be
  ; subject = ?x
  ; prep = at ?y
  (at_lander ?x - lander ?y - waypoint)
  
  ; The rover ?r can traverse waypoint ?x to ?y
  ; verb = can (traverse)
  ; subject = ?r
  ; prep = from ?x
  ; prep = to ?y
  (can_traverse ?r - rover ?x - waypoint ?y - waypoint)

	; The rover ?r is equipped for soil analysis
  ; verb = be (equipped for soil analysis)
  ; subject = ?r
  (equipped_for_soil_analysis ?r - rover)

  ; The rover ?r is equipped for rock analysis
  ; verb = be (equipped for rock analysis)
  ; subject = ?r
  (equipped_for_rock_analysis ?r - rover)

  ; The rover ?r is equipped for image analysis
  ; verb = be (equipped for image analysis)
  ; subject = ?r
  (equipped_for_imaging ?r - rover)

  ; The store ?s is empty
  ; verb = be (empty)
  ; subject = ?s
  (empty ?s - store)

  ; The rover ?r has a rock analysis at waypoint ?w
  ; verb = have (rock analysis)
  ; subject = ?r
  ; prep = at ?w
  (have_rock_analysis ?r - rover ?w - waypoint)

  ; The rover ?r has a soil analysis at waypoint ?w
  ; verb = have (soil analysis)
  ; subject = ?r
  ; prep = at ?w
  (have_soil_analysis ?r - rover ?w - waypoint)

  ; The store ?s is full
  ; verb = be (full)
  ; subject = ?s
  (full ?s - store)

  ; Canera ?c is calibrated for rover ?r
  ; verb = be (calibrated)
  ; subject = ?c
  ; prep = for ?r
	(calibrated ?c - camera ?r - rover) 

  ; Camera ?c supports mode ?m
  ; verb = support
  ; subject = ?c
  ; direct-object = ?m
	(supports ?c - camera ?m - mode)
  
  ; Rover ?r is available
  ; verb = be (available)
  ; subject = ?r
  (available ?r - rover)

  ; The waypoint ?w is visible from waypoint ?p
  ; verb = be (visible)
  ; subject = ?w
  ; prep = from ?p
  (visible ?w - waypoint ?p - waypoint)
             
  ; Rover ?r has image of objective ?o in mode ?m
  ; verb = have
  ; subject = ?r
  ; direct-object = image
  ; prep = of ?o
  ; prep = in ?m mode
  (have_image ?r - rover ?o - objective ?m - mode)

  ; Soil data has been communicated at waypoint ?w
  ; verb = be (communicated)
  ; subject = soil data
  ; prep = at ?w
  (communicated_soil_data ?w - waypoint)

  ; Rock data has been communicated at waypoint ?w
  ; verb = be (communicated)
  ; subject = rock data
  ; prep = at ?w
  (communicated_rock_data ?w - waypoint)

  ; Image data of objective ?o has been communicated at waypoint ?w
  ; verb = be (communicated)
  ; subject = image data
  ; prep = of ?o
  ; prep = in ?m mode
  (communicated_image_data ?o - objective ?m - mode)

  ; Soil sample is at waypoint ?w
  ; verb = be
  ; subject = soil sample
  ; prep = at ?w
  (at_soil_sample ?w - waypoint)
  
  ; Rock sample is at waypoint ?w
  ; verb = be
  ; subject = rock sample
  ; prep = at ?w
  (at_rock_sample ?w - waypoint)

  ; Objective ?o is visible from waypoint ?w
  ; verb = be
  ; subject = ?o
  ; prep = from ?w
  (visible_from ?o - objective ?w - waypoint)

  ; Store ?s is of rover ?r
  ; verb = be (the store)
  ; subject = ?s
  ; prep = of ?r
  (store_of ?s - store ?r - rover)

  ; Soil sample is at waypoint ?w
  ; verb = be
  ; subject = soil sample
  ; prep = at ?w
  (calibration_target ?i - camera ?o - objective)

  ; Camera ?i is on board of rover ?r
  ; verb = be (on board)
  ; subject = ?i
  ; prep = of ?r
  (on_board ?i - camera ?r - rover)

  ; Channel ?l is free
  ; verb = be (free)
  ; subject = ?l
  (channel_free ?l - lander)

)

	
; Moves the rover ?x from ?y waypoint to the waypoint ?z
; verb = go / travel / move
; subject = ?x
; prep = from ?y
; prep = to ?z / towards ?z !
(:durative-action navigate
:parameters (?x - rover ?y - waypoint ?z - waypoint) 
:duration (= ?duration 5)
:condition (and (over all (can_traverse ?x ?y ?z)) (at start (available ?x)) (at start (at ?x ?y)) 
                (over all (visible ?y ?z))
	    )
:effect (and (at start (not (at ?x ?y))) (at end (at ?x ?z))
		)
)

; Samples soil 
; verb = sample
; direct-object = soil
; subject = ?x
; prep = to store at ?s
; prep = at ?p
(:durative-action sample_soil
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 10)
:condition (and (over all (at ?x ?p)) (at start (at ?x ?p)) (at start (at_soil_sample ?p)) (at start (equipped_for_soil_analysis ?x)) (at start (store_of ?s ?x)) (at start (empty ?s))
		)
:effect (and (at start (not (empty ?s))) (at end (full ?s)) (at end (have_soil_analysis ?x ?p)) (at end (not (at_soil_sample ?p)))
		)
)

; Samples rock 
; verb = sample
; direct-object = rock
; subject = ?x
; prep = to store at ?s
; prep = at ?p
(:durative-action sample_rock
:parameters (?x - rover ?s - store ?p - waypoint)
:duration (= ?duration 8)
:condition (and (over all (at ?x ?p)) (at start (at ?x ?p)) (at start (at_rock_sample ?p)) (at start (equipped_for_rock_analysis ?x)) (at start (store_of ?s ?x)) (at start (empty ?s))
		)
:effect (and (at start (not (empty ?s))) (at end (full ?s)) (at end (have_rock_analysis ?x ?p))  (at end (not (at_rock_sample ?p)))
		)
)

; Rover ?x drops something at store 
; verb = drop / leave 
; subject = ?x
; direct-object = the load
; prep = at store ?y
(:durative-action drop
:parameters (?x - rover ?y - store)
:duration (= ?duration 1)
:condition (and (at start (store_of ?y ?x)) (at start (full ?y))
		)
:effect (and (at end (not (full ?y))) (at end (empty ?y))
	)
)

; Rover ?r calibrates camera ?t for objective ?t at waypoint ?w 
; verb = calibrate 
; subject = ?r
; direct-object = ?i
; prep = for ?t !
; prep = at ?w
(:durative-action calibrate
 :parameters (?r - rover ?i - camera ?t - objective ?w - waypoint)
 :duration (= ?duration 5)
 :condition (and (at start (equipped_for_imaging ?r)) (at start (calibration_target ?i ?t)) (over all (at ?r ?w)) (at start (visible_from ?t ?w)) (at start (on_board ?i ?r))
		)
 :effect (at end (calibrated ?i ?r)) 
)



; Rover ?r takes an image 
; verb = take / capture 
; subject = ?r
; direct-object = an image
; prep = of ?o !
; prep = with ?i !
; prep = in ?m mode
; prep = at ?p
(:durative-action take_image
 :parameters (?r - rover ?p - waypoint ?o - objective ?i - camera ?m - mode)
 :duration (= ?duration 7)
 :condition (and (over all (calibrated ?i ?r))
			 (at start (on_board ?i ?r))
                      (over all (equipped_for_imaging ?r))
                      (over all (supports ?i ?m) )
			  (over all (visible_from ?o ?p))
                      (over all (at ?r ?p))
               )
 :effect (and (at end (have_image ?r ?o ?m)) (at end (not (calibrated ?i ?r)))
		)
)

; Communicate soil data
; verb = communicate / transmit
; subject = ?r
; direct-object = soil data
; prep = of ?p !
; prep = from ?x
; prep = to ?l lander !
; prep = at ?y
(:durative-action communicate_soil_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (at ?r ?x)) (over all (at_lander ?l ?y)) (at start (have_soil_analysis ?r ?p)) 
                   (at start (visible ?x ?y)) (at start (available ?r))(at start (channel_free ?l))
            )
 :effect (and (at start (not (available ?r))) (at start (not (channel_free ?l))) (at end (channel_free ?l))
		(at end (communicated_soil_data ?p))(at end (available ?r))
	)
)

; Communicate rock data
; verb = communicate / transmit
; subject = ?r
; direct-object = rock data
; prep = of ?p !
; prep = from ?x
; prep = to ?l lander !
; prep = at ?y
(:durative-action communicate_rock_data
 :parameters (?r - rover ?l - lander ?p - waypoint ?x - waypoint ?y - waypoint)
 :duration (= ?duration 10)
 :condition (and (over all (at ?r ?x)) (over all (at_lander ?l ?y)) (at start (have_rock_analysis ?r ?p)) 
                   (at start (visible ?x ?y)) (at start (available ?r))(at start (channel_free ?l))
            )
 :effect (and (at start (not (available ?r))) (at start (not (channel_free ?l))) (at end (channel_free ?l))(at end (communicated_rock_data ?p))(at end (available ?r))
          )
)

; Communicate image data
; verb = communicate / transmit
; subject = ?r
; direct-object = image data
; prep = of ?o !
; prep = in ?m mode
; prep = from ?x
; prep = to ?l lander !
; prep = at ?y
(:durative-action communicate_image_data
 :parameters (?r - rover ?l - lander ?o - objective ?m - mode ?x - waypoint ?y - waypoint)
 :duration (= ?duration 15)
 :condition (and (over all (at ?r ?x)) (over all (at_lander ?l ?y)) (at start (have_image ?r ?o ?m)) 
                   (at start (visible ?x ?y)) (at start (available ?r)) (at start (channel_free ?l))
            )
 :effect (and (at start (not (available ?r))) (at start (not (channel_free ?l))) (at end (channel_free ?l)) (at end (communicated_image_data ?o ?m)) (at end (available ?r))
          )
)

)
