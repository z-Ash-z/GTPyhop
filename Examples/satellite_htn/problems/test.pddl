(define (problem strips-sat-x-1)
(:domain satellite)
(:objects
	satellite0 - satellite
	instrument0 - instrument
	instrument1 - instrument
	instrument2 - instrument
	satellite1 - satellite
	instrument3 - instrument
	instrument4 - instrument
	instrument5 - instrument
	satellite2 - satellite
	instrument6 - instrument
	instrument7 - instrument
	thermograph0 - mode
	image1 - mode
	Star2 - direction
	Star1 - direction
	GroundStation0 - direction
	Planet3 - direction
	Planet4 - direction
)
(:init
	(supports instrument0 image1)
	(calibration_target instrument0 Star2)
	(supports instrument1 image1)
	(supports instrument1 thermograph0)
	(calibration_target instrument1 Star1)
	(supports instrument2 thermograph0)
	(supports instrument2 image1)
	(calibration_target instrument2 GroundStation0)
	(on_board instrument0 satellite0)
	(on_board instrument1 satellite0)
	(on_board instrument2 satellite0)
	(power_avail satellite0)
	(pointing satellite0 Planet3)
	(= (data_capacity satellite0) 1000)
	(= (fuel satellite0) 140)
	(supports instrument3 thermograph0)
	(supports instrument3 image1)
	(calibration_target instrument3 Star1)
	(supports instrument4 thermograph0)
	(supports instrument4 image1)
	(calibration_target instrument4 GroundStation0)
	(supports instrument5 image1)
	(supports instrument5 thermograph0)
	(calibration_target instrument5 GroundStation0)
	(on_board instrument3 satellite1)
	(on_board instrument4 satellite1)
	(on_board instrument5 satellite1)
	(power_avail satellite1)
	(pointing satellite1 Planet4)
	(= (data_capacity satellite1) 1000)
	(= (fuel satellite1) 138)
	(supports instrument6 thermograph0)
	(supports instrument6 image1)
	(calibration_target instrument6 GroundStation0)
	(supports instrument7 image1)
	(supports instrument7 thermograph0)
	(calibration_target instrument7 GroundStation0)
	(on_board instrument6 satellite2)
	(on_board instrument7 satellite2)
	(power_avail satellite2)
	(pointing satellite2 GroundStation0)
	(= (data_capacity satellite2) 1000)
	(= (fuel satellite2) 159)
	(= (data Planet3 thermograph0) 73)
	(= (data Planet4 thermograph0) 158)
	(= (data Planet3 image1) 42)
	(= (data Planet4 image1) 149)
	(= (slew_time Star2 GroundStation0) 43.3)
	(= (slew_time GroundStation0 Star2) 43.3)
	(= (slew_time Star2 Star1) 27.62)
	(= (slew_time Star1 Star2) 27.62)
	(= (slew_time Star1 GroundStation0) 11.32)
	(= (slew_time GroundStation0 Star1) 11.32)
	(= (slew_time Planet3 GroundStation0) 26.41)
	(= (slew_time GroundStation0 Planet3) 26.41)
	(= (slew_time Planet3 Star1) 43.88)
	(= (slew_time Star1 Planet3) 43.88)
	(= (slew_time Planet3 Star2) 28.05)
	(= (slew_time Star2 Planet3) 28.05)
	(= (slew_time Planet4 GroundStation0) 64.75)
	(= (slew_time GroundStation0 Planet4) 64.75)
	(= (slew_time Planet4 Star1) 27.12)
	(= (slew_time Star1 Planet4) 27.12)
	(= (slew_time Planet4 Star2) 89.01)
	(= (slew_time Star2 Planet4) 89.01)
	(= (slew_time Planet4 Planet3) 29.47)
	(= (slew_time Planet3 Planet4) 29.47)
	(= (data-stored) 0)
	(= (fuel-used) 0)
)
(:goal (and
	(have_image Planet3 thermograph0)
	(have_image Planet4 thermograph0)
))
(:metric minimize (fuel-used))

)

