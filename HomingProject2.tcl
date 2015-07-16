# ===========================================================================
# 
# HomingProject2.tcl --
# 
# ===========================================================================

wm protocol . WM_DELETE_WINDOW quit
#
#steering functions
#
proc forward {} {
	pio setVel 200
	puts "forward"
}
proc turnLeft {} {
	pio setVel2 -200 200
	puts "left"
}
proc turnRight {} {
	pio setVel2 200 -200
	puts "right"
}
proc backward {} {
	pio setVel -200
	puts "back"
}
proc stop {} {
	pio stop
}
#
# Snapshot Function
#
proc takeSnapshot {videoHandle snapshot} {
	puts "return"
	vid startGrab $videoHandle
	while {![vid checkGrab]} {
		update
		after 100	
	}
	ip image red
	ip image green
	ip image blue
	vid fetchGrab red green blue

	update
	imageProcessing red green blue snapshot
}

proc imageProcessing {red green blue snapshot} {
	
	#convert to black/white
	ip image GRAY
	ip rgb2bw red green blue GRAY

	#set inner and outer circle for unwrapping
	ip image MAPPEDNN
	set w [ip width GRAY]
	set h [ip height GRAY]
	set xc 95
	set yc 67
	set r1 14
	set r2 61

	#unwrapping panoramic image
	ip polarMapping POLAR $w $h $xc $yc $r1 $r2 b

	ip mapping GRAY POLAR MAPPEDNN

	#apply butterworth-filter
	ip image $snapshot
	ip butterworth MAPPEDNN 0.15 0.0 1 $snapshot
	
}

proc homeVector {flowVectorList} {
	global PI sizeX sizeY horizon
	set hvX 0
	set hvY 0
	#calculate the homevector for each flowvector
	foreach v $flowVectorList {
		lassign $v pSSX pSSY pCVX pCVY
		set thetaX [expr ((2*$PI)/$sizeX) * $pSSX]
		set deltaX [expr ((2*$PI)/$sizeX) * ($pCVX - $pSSX)]
		set thetaY [expr (0.7/$sizeY) * ($horizon - $pSSY)]
		set deltaY [expr -((0.7/$sizeY) * ($pCVY - $pSSY))]

		if {[expr abs($thetaY)]>=0.01} {
			set sdeltaX [expr sin($deltaX)]
			set cdeltaX [expr cos($deltaX)]
			set tdeltaY [expr tan($thetaY+$deltaY)]
			set ttdeltaY [expr $tdeltaY/$thetaY]

			set beta [expr ($thetaX - atan2($sdeltaX,($ttdeltaY-$cdeltaX))) + $PI]
			#add up all homevectors calculated from flow vectors
			set hvX [expr $hvX+cos($beta)]
			set hvY [expr $hvY+sin($beta)]
		}
	}
	#normalize summed homevector
	set norm [expr 1/(($hvX*$hvX)+($hvY*$hvY))]
	set norm [expr sqrt($norm)]
	set hvX [expr $hvX*$norm]
	set hvY [expr -($hvY*$norm)]
	return [list $hvX $hvY]
}

proc homing {videoHandle currentView} {
	global snapshotOrientation PI horizon sizeX sizeY

	ip showBWImage input snapshot
	set hvXprev 0
	set hvYprev 0
	lassign [pio getXY] xLast yLast
	set started 0

	while {1} {
		vid startGrab $videoHandle
		ip image red
		ip image green
		ip image blue
		vid fetchGrab red green blue
		update
		imageProcessing red green blue currentView

		ip image compass
		#1=manhatten 2=euclidian
		ip shiftedDistH snapshot currentView 1 compass

		set min 0
		set imageCompassPixel 0
		set y 0
		lassign [ip minPixel compass] min imageCompassPixel y
		lassign [ip maxPixel compass] max

		set imageCompassRadians [expr $imageCompassPixel * (2*$PI/$sizeX)]

		set odometricCompassDegree [pio getTheta]
		set odometricCompassPixel [expr int(round(($odometricCompassDegree-$snapshotOrientation) * ($sizeX/(360.0))))]

		set sumWeighted 0
		set sum 0
		for {set i 0} {$i < $sizeX} {incr i} {
    			ip image help
			ip extract compass help $i 0 1 1

			lassign [ip minPixel help] locmin locx
			set xlocRadians [expr $i*(2*$PI/$sizeX)]
			if {$locmin < ($min+(0.2*($max-$min)))} {
				set sum [expr $sum+1.0]
				set sumWeighted [expr $sumWeighted+abs(angleDiffPi($imageCompassRadians,$xlocRadians))]
			}
		}
		set imageCompassQuality 0
		if {$sum != 0} {
			set imageCompassQuality [expr $sumWeighted/$sum]
		}
		if {$imageCompassQuality < 0.1} {
			#set compassAdjust -$odometricCompassPixel
			set compassAdjust $imageCompassPixel
		} else {
			set compassAdjust -$odometricCompassPixel
			#set compassAdjust $imageCompassPixel
		}		
						
		#rotate currentview according to compass
		puts "$imageCompassPixel -$odometricCompassPixel"
		ip extract currentView currentView [expr -$compassAdjust] 0 384 48


		ip showBWImage input2 currentView
		
		set avg 0		
		lassign [ip average compass] avg

		#calculate flowvectors from blockmatching
if {1} {		
set vlist [ip blockMatching snapshot currentView 0 5 38 4 10 10 5 5 30 30 $horizon]
} else {
set wx 5
set wy 5
set rx 15
set ry 10
set x0 0
set y0 5
set yh [expr $sizeY - $y0]
set dx 5
set dy 5
set nx [expr round(double($sizeX) / $dx)]
set ny [expr round(double($yh - $y0) / $dy)]
set vlist [ip blockMatching snapshot currentView \
                   $x0 $y0 $nx $ny $dx $dy $wx $wy $rx $ry $horizon]
}


		#calculate homevector for cv
		set hvX 0
		set hvY 0
		lassign [homeVector $vlist] hvX hvY

		puts "[expr atan2($hvY,$hvX)] [expr atan2($hvYprev,$hvXprev)]"

		#check if current Homevector is orthogonal to old Homevector
		if {([expr cos(atan2($hvY,$hvX)-atan2($hvYprev,$hvXprev))] < 0.0) && $started} {
			lassign [pio getXY] x2 y2
			#puts [expr sqrt(($xLast-$x2)*($xLast-$x2)+($yLast-$y2)*($yLast-$y2))]
			#check if distance travelled from last point is small 
			if {hypot($xLast-$x2, $yLast-$y2) < 200} {
				#update odometry according to image
				pio stop
				vid startGrab $videoHandle
		ip image red
		ip image green
		ip image blue
		vid fetchGrab red green blue
		update
		imageProcessing red green blue currentView
ip shiftedDistH snapshot currentView 1 compass

		set min 0
		set imageCompassPixel 0
		set y 0
		lassign [ip minPixel compass] min imageCompassPixel y
		lassign [ip maxPixel compass] max

		set imageCompassRadians [expr $imageCompassPixel * (2*$PI/$sizeX)]

		set odometricCompassDegree [pio getTheta]
		set odometricCompassPixel [expr int(round(($odometricCompassDegree-$snapshotOrientation) * ($sizeX/(360.0))))]
				puts "$odometricCompassDegree [expr $snapshotOrientation - $imageCompassRadians * (180/$PI)]"
				pio setOdometry $x2 $y2 [expr $snapshotOrientation - ($imageCompassRadians * (180/$PI))]
				return
			}
			set xLast $x2
			set yLast $y2
			set hvXprev $hvX
			set hvYprev $hvY
		}

		#drive there
		#rotate homevector to currentview-CS

		set theta [expr ($compassAdjust+45*($sizeX/360.0))] 

		set theta [expr $theta*$PI/180]
		set hvXcur [expr cos($theta)*$hvX-sin($theta)*$hvY]
		set hvYcur [expr sin($theta)*$hvX+cos($theta)*$hvY]
		pio setVel2 [expr $hvXcur*150] [expr $hvYcur*150]

		
		
		if {!$started} {
			set hvXprev $hvX
			set hvYprev $hvY
			set started 1
		}
	}
}

proc homingRoute {} {
	global routeLength snapShotRoute routeOrientation snapshotOrientation videoHandle sizeX sizeY cycle
	
	while {$cycle} {
		ip image snapShotOverView [expr int($sizeX)] [expr int($routeLength*$sizeY)]
		set counter 0
		foreach k $snapShotRoute {
			ip insert snapShotOverView $k 0 [expr int($counter*$sizeY)] snapShotOverView
			set counter [expr $counter+1]
		}
		ip showBWImage snapshots snapShotOverView
		for {set k [llength $snapShotRoute]} {$k > 0} {incr k -1} {
			ip image snapShotBox
			ip copy snapShotOverView snapShotBox
			ip drawBox snapShotBox 0 [expr int(($k-1)*$sizeY)] 383 [expr int(($k)*$sizeY)] 3 1
			ip showBWImage snapshots snapShotBox
			ip copy [lindex $snapShotRoute [expr $k-1]] snapshot
			set snapshotOrientation [lindex $routeOrientation [expr $k-1]]
			ip image currentView
			homing $videoHandle currentView
		}
	}
}

#
#quit procedure
#
proc quit { } {
    #
    # disconnect from video server
    #
    vid disconnect
    #
    # disconnect from pioneer server
    #
    pio autoSonarOff
    pio disconnect

    
#exit
}

# suggestion for initialization stuff:


set hostname "localhost"

#
# video device parameters
#
set device   "/dev/video0" 
set port     0
set width 176
set height 130
set horizon 24
set sizeX 384.0
set sizeY 48.0

set snapshotOrientation 0

set routeLength 0
set snapShotRoute {} 
set routeOrientation {} 

set cycle 0


proc init {} {
	global hostname device port width height videoHandle cycle

	set videoHandle [vid openVideo $hostname $device $port $width $height 1]
	pio connect $hostname /dev/ttyS0
	#pio autoSonarOn


	#image stream and key events
	ip image stream 320 240
	bind all <Key> {
		if {"%K" eq "Up"} {
			forward
		} elseif {"%K" eq "Down"} {
			backward
		} elseif {"%K" eq "Right"} {
			turnRight
		} elseif {"%K" eq "Left"} {
			turnLeft
		} elseif {"%K" eq "Return"} {
			ip image snapshot
			takeSnapshot $videoHandle snapshot
			ip image snapshot$routeLength
			ip copy snapshot snapshot$routeLength
			lappend snapShotRoute snapshot$routeLength
			set routeLength [expr $routeLength+1]
			set snapshotOrientation [pio getTheta]
			lappend routeOrientation [pio getTheta]
		} elseif {"%K" eq "space"} {
			ip image currentView
			homing $videoHandle currentView
		} elseif {"%K" eq "BackSpace"} {
			stop
		} elseif {"%K" eq "r"} {
			homingRoute
		} elseif {"%K" eq "c"} {
			if {$cycle == 0} {
				set cycle 1			
			} else {
				set cycle 0
			}
		}
	}
}
#to be checked
#bind all <KeyRelease> {
	#if {"%K" eq "w"} {
	#	puts "stop"
	#	stop
	#} elseif {"%K" eq "s"} {
	#	stop
	#}
#}
