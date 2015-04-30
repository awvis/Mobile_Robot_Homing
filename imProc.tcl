#PART 1
proc process {imageName outputName} {
	#puts $imageName
	ip image RED
	ip image GREEN
	ip image BLUE	
	ip loadPPM RED GREEN BLUE "/homes/vsubramaniansekar/projects/original/$imageName.ppm"

	ip image GRAY
	ip rgb2bw RED GREEN BLUE GRAY

	ip image input

	ip image MAPPEDNN
	set w [ip width GRAY]
	set h [ip height GRAY]
	set xc 376
	set yc 283
	set r1 86
	set r2 263

	ip polarMapping POLAR $w $h $xc $yc $r1 $r2 b
	ip mapping GRAY POLAR MAPPEDNN
	
	ip image SCALED
	ip rescaleMag MAPPEDNN 720 90 0 SCALED

	ip image $outputName
	ip butterworth SCALED 0.25 0.0 1 $outputName
}

proc homeVector {vlist} {
	global PI v1 v2
	set v1 0
		set v2 0
		foreach v $vlist {
			set pSSX [lindex $v 0]
			set pSSY [lindex $v 1]
			set pCVX [lindex $v 2]
			set pCVY [lindex $v 3]
			set thetaX [expr ((2*$PI)/720) * $pSSX]
			set deltaX [expr ((2*$PI)/720) * ($pCVX - $pSSX)]
			set thetaY [expr (0.7/90) * $pSSY]
			set deltaY [expr (0.7/90) * ($pCVY - $pSSY)]

			if {$thetaY>=0.01} {
				set sdeltaX [expr sin($deltaX)]
				set cdeltaX [expr cos($deltaX)]
				set tthetaY [expr tan($thetaY)]
				set tdeltaY [expr tan($thetaY+$deltaY)]
				set ttdeltaY [expr $tdeltaY/$tthetaY]

				set beta [expr $thetaX - atan2($sdeltaX,($ttdeltaY-$cdeltaX))+$PI]
				set v1 [expr $v1+cos($beta)]
				set v2 [expr $v2+sin($beta)]
			}
		}
		set x [expr 1/($v1*$v1+$v2*$v2)]
		set x [expr sqrt($x)]
		set v1 [expr $v1*$x]
		set v2 [expr $v2*$x]
}

proc angularError {homeVectorList ssPosX ssPosY} {
	foreach elem $homeVectorList {
		set idX ssPosX
	}
}

set ssX 0
set ssY 1

set xSmall 80
set ySmall 80 

ip image FINAL [expr 17*$xSmall] [expr 10*$ySmall]
ip image SCALED
ip image FILTERED1
process $ssX\_$ssY FILTERED1

for {set i 0} {$i < 10} {incr i} {
	for {set j 0} {$j < 17} {incr j} {

		ip image FILTERED2
		process $i\_$j FILTERED2


		set vlist [ip blockMatching FILTERED1 FILTERED2 0 0 144 18 5 5 5 5 10 10 89]
		
		ip image SCALED
		ip rescaleMag FILTERED2 $xSmall $ySmall 0 SCALED
		ip insert FINAL SCALED [expr $j*$xSmall] [expr $i*$ySmall] FINAL

		set v1 0
		set v2 0
		homeVector $vlist
		set x1 [expr $j*$xSmall+$xSmall/2]
		set y1 [expr $i*$ySmall+$ySmall/2]
		set x2 [expr 40*$v1+$x1]
		set y2 [expr 40*$v2+$y1]
		ip drawLine FINAL $x1 $y1 $x2 $y2 3 1

		if {$ssX == $j && $ssY == $i} {
			set x1 [expr $j*$xSmall]
			set y1 [expr $i*$ySmall]
			set x2 [expr ($j+1)*$xSmall]
			set y2 [expr ($i+1)*$ySmall]
			ip drawBox FINAL $x1 $y1 $x2 $y2 3 1
		}
		
		ip showBWImage input FINAL
	}
}

