# ===========================================================================
# 
# HomingProject1.tcl --
# 
# ===========================================================================

wm protocol . WM_DELETE_WINDOW quit

# suggestion for a quit procedure

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

    exit
}

# suggestion for initialization stuff:

set hostname "localhost"

#
# video device parameters
#
set device   "/dev/video0" 
set port     0
set width 320
set height 240


set videoHandle [vid openVideo $hostname $device $port $width $height 1]

pio connect $hostname
pio autoSonarOn

# insert your code after this comment


