#************************** MDIS4 device descriptor *************************
#
#        Author: ag
#         $Date: 2002/07/18 12:05:17 $
#     $Revision: 1.1 $
#
#   Description: Metadescriptor for M77
#
#****************************************************************************

M77_1  {
	#------------------------------------------------------------------------
	#	general parameters (don't modify)
	#------------------------------------------------------------------------
    DESC_TYPE        = U_INT32  1          # descriptor type (1=device)
    HW_TYPE          = STRING   M077       # hardware name of device

	#------------------------------------------------------------------------
	#	reference to base board
	#------------------------------------------------------------------------
    BOARD_NAME       = STRING   D201_1      # device name of baseboard
    DEVICE_SLOT      = U_INT32  1           # used slot on baseboard (0..n)

	#------------------------------------------------------------------------
	#	debug levels (optional)
	#   this keys have only effect on debug drivers
	#------------------------------------------------------------------------
    DEBUG_LEVEL      = U_INT32  0xc0008000  #  debug level for driver

	#------------------------------------------------------------------------
	#	device parameters
	#------------------------------------------------------------------------
}

