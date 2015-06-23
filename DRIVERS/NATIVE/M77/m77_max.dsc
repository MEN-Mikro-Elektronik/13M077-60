#************************** MDIS4 device descriptor *************************
#
#        Author: ag
#         $Date: 2002/07/18 12:05:16 $
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
    ID_CHECK         = U_INT32  1             # check module ID prom

    CHANNEL_0 {
            SIO_HW_OPTS      = U_INT32  0x0e  # HW options 8N1
            FIOBAUDRATE      = U_INT32  9600  # 1152000 baudrate
            DRV_RX_BUFF_SIZE = U_INT32   512  # driver internal Rx buffer size
            DRV_TX_BUFF_SIZE = U_INT32   512  # driver internal Tx buffer size
            TX_FIFO_LEVEL    = U_INT32    16  # TX FIFO INT trigger level
            RX_FIFO_LEVEL    = U_INT32    16  # RX FIFO INT trigger level
            NO_FIFO          = U_INT32     0  # disable FIFO support
            PHYS_INT         = U_INT32  0x07  # line dirver mode RS232 etc.
            ECHO_SUPPRESS    = U_INT32     1  # echo suppression in HD mode
    }

    CHANNEL_1 {
            SIO_HW_OPTS      = U_INT32  0x0e  # HW options 8N1
            FIOBAUDRATE      = U_INT32  9600  # 1152000 baudrate
            DRV_RX_BUFF_SIZE = U_INT32   512  # driver internal Rx buffer size
            DRV_TX_BUFF_SIZE = U_INT32   512  # driver internal Tx buffer size
            TX_FIFO_LEVEL    = U_INT32    16  # TX FIFO INT trigger level
            RX_FIFO_LEVEL    = U_INT32    16  # RX FIFO INT trigger level
            NO_FIFO          = U_INT32     0  # disable FIFO support
            PHYS_INT         = U_INT32  0x07  # line dirver mode RS232 etc.
            ECHO_SUPPRESS    = U_INT32     1  # echo suppression in HD mode
    }

    CHANNEL_2 {
            SIO_HW_OPTS      = U_INT32  0x0e  # HW options 8N1
            FIOBAUDRATE      = U_INT32  9600  # baudrate
            DRV_RX_BUFF_SIZE = U_INT32   512  # driver internal Rx buffer size
            DRV_TX_BUFF_SIZE = U_INT32   512  # driver internal Tx buffer size
            TX_FIFO_LEVEL    = U_INT32    16  # TX FIFO INT trigger level
            RX_FIFO_LEVEL    = U_INT32    16  # RX FIFO INT trigger level
            NO_FIFO          = U_INT32     0  # disable FIFO support
            PHYS_INT         = U_INT32  0x07  # line dirver mode RS232 etc.
            ECHO_SUPPRESS    = U_INT32     1  # echo suppression in HD mode
    }

    CHANNEL_3 {
            SIO_HW_OPTS      = U_INT32  0x0e  # HW options 8N1
            FIOBAUDRATE      = U_INT32  9600  # baudrate
            DRV_RX_BUFF_SIZE = U_INT32   512  # driver internal Rx buffer size
            DRV_TX_BUFF_SIZE = U_INT32   512  # driver internal Tx buffer size
            TX_FIFO_LEVEL    = U_INT32    16  # TX FIFO INT trigger level
            RX_FIFO_LEVEL    = U_INT32    16  # RX FIFO INT trigger level
            NO_FIFO          = U_INT32     0  # disable FIFO support
            PHYS_INT         = U_INT32  0x07  # line dirver mode RS232 etc.
            ECHO_SUPPRESS    = U_INT32     1  # echo suppression in HD mode
    }
}

