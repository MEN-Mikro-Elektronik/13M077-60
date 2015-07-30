/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m77_drv.c
 *      Project: M77 VxWorks Driver
 *
 *       Author: ag
 *        $Date: 2013/02/13 18:35:53 $
 *    $Revision: 1.9 $
 *
 *  Description: VxWorks BBIS driver for the M45N/M69N/M77 UART modules
 *
 *               Once the driver is installed four tty devices are
 *               automatically created and ready to use. The driver
 *               routine to call is:
 *
 *               m77Drv (&boardDesc, &moduleDesc) or
 *               m77Drv_sw (&boardDesc, &moduleDesc)
 *
 *               e.g. m77Drv (&D201_1, &M77_1)
 *
 *               All other functions are called via the VXWORKS i/o routines:
 *               open, read, write, close, ioctl
 *
 *               This is not a standard MDIS driver, but it connects to
 *               the BBIS base board interface to support all kinds of buses.
 *
 *               For use of the driver with multiple M077 the driver supports
 *               up to M77_MAX_DEVICES M077 modules in one system.
 *               M77_MAX_DEVICES is defined below.
 *
 *
 *     Required: MDIS, BBIS
 *     Switches:
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m77_drv.c,v $
 * Revision 1.9  2013/02/13 18:35:53  channoyer
 * R: Spurious interrupts with some configuration
 * M: Check interrupt request from 0X16C954 before exiting the interrupt handler
 *
 * Revision 1.8  2010/01/22 10:25:26  cs
 * R: MACCESS macros where modified for MPC85xx (compile errors)
 * M: put all MACCESS macros in conditionals into curly braces
 *
 * Revision 1.7  2007/09/25 12:00:46  cs
 * fixed:
 *   - only call m77OxRegDump() when DBG defined (avoid compiler error)
 *
 * Revision 1.6  2006/08/22 09:13:55  cs
 * Added support for M45N/M69N
 * Fixed:
 *     - M77 Descriptors: now matching with xml description and specification
 *     - ECHO_SUPPRESS defaults to 1 if not specified
 *     - PHYS_INT defaults to RS422_FD if not specified
 *
 * Revision 1.5  2005/05/13 14:08:30  AGromann
 * removed problem on A12 with Tornado 2.0.2: System hangs during transmission
 * using Xon Xoff flow control. => Now don't call taskLock() in function
 * m77Startup while in interrupt context.
 * added DBG-Prints
 *
 * Revision 1.4  2004/10/15 14:51:50  ufranke
 * cosmetics
 *
 * Revision 1.3  2003/10/31 11:56:46  Agromann
 * Removed receive problem with ioctl M77_NO_FIFO
 *
 * Revision 1.2  2002/09/09 07:52:32  agromann
 * now initialize lastErr and shadowACR in function m77ChannelRest
 *
 * Revision 1.1  2002/07/18 12:05:11  agromann
 * Initial Revision
 *
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/
#include <vxWorks.h>
#include <iv.h>
#include <ioLib.h>
#include <iosLib.h>
#include <tyLib.h>
#include <intLib.h>
#include <errnoLib.h>
#include <sysLib.h>
#include <vmLib.h>

/* mdis/bbis headers */
#include <MEN/men_typs.h>
#include <MEN/oss.h>
#include <MEN/dbg.h>
#include <MEN/desc.h>
#include <MEN/bb_defs.h>
#include <MEN/bb_entry.h>
#include <MEN/maccess.h>
#include <MEN/mdis_err.h>
#include <MEN/modcom.h>     /* ID PROM functions              */

typedef int LL_HANDLE;

#include <MEN/ll_entry.h>
#include <MEN/os2m.h>
#include <MEN/mk.h>
#include <MEN/bk.h>
#include "tyLib.h"
#include "m77_drv.h"		/* M77 defines + macros */

#include <vxBusLib.h>
#include <hwif/vxbus/vxBus.h>
#include <hwif/vxbus/vxbPciLib.h>
#include <hwif/util/vxbParamSys.h>

/*--------------------------------------+
|   TYPDEFS                             |
+--------------------------------------*/
typedef struct			/* M77_BAUD */
{
    int			rate;					/* baud rate */
    int			preset;					/* counter preset value */
} M77_BAUD;


/* tyCo device descriptors */

typedef struct			/* TY_CO_DEV */
    {
    TY_DEV      tyDev;
    BOOL        created;			/* true if device has already been created */
    int	        rdBufSize;			/* created receive buffer size */
    int         wrtBufSize;			/* created transmitt buffer size */
    u_int8	    ch_num;				/* channel number on uart 0...3 */
    u_int32	    device_num;			/* number of the M77|M45N|M69N module in system */
	MACCESS		ma;					/* offset address of channel */
	UINT        baudrate;           /* baudrate */
	BOOL		echo_suppress;		/* echo suppression in HD-Mode */
	BOOL		fifo_enabled;		/* FIFO is enabled */

	BOOL		auto_RTS;			/* automatic RTS handshaking is enabled */
	BOOL		auto_CTS;			/* automatic CTS handshaking is enabled */
	BOOL		auto_DTR;			/* automatic DTR handshaking is enabled */
	BOOL		auto_DSR;			/* automatic DSR handshaking is enabled */
	BOOL		auto_XONXOFF;		/* automatic Xon/Xoff handshaking is enabled */

	u_int16		transmit_level;		/* transmit trigger level */
	u_int16		receive_level;		/* receive trigger level */
	u_int16		sio_hw_opts;	   	/* UART hardware options */
	u_int16     physInt;            /* settings for physical interface */
	u_int16     hsMode;             /* settings for handshake */
	u_int16     hsInBandMode;       /* settings for in band handshake */
	u_int16     hsHighFifoLevel;    /* flow control upper FIFO level */
	u_int16     hsLowFifoLevel;     /* flow control lower FIFO level */

	u_int16		shadowACR;			/* the current value of ACR */
	u_int8      lastErr;            /* represents LSR value */

    } M77_TY_CO_DEV;


typedef struct {
	/* parameters for context between DriverEntry and Constructor */
    BOOL			created;						/* flag if it is created */

	/* parameters initialized in DriverEntry */
	MACCESS			ma;                             /* offest address of module */
	u_int32			devSlot;						/* device slot */
    u_int32     	modId;							/* M-Module Id of this device */
    u_int32         numChannels;					/* number of SIO channels of module */
	char			brdName[BK_MAX_BBIS_BRD_NAME];	/* base board name */
	char			pModName[11];					/* module name (e.g. m77_1)	*/

	/* parameters initialized in Constructor */
	BBIS_HANDLE		*brdHdl;						/* ptr to bbis board handle */
	BBIS_ENTRY		pBbisEntry;						/* ptr to bbis functions */

	BK_BASE_BOARD_LIST_ELEMENT  *bbListElement;     /* handle to base board list element */
    DESC_SPEC       *bbDescSpec;					/* base board desc spec */
    u_int32         idCheck;                        /* check M-module ID */
} MODSTAT;


/*-----------------------------------------+
|  DEFINES & CONST                         |
+------------------------------------------*/

/* defines for M-Module ID check */
#define MOD_ID_MAGIC		0x5346      /* eeprom identification (magic) */
#define MOD_ID_N_OFFSET		32000		/* offset of module id for MxxxN boards */
#define MOD_ID_M45N			(45 + MOD_ID_N_OFFSET)	/* module id M45N */
#define MOD_ID_M69N			(69 + MOD_ID_N_OFFSET)	/* module id M69N */
#define MOD_ID_M77			77						/* module id M77 */


/* m-module modes needed because MDIS not available for M77 */
/* m-module address modes */
#define MM_MA_NONE			0x0000
#define MM_MA08				0x0001
#define MM_MA24				0x0002
#define MM_MA_PCICFG		0x0100

/* m-module data access modes */
#define MM_MD_NONE			0x0000
#define MM_MD08				0x0004
#define MM_MD16				0x0008
#define MM_MD32				0x0010

#define M77_ADDRESS_MODE	MM_MA08
#define M77_DATA_MODE		MM_MD16

#define IRQ_DISABLE				0
#define IRQ_ENABLE				1

#define M77_MAX_DEV_CHANNEL		8

#define M77_MAX_DEVICES			8

#define M77_DEFAULT_BAUD		9600

#define M77_DEF_TX_BUFF_SIZE	512
#define M77_DEF_RX_BUFF_SIZE	512
#define M77_DEF_SIO_HW_OPTS		0x0e
#define M77_FIFO_SIZE			128
#define TX_FIFO_LEVEL_DEFAULT   16
#define RX_FIFO_LEVEL_DEFAULT   16

#define M77_DRV_BUFF_OVERFLOW   0x08

#ifdef DBG
#define DBH				    M77_DbgHdl
#define DBG_MYLEVEL			m77DgbLev
#endif /* DBG */

#undef M77_DBG

#ifndef M77_SW
#	define M77_DEVICE_NAME     "/m77_"
#	define M45N_DEVICE_NAME    "/m45n_"
#	define M69N_DEVICE_NAME    "/m69n_"
#else
#	define M77_DEVICE_NAME     "/m77sw_"
#	define M45N_DEVICE_NAME    "/m45n_sw_"
#	define M69N_DEVICE_NAME    "/m69n_sw_"
#endif

#define M77_INVALID_BAUD    0xf0

/* baud rate table for M77 */
static M77_BAUD m77BaudTable [] =
    {
    {300, 3840}, {600, 1920}, {1200, 960}, {2400, 480},
    {4800, 240}, {9600, 120}, {14400, 80}, {19200, 60},
    {28800, 40}, {38400, 30}, {57600, 20}, {115200, 10},
    {230400, 5}, {576000, 2}, {1152000, 1}
    };

/*-----------------------------------------+
|  GLOBALS                                 |
+------------------------------------------*/
static u_int32          m77DrvNum[M77_MAX_DEVICES];	/* driver number assigned to this driver */

static M77_TY_CO_DEV    m_tyCoDv [M77_MAX_DEVICES][M77_MAX_DEV_CHANNEL];	/* M-module tty device descriptors */

static MODSTAT          *m77DevData[M77_MAX_DEVICES];	/* global device data structure, */

static u_int32          m77DgbLev = OSS_DBG_DEFAULT;	/* debug level */

static u_int32	dev_num_m77  = 0;	/* number of the M77  module in system */
static u_int32	dev_num_m45n = 0;	/* number of the M45N module in system */
static u_int32	dev_num_m69n = 0;	/* number of the M69N module in system */

#ifdef DBG
static DBG_HANDLE	    *M77_DbgHdl = NULL;				/* debug handle */
#endif /* DBG */

#ifdef DBG
#ifdef M77_SW
int m77OutBytes_sw = 0;
int m77InBytes_sw = 0;
int m77IntCountIn_sw = 0;
int m77IntCountOut_sw = 0;
int m77IntCountHs_sw = 0;
#else
int m77OutBytes = 0;
int m77InBytes = 0;
int m77IntCountIn = 0;
int m77IntCountOut = 0;
int m77IntCountHs = 0;
#endif

#endif /* DBG */

/*-----------------------------------------+
|  PROTOTYPES                              |
+------------------------------------------*/
static int m77Open (M77_TY_CO_DEV *pChan, char *name, int mode);
static int m77Ioctl (M77_TY_CO_DEV *pChan, int	request, int arg);
static STATUS m77DevCreate ( char *name, int device_num, FAST int channel);
static int32 m77IrqInstall( u_int32 devBusType,
                     u_int32 bbisBusType,
                     u_int32 irqVect,
                     u_int32 irqLevel,
                     u_int32 intMode,
                     void    *newIsr,
                     u_int32 device_num );

static STATUS  m77BaudSet
	(
	M77_TY_CO_DEV	*pM77TyDev,	/* pointer to channel */
   	UINT	   		baud		/* requested baud rate */
	);

static STATUS m77OptsSet
    (
    M77_TY_CO_DEV 		*pChan,		/* pointer to channel */
    UINT 				options		/* new hardware options */
    );

static int m77Int(int devNum);

static int m77Startup
    (
    M77_TY_CO_DEV 			*pChan		/* tty device to start up */
    );

static int m77ChanConfig (M77_TY_CO_DEV * pM77TyDev, u_int32 device_num);

static int m77PhysIntSet(M77_TY_CO_DEV *pChan, u_int16 drvMode);

static int m77HSModeSet(M77_TY_CO_DEV *pChan);

static void m77ChannelRest(M77_TY_CO_DEV *pChan);

static UINT16 readDivisor(M77_TY_CO_DEV *pChan);

static void writeDivisor(M77_TY_CO_DEV *pChan, UINT16 divisor);

static u_int16 read650(M77_TY_CO_DEV *pChan, u_int16 offset);

static void write650(M77_TY_CO_DEV *pChan, u_int16 offset, u_int16 value);

static void writeICR(M77_TY_CO_DEV *pChan, u_int16 index, u_int16 value);

#ifdef DBG
static u_int16 readICR(M77_TY_CO_DEV *pChan, u_int16 index);
#endif

static void UnlockAdditionalStatus(M77_TY_CO_DEV *pChan);

static void LockAdditionalStatus(M77_TY_CO_DEV *pChan);

static u_int16 readASRxFL(M77_TY_CO_DEV *pChan, u_int16 offset);

static STATUS setManualRTS(M77_TY_CO_DEV *pChan, int arg);
static STATUS setManualDTR(M77_TY_CO_DEV *pChan, int arg);
static int getManualCTS(M77_TY_CO_DEV *pChan);
static int getManualDSR(M77_TY_CO_DEV *pChan);
#if 0  /* currently not used */
static int getManualDCD(M77_TY_CO_DEV *pChan);
#endif /* currently not used */
static void setAutoCTSEnable(M77_TY_CO_DEV *pChan, BOOL state);
static void setAutoRTSEnable(M77_TY_CO_DEV *pChan, BOOL state);
static void setAutoDSREnable(M77_TY_CO_DEV *pChan, BOOL state);
static void setAutoDTREnable(M77_TY_CO_DEV *pChan, BOOL state);
static void setInBandFlowControlMode(M77_TY_CO_DEV *pChan, u_int8 mode);
static void setM45nTristate(M77_TY_CO_DEV *pChan, BOOL state);

static int32 m77ReadDesc(DESC_SPEC *pDesc, MODSTAT *pModData, int devNum);
static int32 m77ReadChanDesc(DESC_SPEC *pDesc, MODSTAT *pModData, int devNum);

#ifdef DBG
static void m77OxRegDump(M77_TY_CO_DEV *pChan);
#endif

/**************************** M77_GetEntry *********************************
 *
 *  Description:  Dummy function to be conform with MDIS
 *                Gets the entry points of the low-level driver functions.
 *                Fill all with NULL pointer.
 *
 *
 *---------------------------------------------------------------------------
 *  Input......:  ---
 *
 *  Output.....:  drvP  pointer to the initialized structure
 *
 *  Globals....:  ---
 *
 ****************************************************************************/

#ifdef _ONE_NAMESPACE_PER_DRIVER_
    extern void LL_GetEntry( LL_ENTRY* drvP )
#else

#if (defined (M77_SW))
    extern void M77_SW_GetEntry( LL_ENTRY* drvP )
#else
    extern void M77_GetEntry( LL_ENTRY* drvP )
#endif
#endif
{
    drvP->init        = NULL;
    drvP->exit        = NULL;
    drvP->read        = NULL;
    drvP->write       = NULL;
    drvP->blockRead   = NULL;
    drvP->blockWrite  = NULL;
    drvP->setStat     = NULL;
    drvP->getStat     = NULL;
    drvP->irq         = NULL;
    drvP->info        = NULL;
}

/******************************** m77Drv ************************************
 *
 *  Description: initialize the M077 tty driver
 *
 *  This routine initializes the serial driver, sets up interrupt vectors,
 *  and performs hardware initialization of the serial ports.
 *  To support multiple devices the device channels are also created by m77Drv
 *
 *  This routine must be called exactly once for each M77 module in the
 *  system, before any open, read ,write, ioctl.
 *
 *  GLOBAL descriptor keys:
 *
 *  Descriptor key        Default          Range
 *  --------------------  ---------------  -------------
 *	DEBUG_LEVEL           OSS_DBG_DEFAULT  see dbg.h
 *  ID_CHECK              1                0..1
 *
 *
 *  Per channel parameters: (must be in directory CHANNEL_x)
 *
 *  Descriptor key        Default          		Range
 *  --------------------  ---------------  		-----------------
 *  SIO_HW_OPTS           M77_DEF_SIO_HW_OPTS   see below
 *  Bit   7   6   5   4  3...2   1     0
 *     +-----------------------------------+
 *     |PODD|PEN|STB|HUP|CSIZE|CREAD|CLOCAL|
 *     *-----------------------------------+
 *              Description         Value
 *     CLOCAL   Not supported
 *     CREAD    Receiver Enable       0 = receiver disabled
 *                                    1 = receiver enabled (default)
 *     CSIZE    Set Word Length       0 0 = 5 bits
 *                                    0 1 = 6 bits
 *                                    1 0 = 7 bits
 *                                    1 1 = 8 bits (default)
 *     HUP      Not supported
 *     STB      Number of Stop Bits   0 = 1 stop bit (default)
 *                                    1 = 2 stop bits
 *     PEN      Parity Enable Bit     0 = parity disabled (default)
 *                                    1 = parity enabled
 *     PODD     Parity ODD Enable Bit 0 = even parity (default)
 *
 *  FIOBAUDRATE           M77_DEFAULT_BAUD      see below
 *    The following values for FIOBAUDRATE are allowed:
 *    300, 600, 1200, 2400, 4800, 9600,  14400, 19200, 28800, 38400,
 *    57600, 115200, 230400, 576000, 1152000
 *    Note: For RS232/RS423 the max. value is 115200
 *
 *  DRV_RX_BUFF_SIZE      M77_DEF_RX_BUFF_SIZE  0...max
 *  DRV_TX_BUFF_SIZE      M77_DEF_TX_BUFF_SIZE  0...max
 *  TX_FIFO_LEVEL         TX_FIFO_LEVEL_DEFAULT 0...127
 *  RX_FIFO_LEVEL         RX_FIFO_LEVEL_DEFAULT 1...127
 *  NO_FIFO               0                     1: FIFO disabeld 0: FIFO enabled
 *  PHYS_INT              M77_RS422_FD          M77_RS423 | M77_RS422_HD |
 *                                              M77_RS422_FD |M77_RS485_HD |
 *                                              M77_RS485_FD | M77_RS485_FD |
 *                                              M77_RS232 (defined in m77_drv.h)
 *  ECHO_SUPPRESS         1                     0: no echo suppression
 *                                              1: echo suppression enabled
 *  HANDSHAKE_XONXOFF     0                     0: no software handshake
 *                                              1: software handhake enabled
 *  HANDSHAKE_AUTO_CTS    0                     0: no automatic CTS handshaking
 *                                              1: auto CTS handshaking enabled
 *  HANDSHAKE_AUTO_RTS    0                     0: no automatic RTS handshaking
 *                                              1: auto RTS handshaking enabled
 *  HANDSHAKE_AUTO_DSR    0                     0: no automatic DSR handshaking
 *                                              1: auto DSR handshaking enabled
 *  HANDSHAKE_AUTO_DTR    0                     0: no automatic DTR handshaking
 *                                              1: auto DTS handshaking enabled
 *  HANDSHAKE_HIGH_FIFO_LEVEL   M77_DEF_FCH     1...127
 *  HANDSHAKE_LOW_FIFO_LEVEL    M77_DEF_FCL     1...127
 *
 *----------------------------------------------------------------------------
 *  Input......:  bbDescSpec       carrier board descriptor (e.g. &D201_1)
 *                m77DescSpec      M77 descriptor (e.g. &M77_1)
 *
 *  Output.....:  Return 		OK | ERROR
 *
 *  Globals....:  m77DrvNum, m_tyCoDv, m77DevData,
 *
 *****************************************************************************/
STATUS m77Drv
(
	DESC_SPEC *bbDescSpec,
	DESC_SPEC *m77DescSpec
)
{

	int32		retCode=0;
	u_int32		i;

	int			channel;
	int			device_num = 0;
	u_int32		gotSize;
	u_int32		irqVect;
	u_int32		irqLevel;
	u_int32		intMode;
	u_int32		busType;
	u_int32		devBusType;
	void		*physAddr;
	char		tyName [20];
    int32		modIdMagic;
    int32		modId;

	u_int32		availSize;
    BK_BASE_BOARD_LIST_ELEMENT *bbListElement;

	DBGINIT(( NULL, &M77_DbgHdl ));

	DBGWRT_1((DBH,"m77Drv:"));

	/*----------------+
	| check arguments |
	+----------------*/

	if ( bbDescSpec == NULL || m77DescSpec == NULL )
	{
		#ifdef M77_DBG
			fprintf(stderr, "*** Basebord or module descriptor not valid !\n");
		#endif
		DBGWRT_ERR((DBH,"*** m77Drv: Basebord or module descriptor not valid !\n"));
		return ERROR;
	}

    /*----------------------------+
    | select device descriptor    |
    | to support multiple devices |
    +----------------------------*/
    for ( i = 0; i < M77_MAX_DEVICES; i++ )	/* maximum of M77_MAX_DEVICES devices */
	{
		if ( m77DevData[i] == NULL )
		{
			device_num = i;
			break;
		}
	}

	#ifdef M77_DBG
		fprintf (stderr, "m77Drv: device_num= 0x%i\n", device_num);
	#endif

	DBGWRT_2 ((DBH, "m77Drv: device_num= 0x%i\n", device_num));

	/*--------------------+
	| allocate memory for |
	| hardware descriptor |
	+--------------------*/
	m77DevData[device_num] = (MODSTAT*)OSS_MemGet( OSS_VXWORKS_OS_HDL,
												   sizeof(MODSTAT),
												   &gotSize );

    if( m77DevData[device_num] == NULL )
    {
        #ifdef M77_DBG
        	fprintf (stderr, "*** m77Drv: memory allocation failed\n");
        #endif
        DBGWRT_ERR((DBH,"*** m77Drv: Memory allocation for m77DevData failed\n"));
        return( ERROR );
    }/*if*/

    /* fill the turkey with zero */
    OSS_MemFill(OSS_VXWORKS_OS_HDL, gotSize, (char*) m77DevData[device_num], 0 );

	m77DevData[device_num]->created = FALSE;

	/*---------------------------------------+
	| save module data in driver structure   |
	+---------------------------------------*/
	m77DevData[device_num]->bbDescSpec = bbDescSpec;
	retCode = m77ReadDesc(m77DescSpec, m77DevData[device_num], device_num);

	if ( retCode ) {
		#ifdef M77_DBG
			fprintf (stderr, " *** m77Drv: m77ReadDesc failed retCode = 0x%08lx\n",
			retCode);
		#endif
		DBGWRT_ERR( ( DBH, "m77Drv: m77ReadDesc failed\n"));

		goto CLEANUP;
	}/*if*/

	/*-----------------------+
	| get BBIS function list |
	+-----------------------*/
	retCode = BK_GetBBHandle(  bbDescSpec,
                               &m77DevData[device_num]->brdName[0],
                               &bbListElement);

	if ( retCode ) {
		#ifdef M77_DBG
			fprintf (stderr, "*** m77Drv: BBIS Handle failed\n");
		#endif
	    DBGWRT_ERR( ( DBH, "m77Drv: BK_GetBBHandle failed\n"));

 	    goto CLEANUP;
	}/*if*/

	m77DevData[device_num]->bbListElement = bbListElement;
	m77DevData[device_num]->brdHdl = bbListElement->bbData->bbHdlData;
	m77DevData[device_num]->pBbisEntry = bbListElement->bbData->bbHdlEntrys;

	/*------------------------------------+
	| get interrupt description from BBIS |
	| intVector, intLevel, intMode        |
	+------------------------------------*/
	retCode = m77DevData[device_num]->pBbisEntry.cfgInfo(	m77DevData[device_num]->brdHdl,
                                       				        BBIS_CFGINFO_IRQ,
                                                			m77DevData[device_num]->devSlot,
                                                			&irqVect,
                                                			&irqLevel,
   	                                            			&intMode );

    if( retCode != 0 )
    {
    	#ifdef M77_DBG
    		fprintf (stderr, "*** m77Drv: cfg failed\n");
    	#endif
    	DBGWRT_ERR( ( DBH, "m77Drv: BBIS cfgInfo failed"));

        goto CLEANUP;
    }

    /*-----------------------+
    | get bus type from BBIS |
    | PCI, VME, onBoard      |
    +-----------------------*/
    retCode = m77DevData[device_num]->pBbisEntry.brdInfo( BBIS_BRDINFO_BUSTYPE, &busType );

    if( retCode != 0 )
    {
        #ifdef M77_DBG
    		fprintf (stderr, "*** m77Drv: brdInfo failed\n");
    	#endif
        DBGWRT_ERR( ( DBH, "m77Drv: BBIS brdInfo failed" ));

        goto CLEANUP;
    }


    /*------------------------------+
    | get device bus type from BBIS |
    +------------------------------*/
    retCode = m77DevData[device_num]->pBbisEntry.brdInfo(	BBIS_BRDINFO_DEVBUSTYPE,
        													m77DevData[device_num]->devSlot,
        													&devBusType );

    if( retCode != 0 ) {
    	/* workaround for BBIS not supporting BBIS_BRDINFO_DEVBUSTYPE */
    	if( retCode ==  ERR_BBIS_UNK_CODE)	/* ERR_BBIS_UNK_CODE (A201) */
			devBusType = OSS_BUSTYPE_MMODULE;
        else {
	    	#ifdef M77_DBG
    			fprintf (stderr, "*** m77Drv: brdInfo2 failed\n");
    		#endif
	    	DBGWRT_ERR( ( DBH, "m77Drv: BBIS brdInfo failed" ));

        	goto CLEANUP;
	    } /*if*/
    }/*if*/

    /*-------------------------------------+
    | set M-Module interface on base board |
    | by BBIS function                     |
    +-------------------------------------*/
    retCode = m77DevData[device_num]->pBbisEntry.setMIface( m77DevData[device_num]->brdHdl,
                                                			m77DevData[device_num]->devSlot,
                                                			M77_ADDRESS_MODE,
                                                			M77_DATA_MODE );

    if( retCode != 0 )
    {
    	#ifdef M77_DBG
    		fprintf (stderr, "*** m77Drv: setMIface failed\n");
    	#endif
    	DBGWRT_ERR( ( DBH, "m77Drv: BBIS setMIface failed" ));
        goto CLEANUP;
    }/*if*/


	/*--------------------------+
    | get addr spaces from BBIS |
    +--------------------------*/
    retCode = m77DevData[device_num]->pBbisEntry.getMAddr(	m77DevData[device_num]->brdHdl,
                                                			m77DevData[device_num]->devSlot,
                                                			M77_ADDRESS_MODE,
                                                			M77_DATA_MODE,
                                                			&physAddr,
                                                			&availSize );

    if( retCode != 0 )
    {
        #ifdef M77_DBG
    		fprintf (stderr, "*** m77Drv: getMAddr failed\n");
    	#endif
        DBGWRT_ERR( ( DBH, "m77DRv: BBIS getMAddr failed" ));

        goto CLEANUP;
    }/*if*/


	DBGWRT_2 ((DBH, "m77Drv: getMAddr: physical address = 0x%08lx\n", physAddr));

	/* assign base address of module */
	m77DevData[device_num]->ma = (MACCESS)(physAddr);

	/*--------------------------------+
	| check module Id				  |
	+--------------------------------*/
	if (m77DevData[device_num]->idCheck) {
		DBGWRT_3((DBH,"m77Drv: Checking module ID\n"));
		modIdMagic = m_read((u_int32)m77DevData[device_num]->ma, 0);
		modId      = m_read((u_int32)m77DevData[device_num]->ma, 1);
		DBGWRT_3((DBH,"m77Drv: module id: magic=0x%04x, id=%d\n",
					  modIdMagic, modId));
		if( modIdMagic != MOD_ID_MAGIC ) {
			DBGWRT_ERR((DBH,"*** m77Drv: illegal magic=0x%04x\n", modIdMagic));
			goto CLEANUP;
		}

		if( modId != MOD_ID_M77 &&
			modId != MOD_ID_M45N &&
			modId != MOD_ID_M69N ) {
			DBGWRT_ERR( (DBH,"*** m77Drv: illegal id=%d\n", modId));
			goto CLEANUP;
		}
		m77DevData[device_num]->modId = modId;

	} else {
		m77DevData[device_num]->modId = MOD_ID_M77;
	}

	/*----------------------------+
	| set M-Module specific info: |
	| channels, devName           |
	| save in driver structure    |
	+----------------------------*/
	switch ( m77DevData[device_num]->modId ){
	case MOD_ID_M45N:
		sprintf( m77DevData[device_num]->pModName, "%s%d",
				 M45N_DEVICE_NAME, (int)dev_num_m45n++);
		m77DevData[device_num]->numChannels = 8;
		break;
	case MOD_ID_M69N:
		sprintf( m77DevData[device_num]->pModName, "%s%d",
				 M69N_DEVICE_NAME, (int)dev_num_m69n++);
		m77DevData[device_num]->numChannels = 4;
		break;
	default:
		sprintf( m77DevData[device_num]->pModName, "%s%d",
				 M77_DEVICE_NAME, (int)dev_num_m77++);
		m77DevData[device_num]->numChannels = 4;
		break;
	}

	/*--------------------------------------------------------+
	| save channel specific module data in driver structure   |
	+--------------------------------------------------------*/
	retCode = m77ReadChanDesc(m77DescSpec, m77DevData[device_num], device_num);

	if ( retCode ) {
		#ifdef M77_DBG
			fprintf (stderr, " *** m77Drv: m77ReadChanDesc failed retCode = 0x%08lx\n",
			retCode);
		#endif
		DBGWRT_ERR( ( DBH, "m77Drv: m77ReadChanDesc failed\n"));

		goto CLEANUP;
	}/*if*/

	/*-------------------------+
	| call VxWorks function to |
	| install serial driver    |
	+-------------------------*/
	#ifdef M77_DBG
    	fprintf (stderr, "m77Drv: iosDrvInstall\n");
    #endif
    DBGWRT_2( ( DBH, "m77Drv: iosDrvInstall\n" ));

	m77DrvNum[device_num] = iosDrvInstall (m77Open, (FUNCPTR) 0, m77Open,
				(FUNCPTR) 0, tyRead, tyWrite, m77Ioctl);

	if (m77DrvNum[device_num] == ERROR)
	{
		#ifdef M77_DBG
    		fprintf (stderr, "*** m77Drv: iosDrvInstall failed\n");
    	#endif
		DBGWRT_ERR( ( DBH, "m77Drv: iosDrvInstall failed" ));
		goto CLEANUP;
	}
	m77DevData[device_num]->created = TRUE;

	#ifdef M77_DBG
    	fprintf (stderr, "m77Drv: m77DevCreate\n");
    #endif
    DBGWRT_2( ( DBH, "m77Drv: m77DevCreate\n" ));

	/*---------------------------+
	| call m77DevCreate          |
	| creates all channels of    |
	| the device                 |
	+---------------------------*/
	for ( channel = 0; channel < m77DevData[device_num]->numChannels; channel++ )
	{
		sprintf (tyName, "%s%s%d", m77DevData[device_num]->pModName, "/", channel);
		if ( ERROR == m77DevCreate ( tyName, device_num, channel)) {
			#ifdef M77_DBG
	    		fprintf (stderr, "m77Drv: m77DevCreate failed dev_num = %d, channel = %d\n"
								 "Please check descriptor values !\n", device_num, channel);
	    	#endif

			DBGWRT_ERR( ( DBH, "m77Drv: m77DevCreate failed dev_num = %d, channel = %d\n"
							   "Please check descriptor values !\n", device_num, channel));
			goto CLEANUP;
		}
	}

	#ifdef M77_DBG
    	fprintf (stderr, "m77Drv: m77IrqInstall\n");
    #endif
    DBGWRT_2( ( DBH, "m77Drv: m77IrqInstall: irqLevel = %d irqVect = %d\n"
    , irqLevel, irqVect));

	/*--------------------------+
    | installation of           |
    | interrupt service routine |
    | in BBIS                   |
    +--------------------------*/
    retCode = m77IrqInstall(    devBusType,
       							busType,
                           		irqVect,
                                irqLevel,
                                intMode,
                                m77Int,
                                device_num
                           );

    if ( retCode != 0 )
    {
    	DBGWRT_ERR( ( DBH, "m77Drv: m77IrqInstall failed\n" ));

        goto CLEANUP;
    }/*if*/



	#ifdef M77_DBG
    	fprintf (stderr, "m77Drv: pBbisEntry.irqEnable\n");
    #endif
    DBGWRT_2( ( DBH, "m77Drv: pBbisEntry.irqEnable\n" ));

    /*--------------------------------+
    | enable board interrupts by BBIS |
    +--------------------------------*/
    retCode = m77DevData[device_num]->pBbisEntry.irqEnable(	m77DevData[device_num]->brdHdl,
                   											m77DevData[device_num]->devSlot,
                   											IRQ_ENABLE );

    if ( retCode != 0 )
    {
    	DBGWRT_ERR( ( DBH, "m77Drv: BBIS irqEnable failed\n" ));

        goto CLEANUP;
    }/*if*/

    /*--------------------------------------------------+
    |    enable Interrupt and TX line drivers on M77    |
    +--------------------------------------------------*/
	MWRITE_D16 (m77DevData[device_num]->ma, M77_IRQ_REG, (M77_IRQ_EN | M77_TX_EN));

	if( m77DevData[device_num]->modId == MOD_ID_M45N ) {
		MWRITE_D16( m77DevData[device_num]->ma,
					M77_IRQ_REG + M45N_CTRL_REG_BLOCK_SIZE,
					M77_IRQ_EN );
	}

	#ifdef M77_DBG
    	fprintf (stderr, "m77Drv: Done\n");
    #endif
    DBGWRT_1( ( DBH, "m77Drv: Done\n" ));

    return OK;


CLEANUP:
    /*----------------------+
    | error - clean up      |
    +----------------------*/
    #ifdef M77_DBG
    	fprintf (stderr, "*** m77Drv aborted\n");
    #endif
    DBGWRT_ERR( ( DBH, "m77Drv: installation aborted\n" ));
    if( retCode )
    {
    	if( bbListElement )
			BK_UnGetBBHandle( &bbListElement );
        return(retCode);         /* return error */
    }/*if*/

	DBGEXIT((&DBH));

	/* remove driver if created */
	if (m77DevData[device_num]->created == TRUE)
		iosDrvRemove ( m77DrvNum[device_num], 1 );

	/* free m77DevData structure */
	OSS_MemFree(OSS_VXWORKS_OS_HDL, (void*) m77DevData[device_num], gotSize);

    return ERROR;

}

/******************************** m77DevCreate ******************************
 *
 *  Description: create a device for an M77 serial port
 *
 *  This routine creates a device for a specified M77 serial port.  Each port
 *  to be used should have exactly one device associated with it by calling
 *  this routine.
 *  Unlike the standard tty drivers this routine is directly called by m77Drv
 *  to manage the ability to support multiple M77 modules
 *
 *
 *----------------------------------------------------------------------------
 *  Input......:  name			device name (e.g. "/m77/0")
 *                device_num    number of the M77 device
 *                channel       the channel number of the M77/M69N (0...3), M45N: (0..7)
 *
 *  Output.....:  Return 		OK | ERROR
 *
 *  Globals....:
 *****************************************************************************/
static STATUS m77DevCreate
    (
    char 		*name,		/* name to use for this device */
    int			device_num, /* number of M77 device */
    FAST int 	channel  	/* physical channel for this device  */
    )
{

    M77_TY_CO_DEV *pM77TyDev;
    u_int32 controllerNum = 0;


    /*----------------+
    | check arguments |
    +----------------*/
    if (channel > (m77DevData[device_num]->numChannels - 1))
    {
		/* DBG output */
		DBGWRT_ERR( ( DBH, "*** m77DevCreate: channel number not supported" ));
		return (ERROR);
	}

    if (m77DrvNum[device_num] <= 0)
	{
		/* DBG output */
		DBGWRT_ERR( ( DBH, "*** m77DevCreate: driver not installed" ));
		return (ERROR);
	}

    /*----------------------+
    | assign device pointer |
    +----------------------*/
	pM77TyDev = &m_tyCoDv[device_num][channel];

    /* if this device already exists, don't create it */
    if (pM77TyDev->created)
    {
    	DBGWRT_ERR( ( DBH, "*** m77DevCreate: channel already created" ));
		return (ERROR);
    }

    /*--------------------------------+
    |    assign device structure      |
    +--------------------------------*/

    /* assign channel accesss address  */
	if( (m77DevData[device_num]->modId == MOD_ID_M45N) && (channel > 3) )
		controllerNum = 1;

	pM77TyDev->ma = (MACCESS) ( (u_int32)m77DevData[device_num]->ma +
								(controllerNum * M45N_CTRL_REG_BLOCK_SIZE) +
								((channel - controllerNum * 4) * 0x10) );
	DBGWRT_2 ((DBH, "m77DevCreate: pM77TyDev->ma = 0x%08lx\n", pM77TyDev->ma));


	/* set channel number in TyCo structure */
	pM77TyDev->ch_num = (u_int8) channel;

	/* introduce driver to VxWorks I/O system */
    if (tyDevInit ( &pM77TyDev->tyDev, pM77TyDev->rdBufSize, pM77TyDev->wrtBufSize,
    				(FUNCPTR)m77Startup) != OK)
	{
		DBGWRT_ERR( ( DBH, "*** m77DevCreate: tyDevInit failed" ));
		return (ERROR);
	}

	DBGWRT_2(( DBH, "m77DevCreate: tyDevInit: RX-Buff size = %d TX-Buff size = %d\n",
				pM77TyDev->rdBufSize, pM77TyDev->wrtBufSize));

	/*--------------------+
    | init single channel |
    +--------------------*/
    /* save channel enumeration in TyCo structure */
    pM77TyDev->device_num = device_num;

	/* make channel configuration */
	if (ERROR == m77ChanConfig( pM77TyDev, pM77TyDev->device_num)) {
		return ERROR;
	}

	/* set flow control levels */
	pM77TyDev->hsHighFifoLevel = M77_DEF_FCH;
	pM77TyDev->hsLowFifoLevel  = M77_DEF_FCL;

	if( m77DevData[device_num]->modId != MOD_ID_M77 &&
		m77HSModeSet(pM77TyDev) != OK )
	{
		return ERROR;
	}

#ifdef DBG
	m77OxRegDump(pM77TyDev);
#endif /* DBG */

    /* mark the device as created, and add the device to the I/O system */
    pM77TyDev->created = TRUE;
    return (iosDevAdd (&pM77TyDev->tyDev.devHdr, name, m77DrvNum[device_num]));
    }


/******************************** m77ChanConfig ******************************
 *
 *  Description: Set M77 channel to default state got from descriptor
 *
 *  Set baudrate, change to enhanced mode, enable / disable FIFOs
 *  clear FIFOS, set Rx/Tx trigger levels, set HW options and  enable
 *  interrupts on oxford chip.
 *
 *------------------------------------------------------------------------------
 *  Input......:  pChan         pointer to channel device
 *                device_num    number of M77 module in system
 *
 *  Output.....:  Return 		OK or ERROR
 *
 *  Globals....:
 *******************************************************************************/
static int m77ChanConfig (M77_TY_CO_DEV * pM77TyDev, u_int32 device_num)
{
	u_int16 efr;
	int oldLevel;
	u_int16 lcr;
	STATUS retStat = OK;

	/*-----------------------+
	|   reset this channel   |
	+-----------------------*/
	m77ChannelRest(pM77TyDev);

	/*-------------------------+
	|  set physical interface  |
	+-------------------------*/
	if( m77DevData[device_num]->modId == MOD_ID_M77 ) { /* only M77 supports RS485 HD */
		writeICR (pM77TyDev, M77_ACR_OFFSET, pM77TyDev->shadowACR | M77_ACR_DTR_HD_H_DRIVER_CTL);
		if (ERROR == m77PhysIntSet(pM77TyDev, pM77TyDev->physInt)) {
			DBGWRT_ERR( ( DBH, "*** m77ChanConfig: illegal pyhsical interface DevNo: %d CH: %d\n",
			device_num, pM77TyDev->ch_num));
			return ERROR;
		}
	}

    /*----------------------------------+
    | set baudrate to descriptor value  |
    +----------------------------------*/
    retStat = m77BaudSet (pM77TyDev, pM77TyDev->baudrate);
    if (retStat != OK) {
    	if(retStat == M77_INVALID_BAUD) {
			DBGWRT_ERR( ( DBH, "*** m77ChanConfig: illegal baudrate DevNo: %d CH: %d BAUD: %d\n",
						device_num, pM77TyDev->ch_num, pM77TyDev->baudrate));
		} else {
			DBGWRT_ERR( ( DBH, "*** m77ChanConfig: error setting baudrate DevNo: %d CH: %d errNo: 0x%08x\n",
						device_num, pM77TyDev->ch_num, retStat));
		}
		return ERROR;
    }

    /*----------------------+
    | reset actual channel  |
    | with standard options |
    +----------------------*/

	/* change to enhanced (16C950) mode */
    taskLock();
    oldLevel = intLock();

	efr = read650(pM77TyDev, M77_EFR_OFFSET);
	write650(pM77TyDev, M77_EFR_OFFSET, (u_int16)(efr | M77_EFR_ENHANCED_MODE));

	if (pM77TyDev->fifo_enabled) {
		/* enable FIFOs, because clear command (see below) requires FIFOs to be enabled */
		MWRITE_D16(pM77TyDev->ma, M77_ISR_FCR_REG, M77_FIFO_ENABLE);
		/* set trigger leves and enable 950 interrupt trigger levels */
		writeICR (pM77TyDev, M77_TTL_OFFSET, pM77TyDev->transmit_level);
		writeICR (pM77TyDev, M77_RTL_OFFSET, pM77TyDev->receive_level);
		writeICR (pM77TyDev, M77_ACR_OFFSET, (pM77TyDev->shadowACR | M77_ACR_950_TRIG_EN));
		/* clear FIFOs */
	    MWRITE_D16(pM77TyDev->ma, M77_ISR_FCR_REG, (FCR_DMA_MODE | RxCLEAR | TxCLEAR | FIFO_ENABLE));

	}
	else {

	    /* switch to DMA mode 1 FIFO disabled */
	    MWRITE_D16(pM77TyDev->ma, M77_ISR_FCR_REG, FCR_DMA_MODE);
		/* clear the port */
		MREAD_D16(pM77TyDev->ma, M77_HOLD_DLL_REG);
	}

	intUnlock (oldLevel);
	taskUnlock();

	/* set hardware options */
    switch (pM77TyDev->sio_hw_opts & CSIZE) {
		case CS5:
			lcr = M77_LCR_CS5;
			break;
		case CS6:
			lcr = M77_LCR_CS6;
			break;
		case CS7:
			lcr = M77_LCR_CS7;
			break;
		default:
		case CS8:
			lcr = M77_LCR_CS8;
			break;
	}

    if (pM77TyDev->sio_hw_opts & STOPB)
		lcr |= M77_LCR_2_STB;
    else
		lcr |= M77_LCR_1_STB;

    switch (pM77TyDev->sio_hw_opts & (PARENB | PARODD)) {
		case PARENB|PARODD:
			lcr |= M77_LCR_PEN;
			break;
		case PARENB:
			lcr |= (M77_LCR_PEN | M77_LCR_EPS);
			break;
		default:
		case 0:
			lcr |= M77_LCR_PDIS; break;
	}
    MWRITE_D16(pM77TyDev->ma, M77_LCR_REG, lcr);


	/* unmask Rx & RX-Error interrupts on Oxford chip */
	if (pM77TyDev->sio_hw_opts & CREAD) {
		MSETMASK_D16( pM77TyDev->ma, M77_IER_DLM_REG, (M77_IER_RXRDY | M77_IER_LSI));
	} else {
		MCLRMASK_D16( pM77TyDev->ma, M77_IER_DLM_REG, M77_IER_RXRDY);
	}

	/* enable interrupts for current channel on Oxford-Chip */
   	MSETMASK_D16(pM77TyDev->ma, M77_MCR_REG, M77_MCR_INT_EN);

	return OK;
}

/************************** m77IrqInstall *************************************
 *
 *  Description:  Installs the M77 interrupt service routine in system.
 *                Adapted from MDIS MK_IrqInstall
 *
 *---------------------------------------------------------------------------
 *  Input......:  devBusType  bustype OSS_BUSTYPE _NONE | _VME | _PCI | _ISA | _ISAPNP
 *                bbisBusType bustype OSS_BUSTYPE _NONE | _VME | _PCI | _ISA | _ISAPNP
 *                irqVect     interrupt number
 *                irqLevel    interrupt level
 *                intMode     shared or exclusive
 *                newIsr      new interrrupt service routine
 *                device_num  argument for interrupt service routine
 *
 *  Output.....:  return    0 | error code
 *
 *  Globals....:  ---
 *
 ****************************************************************************/
static int32 m77IrqInstall( u_int32 devBusType,
                     u_int32 bbisBusType,
                     u_int32 irqVect,
                     u_int32 irqLevel,
                     u_int32 intMode,
                     void    *newIsr,
                     u_int32 device_num )

{

    STATUS	vxRetCode = 0;
    int32   retCode = 0;
    /* pointer to IRQ connect routine */
	STATUS (*M77_intConnectRtn)
	(VOIDFUNCPTR *vector, VOIDFUNCPTR routine, int parameter );
	/* pointer to VME IRQ enable routine */
	STATUS (*M77_vmeIntEnableRtn)(int level);
	/* pointer to IRQ enable routine */
	STATUS (*M77_intEnableRtn)(int level);

    M77_intConnectRtn   = MK_GetIntConnectRtn();
    M77_vmeIntEnableRtn = MK_GetVmeIntEnableRtn();
    M77_intEnableRtn    = MK_GetIntEnableRtn();

#if 0
    /* install the new isr */
    vxRetCode = (*M77_intConnectRtn) ((VOIDFUNCPTR*)  INUM_TO_IVEC(irqVect),
                                      (VOIDFUNCPTR)   newIsr,
                                      (int)           device_num );
    if( vxRetCode != OK )
	{
    	DBGWRT_ERR((DBH, " *** m77IrqInstall: Error calling M77_intConnectRtn\n"));
    	retCode = ERR_MK_IRQ_INSTALL;
    	goto IRQ_INST_ERR;
	}/*if*/
#else
		/* use vxbIntConnect( ), the VXB_INTR_DYNAMIC is configured in the BSP */
		{
			VXB_DEVICE_ID devID = vxbInstByNameFind("menPciCham",0);
			if( devID == NULL )
				goto IRQ_INST_ERR;

			DBGWRT_1((DBH, "vxbIntConnect connecting 0x%x, 0x%x\n", newIsr, device_num));
		    if ((vxbIntConnect (devID, 0, (VOIDFUNCPTR)newIsr, (void*)device_num)) != OK)
		    	DBGWRT_1((DBH, "vxbIntConnect returned ERROR\n"));
		    DBGWRT_1((DBH, "vxbIntConnect returned OK\n"));
		}
#endif

	if( bbisBusType == OSS_BUSTYPE_VME ) {
        /* enables VMEbus interrupts */
        if (M77_vmeIntEnableRtn) {
            if (ERROR == (*M77_vmeIntEnableRtn) ( irqLevel )) {
	        	DBGWRT_ERR((DBH, " *** m77IrqInstall: ERROR calling M77_vmeIntEnableRtn\n"
	        	"Please make sure that right function was assigned using "
	        	"MK_SetIntEnableRtn\n"));
	        	retCode = ERR_MK_IRQ_ENABLE;
	        	goto IRQ_INST_ERR;
	        }
	    }
	    else {
        	DBGWRT_ERR((DBH, " *** m77IrqInstall: Warning: No VME interrupt "
								  "enable function installed. Call "
								  "MK_SetVmeIntEnableRtn()\n"));
        	retCode = ERR_MK_IRQ_ENABLE;
        	goto IRQ_INST_ERR;

	    }

	} /* if */
	if( ((devBusType == OSS_BUSTYPE_ISA) ||
		 (devBusType == OSS_BUSTYPE_ISAPNP) ||
		 (devBusType == OSS_BUSTYPE_PCI ) ||
		 (bbisBusType == OSS_BUSTYPE_PCI ))){
        /* enables other interrupts */
        if (M77_intEnableRtn) {
            if (ERROR == (*M77_intEnableRtn) ( irqLevel )) {
	        	DBGWRT_ERR((DBH, " *** m77IrqInstall: ERROR calling M77_intEnableRtn\n"
	        	"Please make sure that right function was assigned using "
	        	"MK_SetIntEnableRtn \n"));
	        	retCode = ERR_MK_IRQ_ENABLE;
	        	goto IRQ_INST_ERR;
	        }
		}
		else {
        	DBGWRT_ERR((DBH, " *** m77IrqInstall: Warning: No interrupt "
								  "enable function installed. Call "
								  "MK_SetIntEnableRtn)\n"));
        	retCode = ERR_MK_IRQ_ENABLE;
        	goto IRQ_INST_ERR;
		}
	}/*if*/

IRQ_INST_ERR:

    return( retCode );
}/*m77IrqInst*/


/******************************** m77Open *************************************
 *
 *  Description: open a file to the channel
 *
 *  This routine opens a file to the channel and gives back the file
 *  descriptor to the system. Called by tty function open.
 *
 *
 *----------------------------------------------------------------------------
 *  Input......:  m77TyDev      device tty structure
 *                name          the name of the device (not used)
 *                mode          the mode (not used)
 *
 *  Output.....:  Return 		pointer to m77TyDev
 *
 *  Globals....:
 *****************************************************************************/
static int m77Open
    (
    M77_TY_CO_DEV *pChan,
    char *name,
    int mode)

{
    FAST int     oldlevel;		/* current interrupt level mask */

	DBGWRT_2((DBH, "m77Open: pChan->ma = 0x%08lx\n",
			  pChan->ma));

	oldlevel = intLock ();
	setManualRTS(pChan, 1);
	setManualDTR(pChan, 1);
	intUnlock (oldlevel);

	/* flush any characters which were received during configuration */
	tyIoctl (&pChan->tyDev, FIOFLUSH, 0);

	return ((int) pChan);

}


/******************************** m77BaudSet *********************************
 *
 *  Description: Set the baud rate for the channel
 *               The interrupts are disabled during chip access
 *
 *----------------------------------------------------------------------------
 *  Input......:  pM77TyDev         pointer to channel device
 *                baud          requested baud rate
 *
 *  Output.....:  Return 		OK | ERROR
 *
 *  Globals....:
 *****************************************************************************/
static STATUS  m77BaudSet
	(
	M77_TY_CO_DEV	*pM77TyDev,	/* pointer to channel */
   	UINT	   		baud		/* requested baud rate */
   	)
	{
    int   	oldlevel;
    STATUS	status;
    FAST	int	ix;

    status = M77_INVALID_BAUD;

    if( pM77TyDev->physInt == M77_RS232 &&
    	baud > 115200 )
	{
		DBGWRT_ERR((DBH, "m77BaudSet: This baudrate (%d) is not supported for "
						 "RS232\n", baud));
		goto EXIT_END;
	}

    for (ix = 0; ix < NELEMENTS (m77BaudTable); ix++)
	{
		if (m77BaudTable [ix].rate == baud)	/* lookup baud rate value */
	    {

		    /* disable interrupts during chip access */
		    if (ERROR == taskLock())
		    	return ERROR;
		    oldlevel = intLock ();

	    	writeDivisor(pM77TyDev, (u_int16) m77BaudTable[ix].preset);

		    intUnlock(oldlevel);
		    if (ERROR == taskUnlock())
		    	return ERROR;

    		/* store baudrate in device structure */
    		pM77TyDev->baudrate = baud;
			status = OK;
			break;
		}
	}
EXIT_END:
    return (status);
	}

/******************************** m77OptsSet ***********************************
 *
 *  Description: Set the channel serial options
 *
 *               Supported options:
 *               CREAD, CSIZE, PARENB, PARODD
 *               Not supported options:
 *               CLOCAL, HUPCL
 *
 *------------------------------------------------------------------------------
 *  Input......:  pChan         pointer to channel device
 *                options       new hardware options to set
 *
 *  Output.....:  Return 		OK | ERROR
 *
 *  Globals....:
 *******************************************************************************/
static STATUS m77OptsSet
    (
    M77_TY_CO_DEV 		*pChan,		/* pointer to channel */
    UINT 				options		/* new hardware options */
    )
    {
    FAST int     oldlevel;		/* current interrupt level mask */
    u_int16 mcr = MREAD_D16( pChan->ma, M77_MCR_REG );
    u_int16 lcr = 0;
    u_int16 ier;

    if (pChan == NULL || options & 0xffffff00)
	return ERROR;

	DBGWRT_2((DBH, "m77OptsSet: pChan->ma = 0x%08lx options = 0x%08x\n",
			  pChan->ma, options));

    switch (options & CSIZE)
	{
	case CS5:
	    lcr = M77_LCR_CS5; break;
	case CS6:
	    lcr = M77_LCR_CS6; break;
	case CS7:
	    lcr = M77_LCR_CS7; break;
	default:
	case CS8:
	    lcr = M77_LCR_CS8; break;
	}

    if (options & STOPB)
	lcr |= M77_LCR_2_STB;
    else
	lcr |= M77_LCR_1_STB;

    switch (options & (PARENB | PARODD))
	{
	case PARENB|PARODD:
	    lcr |= M77_LCR_PEN; break;
	case PARENB:
	    lcr |= (M77_LCR_PEN | M77_LCR_EPS); break;
	default:
	case 0:
	    lcr |= M77_LCR_PDIS; break;
	}

    if (ERROR == taskLock())
    	return ERROR;
    oldlevel = intLock ();

    /* now clear the FIFO/Port */
    if (pChan->fifo_enabled) {
    	MWRITE_D16(pChan->ma, M77_ISR_FCR_REG, (FCR_DMA_MODE | RxCLEAR | TxCLEAR | FIFO_ENABLE));
    }
   	MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);

	/* set Handshake handling */
 	ier = MREAD_D16(pChan->ma, M77_IER_DLM_REG);
 	mcr &= ~(M77_MCR_DTR | M77_MCR_RTS);

    if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 )
 	{
        ier &= ~M77_IER_MSI; /* disable modem status interrupt */
		ier |= M77_IER_TBE;
	} else {
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M45N ) {
			if( pChan->ch_num <= 1 )
				mcr |= M77_MCR_DTR | M77_MCR_RTS;
			else
				mcr |= M77_MCR_DTR;
		} else if( m77DevData[pChan->device_num]->modId == MOD_ID_M69N ) {
			if( pChan->ch_num <= 1 )
				mcr |= M77_MCR_DTR;
			else
				mcr |= M77_MCR_DTR | M77_MCR_RTS;
		}

		ier |= M77_IER_MSI;    /* enable modem status interrupt */

		/* handshaking enabled and CTS line is asserted: enable Tx interrupt */
		if( pChan->auto_CTS ) {
			u_int16 msr = MREAD_D16(pChan->ma, M77_MSR_REG);
			if( msr & M77_MSR_CTS )
				ier |= M77_IER_TBE;
			else
				ier &= (~M77_IER_TBE);
		}
	}

    MWRITE_D16(pChan->ma, M77_LCR_REG, lcr);
    MWRITE_D16(pChan->ma, M77_MCR_REG, mcr);

    if (options & CREAD)
		ier |= M77_IER_RXRDY;
	else
		ier &= ~M77_IER_RXRDY;

    MWRITE_D16(pChan->ma, M77_IER_DLM_REG, ier);

    intUnlock (oldlevel);
    if (ERROR == taskUnlock())
    	return ERROR;


    pChan->sio_hw_opts = (u_int16) options;

    return OK;
}


/******************************** m77Ioctl ***********************************
 *
 *  Description: Special device control
 *
 *               The following M77 specific commands are included:
 *               M77_ECHO_SUPPRESS
 *               M77_TX_FIFO_LEVEL
 *               M77_RX_FIFO_LEVEL
 *               M77_NO_FIFO
 *               M77_PHYS_INT_SET
 *               M77_HS_HIGH_FIFO_LEVEL
 *               M77_HS_LOW_FIFO_LEVEL
 *               M77_MODEM_AUTO_RTS
 *               M77_MODEM_AUTO_CTS
 *               M77_MODEM_AUTO_DSR
 *               M77_MODEM_AUTO_DTR
 *               M77_MODEM_XONXOFF
 *               M77_GET_LAST_ERR
 *               M77_REG_DUMP
 *               M77_TRISTATE
 *               get/set baud rate, set mode (INT),
 *               hardware options (parity, number of data bits),
 *               handshaking (RTS/CTS/DSR/DTR/XON_XOFF)
 *               M45N set line drivers to tristate
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         pointer to channel device
 *                request       request code
 *                arg           argument for request
 *
 *  Output.....:  Return 		OK | EIO on device error
 *                              | ENOSYS on unsupported request
 *
 *  Globals....:
 *****************************************************************************/
 static int m77Ioctl
    (
    M77_TY_CO_DEV 		*pChan,	 /* device to control */
    int 				request, /* request code */
    int 				arg		 /* some argument */
    )
    {
    int ix;
    int divisor;
    int oldlevel;
    int status = OK;
    int lockKey;
    u_int16 efr;

	DBGWRT_1(( DBH, "m77Ioctl: MOD_ID: %d: Module: %d, Channel: %d, request: 0x%04x, arg: 0x%08x\n",
				m77DevData[pChan->device_num]->modId, pChan->device_num, pChan->ch_num, request, arg));


    switch (request)
	{
	case FIOBAUDRATE: /* not SIO_BAUD_SET, because only used in sio drivers */
    	status = (m77BaudSet(pChan, arg) == OK) ? OK : EIO;
    	break;

    case SIO_BAUD_GET: /* not supported in standard drivers */


        status = EIO;

        if (ERROR == taskLock()) {
        	status = ERROR;
        	goto endIoctrl;
        }
        oldlevel = intLock();

        divisor = readDivisor ( pChan );

        intUnlock(oldlevel);
        if (ERROR == taskUnlock()) {
        	status = ERROR;
        	goto endIoctrl;
		}

        for (ix = 0; ix < NELEMENTS (m77BaudTable); ix++)
        {
            if ( divisor  ==  (m77BaudTable[ix].preset))
            {
                *(int *)arg = m77BaudTable[ix].rate;
                status = OK;
            }
        }

        break;


    case SIO_HW_OPTS_SET: /* not supported in standard drivers */
    	    status = (m77OptsSet (pChan, arg) == OK) ? OK : EIO;
    	    break;

    case SIO_HW_OPTS_GET: /* not supported in standard drivers */
        *(int *)arg = pChan->sio_hw_opts;
        break;

	case SIO_MSTAT_GET:           /* get modem status lines */
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 ) {
			status = EIO;
			break;
		}
		*(int *)arg = 0;

		/* when automatic DSR/CTS is enabled don't read this lines */
		if( pChan->auto_CTS || pChan->auto_DSR ) {
    		DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR SIO_MSTAT_GET,"
    						 " auto_CTS/DSR enabled\n"));
       		status = ERROR;
			break;
		}

		/* get input lines */
		if( getManualCTS(pChan) )
			*(int *)arg |= SIO_MODEM_CTS;

		if( ( m77DevData[pChan->device_num]->modId == MOD_ID_M45N &&
			  pChan->ch_num <= 1 ) ||
			( m77DevData[pChan->device_num]->modId == MOD_ID_M69N &&
			  pChan->ch_num > 1 ) )
		{
			if( getManualDSR(pChan) )
				*(int *)arg |= SIO_MODEM_DSR;
		}

		/* get output lines */
		if( MREAD_D16( pChan->ma, M77_MCR_REG ) & M77_MCR_RTS )
			*(int *)arg |= SIO_MODEM_RTS;

		if( MREAD_D16(pChan->ma, M77_MCR_REG) & M77_MCR_DTR )
			*(int *)arg |= SIO_MODEM_DTR;

		break;
	case SIO_MCTRL_BITS_SET:      /* set selected modem lines */
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 ) {
			status = EIO;
			break;
		}

		if( arg & SIO_MODEM_DTR ) {
			if( pChan->auto_DTR ) {
        		DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR SIO_MCTRL_BITS_SET,"
        						 " auto_DTR enabled\n"));
           		status = ERROR;
				break;
			} else
				setManualDTR(pChan, 1);
		}

		if( ( arg & SIO_MODEM_RTS ) &&
			( ( m77DevData[pChan->device_num]->modId == MOD_ID_M45N &&
			    pChan->ch_num <= 1 ) ||
			  ( m77DevData[pChan->device_num]->modId == MOD_ID_M69N &&
			    pChan->ch_num > 1 ) ) )
		{
			if( pChan->auto_RTS ) {
        		DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR SIO_MCTRL_BITS_SET,"
        						 " auto_RTS enabled\n"));
           		status = ERROR;
				break;
			} else
				setManualRTS(pChan, 1);
		}

		break;
	case SIO_MCTRL_BITS_CLR:      /* clear selected modem lines  */
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 ) {
			status = EIO;
			break;
		}

		if( arg & SIO_MODEM_DTR ) {
			if( pChan->auto_DTR ) {
        		DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR SIO_MCTRL_BITS_SET,"
        						 " auto_DTR enabled\n"));
           		status = ERROR;
				break;
			} else
				setManualDTR(pChan, 0);
		}

		if( (arg & SIO_MODEM_RTS) &&
			( ( m77DevData[pChan->device_num]->modId == MOD_ID_M45N &&
			    pChan->ch_num <= 1 ) ||
			  ( m77DevData[pChan->device_num]->modId == MOD_ID_M69N &&
			    pChan->ch_num > 1 ) ) )
		{
			if( pChan->auto_RTS ) {
        		DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR SIO_MCTRL_BITS_CLR,"
        						 " auto_RTS enabled\n"));
           		status = ERROR;
				break;
			} else
				setManualRTS(pChan, 0);
		}

		break;
	case SIO_MCTRL_ISIG_MASK:     /* mask of lines that can be read */
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M45N ) {
			if( pChan->ch_num <= 1 )
				*(int *)arg = SIO_MODEM_CTS | SIO_MODEM_DSR;
			else
				*(int *)arg = SIO_MODEM_CTS;
		} else if( m77DevData[pChan->device_num]->modId == MOD_ID_M69N ) {
			if( pChan->ch_num <= 1 )
				*(int *)arg = SIO_MODEM_CTS;
			else
				*(int *)arg = SIO_MODEM_CTS | SIO_MODEM_DSR;
		} else
			*(int *)arg = 0;
		break;
	case SIO_MCTRL_OSIG_MASK:     /* mask of lines that can be set */
		if( m77DevData[pChan->device_num]->modId == MOD_ID_M45N ) {
			if( pChan->ch_num <= 1 )
				*(int *)arg = SIO_MODEM_DTR | SIO_MODEM_RTS;
			else
				*(int *)arg = SIO_MODEM_DTR;
		} else if( m77DevData[pChan->device_num]->modId == MOD_ID_M69N ) {
			if( pChan->ch_num <= 1 )
				*(int *)arg = SIO_MODEM_DTR;
			else
				*(int *)arg = SIO_MODEM_DTR | SIO_MODEM_RTS;
		} else
			*(int *)arg = 0;
		break;
	case M77_ECHO_SUPPRESS:		/* only available on M77 */
		if( m77DevData[pChan->device_num]->modId != MOD_ID_M77 ) {
			status = EIO;
			break;
		}

		if (arg) {
			pChan->echo_suppress = TRUE;
			status = m77PhysIntSet (pChan, pChan->physInt);
		}
		else {
			pChan->echo_suppress = FALSE;
			status = m77PhysIntSet (pChan, pChan->physInt);
		}

		break;

	case M77_TX_FIFO_LEVEL:

	    /* check if arg is in range */
		if (arg < 0 || arg >= M77_FIFO_SIZE) {
        	DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR M77_TX_FIFO_LEVEL not in range\n"));
           	status = ERROR;
           	goto endIoctrl;
		}
		pChan->transmit_level = (u_int16) arg; /* transmit trigger level */

		if (ERROR == taskLock()) {
           	status = ERROR;
           	goto endIoctrl;
        }
		lockKey = intLock();

		/* swich to enhanced mode */
		efr = read650(pChan, M77_EFR_OFFSET);
		write650(pChan, M77_EFR_OFFSET, (u_int16)(efr | M77_EFR_ENHANCED_MODE));

		/* enable FIFOs, because clear command (see below) requires FIFOs to be enabled */
		MWRITE_D16(pChan->ma, M77_ISR_FCR_REG, M77_FIFO_ENABLE);
		/* set trigger levels and enable 950 interrupt trigger levels */
		writeICR (pChan, M77_TTL_OFFSET, pChan->transmit_level);
		writeICR (pChan, M77_ACR_OFFSET, (pChan->shadowACR | M77_ACR_950_TRIG_EN));
		/* clear FIFOs */
	    MWRITE_D16(pChan->ma, M77_ISR_FCR_REG,
	    (FCR_DMA_MODE | RxCLEAR | TxCLEAR | FIFO_ENABLE));
		/* set flag in device structure FIFOs enabled */
		pChan->fifo_enabled = TRUE;

		intUnlock(lockKey);
		if (ERROR == taskUnlock()) {
           	status = ERROR;
           	goto endIoctrl;
        }

		break;

	case M77_RX_FIFO_LEVEL:

	    /* check if arg is in range arg = 0 causes int which can not be cleared !! */
		if (arg < 1 || arg >= M77_FIFO_SIZE) {
        	DBGWRT_ERR((DBH, " *** m77Ioctl: ERROR M77_RX_FIFO_LEVEL not in range\n"));
           	status = ERROR;
           	goto endIoctrl;
		}

	    pChan->receive_level = (u_int16) arg;  /* receive trigger level */

		if (ERROR == taskLock()) {
           	status = ERROR;
           	goto endIoctrl;
        }
		lockKey = intLock();

		/* swich to enhanced mode */
		efr = read650(pChan, M77_EFR_OFFSET);
		write650(pChan, M77_EFR_OFFSET, (u_int16)(efr | M77_EFR_ENHANCED_MODE));

		/* enable FIFOs, because clear command (see below) requires FIFOs to be enabled */
		MWRITE_D16(pChan->ma, M77_ISR_FCR_REG, M77_FIFO_ENABLE);
		/* set trigger leves and enable 950 interrupt trigger levels */
		writeICR (pChan, M77_RTL_OFFSET, pChan->receive_level);
		writeICR (pChan, M77_ACR_OFFSET, (pChan->shadowACR | M77_ACR_950_TRIG_EN));
		/* clear FIFOs */
	    MWRITE_D16(pChan->ma, M77_ISR_FCR_REG,
	    (FCR_DMA_MODE | RxCLEAR | TxCLEAR | FIFO_ENABLE));
		/* set flag in device structure FIFOs enabled */
		pChan->fifo_enabled = TRUE;

		intUnlock(lockKey);
		if (ERROR == taskUnlock()) {
           	status = ERROR;
           	goto endIoctrl;
        }

		break;

	case M77_NO_FIFO:
		pChan->fifo_enabled = FALSE;

		if (ERROR == taskLock()) {
           	status = ERROR;
           	goto endIoctrl;
        }
	    lockKey = intLock();

		/* disable 950 interrupt trigger levels */
		writeICR (pChan, M77_ACR_OFFSET, (u_int16)(pChan->shadowACR & (~M77_ACR_950_TRIG_EN)));

		/* disable enhanced (16C950) mode */
		efr = read650(pChan, M77_EFR_OFFSET);
		write650(pChan, M77_EFR_OFFSET, (u_int16)(efr & (~M77_EFR_ENHANCED_MODE)));

		/* clear all bits in FCR except DMA bit */
		MWRITE_D16 (pChan->ma, M77_ISR_FCR_REG, FCR_DMA );
		/* clear the port */
		MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);

		intUnlock(lockKey);
		if (ERROR == taskUnlock()) {
           	status = ERROR;
           	goto endIoctrl;
        }

		break;

	case M77_PHYS_INT_SET:		/* only available on M77 */
		if( m77DevData[pChan->device_num]->modId != MOD_ID_M77 ) {
			status = EIO;
			break;
		}

		status = m77PhysIntSet(pChan, (u_int16) arg);

		break;

	case M77_HS_HIGH_FIFO_LEVEL:
	case M77_HS_LOW_FIFO_LEVEL:
		if( pChan->fifo_enabled == FALSE ||
			m77DevData[pChan->device_num]->modId == MOD_ID_M77 )
		{
			status = ERROR;
			goto endIoctrl;
		}

		if( arg > 127 ) arg = 127;
		if( arg < 1 )   arg = 1; 		/* FCH: bit 1 not writable
										 * FCL: value of 0 is illegal */

		if( request == M77_HS_HIGH_FIFO_LEVEL )
			pChan->hsHighFifoLevel = (u_int16)arg;
		else
			pChan->hsLowFifoLevel = (u_int16)arg;

		status = m77HSModeSet(pChan);

		break;

	case M77_MODEM_AUTO_RTS:
		if (arg) {
			pChan->auto_RTS = TRUE;
			pChan->hsMode |= M77_MODEM_HS_AUTO_RTS;
		} else {
			pChan->auto_RTS = FALSE;
			pChan->hsMode &= ~M77_MODEM_HS_AUTO_RTS;
		}

		status = m77HSModeSet(pChan);

		break;

	case M77_MODEM_AUTO_CTS:
		if (arg) {
			pChan->auto_CTS = TRUE;
			pChan->hsMode |= M77_MODEM_HS_AUTO_CTS;
		} else {
			pChan->auto_CTS = FALSE;
			pChan->hsMode &= ~M77_MODEM_HS_AUTO_CTS;
		}

		status = m77HSModeSet(pChan);

		break;

	case M77_MODEM_AUTO_DSR:
		if (arg) {
			pChan->auto_DSR = TRUE;
			pChan->hsMode |= M77_MODEM_HS_AUTO_DSR;
		} else {
			pChan->auto_DSR = FALSE;
			pChan->hsMode &= ~M77_MODEM_HS_AUTO_DSR;
		}

		status = m77HSModeSet(pChan);

		break;

	case M77_MODEM_AUTO_DTR:
		if (arg) {
			pChan->auto_DTR = TRUE;
			pChan->hsMode |= M77_MODEM_HS_AUTO_DTR;
		} else {
			pChan->auto_DTR = FALSE;
			pChan->hsMode &= ~M77_MODEM_HS_AUTO_DTR;
		}

		status = m77HSModeSet(pChan);

		break;

	case M77_MODEM_XONXOFF:
		if (arg)
			pChan->hsMode |= M77_MODEM_HS_XONXOFF;
		else
			pChan->hsMode &= ~M77_MODEM_HS_XONXOFF;

		pChan->hsInBandMode = (u_int16)arg;
		status = m77HSModeSet(pChan);

		break;

	case M77_TRISTATE:
		if( m77DevData[pChan->device_num]->modId != MOD_ID_M45N )
		{
			status = ERROR;
			goto endIoctrl;
		}

		if( arg )
			setM45nTristate(pChan, 1);
		else
			setM45nTristate(pChan, 0);
		break;
	case M77_GET_LAST_ERR:
			*(int *)arg = pChan->lastErr;
			pChan->lastErr = 0; /* clear error flags */

		break;
#ifdef DBG
	case M77_REG_DUMP:
		m77OxRegDump(pChan);
		break;
#endif /* DBG */

	default:
	    status = tyIoctl (&pChan->tyDev, request, arg);
	    break;

	}
endIoctrl:
	DBGWRT_2(( DBH, "m77Ioctl: MOD_ID: %d: Module: %d, Channel: %d, request: 0x%04x, status 0x%04x\n",
				m77DevData[pChan->device_num]->modId, pChan->device_num, pChan->ch_num, request, status));
    return (status);
    }



/********************************** m77Int *********************************
 *
 *  Description: Handle a receiver/transmitter/error interrupt
 *
 *	This routine handles four sources of interrupts from the UART. They are
 *  prioritized in the following order by the Interrupt Identification Register:
 *  Receiver Line Status, Received Data Ready/Receiver FIFO timeout,
 *  Transmit Holding Register Empty/Transmit FIFO threshold,
 *  and Modem Status.
 *
 *  If there is another character to be transmitted, it sends it.  If
 *  not, or if a device has never been created for this channel, just
 *  disable the interrupt.
 *  When a modem status interrupt occurs, it is cleared because M77 doesn't
 *  support modem lines.
 *
 *------------------------------------------------------------------------------
 *  Input......:  devNum        number of M77 module in system
 *
 *  Output.....:  Return
 *
 *  Globals....:
 *******************************************************************************/
static int m77Int(int devNum)
{
    char outChar;
    u_int16 intID;
    u_int16 lineStatus;
	u_int16 msr;
	int  bytesFree;
	volatile int  readBytes;
	int  uart;
	M77_TY_CO_DEV	*pChan;
	u_int16  intSrc;
	int32 BbisIrqCause;
	u_int16 rxHold;
	u_int16 ierReg;
	u_int16 isrReg;
	volatile u_int16 tflReg;
	u_int32 controllerNum = 0;
	int timedOut;


	IDBGWRT_1( ( DBH, "m77Int\n" ));

    BbisIrqCause = m77DevData[devNum]->pBbisEntry.irqSrvInit(m77DevData[devNum]->brdHdl,
    														 m77DevData[devNum]->devSlot);

	if ( BbisIrqCause &  BBIS_IRQ_EXP) {
		IDBGWRT_ERR (( DBH, ">>> *** m77Int: ERROR - bbis exception at %s ***\n",
		m77DevData[devNum]->brdName ));
	}

	/* if INT could be caused from M77 do interrupt service */
	if ((BbisIrqCause & BBIS_IRQ_UNK) || (BbisIrqCause & BBIS_IRQ_YES)) {

		/* determine interrupt source by reading each channels ISR register */
		for (uart = 0; uart < m77DevData[devNum]->numChannels; uart++) {

			pChan = &m_tyCoDv[devNum][uart];
			intSrc = MREAD_D16 (pChan->ma, M77_ISR_FCR_REG);
			IDBGWRT_3 (( DBH, ">>> m77Int: pChan->ma = 0x%08lx ISR=0x%04x\n", pChan->ma, intSrc));
			if (intSrc & M77_ISR_NO_INT_PEND){
			}
			else
				goto myInterrup;

		}

		/* interrupt was not caused from device with number devNum	*/
		IDBGWRT_2 (( DBH, ">>> m77Int: ERROR - interrupt was not caused from devNum = %d\n",
		devNum));

		goto isrEnd;


	myInterrup:

		intID = intSrc & M77_ISR_MASK;
	    switch (intID)
	    {
	    	/* serial error or break */
	    	case M77_ISR_RX_ERROR:
    			lineStatus = MREAD_D16(pChan->ma, M77_LSR_REG);
				pChan->lastErr |= (u_int8) ((lineStatus >> 1) & 0x07);
				IDBGWRT_ERR( ( DBH, ">>> m77Int: M77_ISR_RX_ERROR CH%d: "
				"lineStatus = 0x%02x\n", pChan->ch_num, lineStatus));
    			break;

	    	/* data received interrupt */
	    	case M77_ISR_RX_DATA_AVAIL:
	    	case M77_ISR_RX_TIMEOUT:
				if (pChan->fifo_enabled) {

					/* read RFL twice a. compare the values to be sure data are valid */
    				do {
    					readBytes = readASRxFL(pChan, M77_RFL_OFFSET);
    					IDBGWRT_2( ( DBH, ">>> m77Int: m77LeaveRxInBuf, FIFO enbl, readBytes = %d\n", readBytes));
    				} while (readBytes != readASRxFL(pChan, M77_RFL_OFFSET) );

	    			while (readBytes--)
	    			{
	    				if (pChan->created) {
		       				rxHold = MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);
		       				if (ERROR == tyIRd (&pChan->tyDev, (char) rxHold)) {
		       					IDBGWRT_ERR(( DBH, ">>> *** m77Int: M77_ISR_RX_DATA_AVAIL Ringbuffer full\n"));
		       					pChan->lastErr |= (u_int8) M77_DRV_BUFF_OVERFLOW;
		       				}
							IDBGWRT_2( ( DBH, ">>> m77Int: M77_ISR_RX_: rxHold = 0x%02x, %c "
							"FIFO enbl, readBytes = %d\n", (char) rxHold, (char) rxHold, readBytes));
	#ifdef DBG
	# ifdef M77_SW
							m77InBytes_sw++;
	# else
							m77InBytes++;
	# endif /* M77_SW */
	#endif /* DBG */
						}

		   				else {
		       				MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);
	#ifdef DBG
	# ifdef M77_SW
		       				m77InBytes_sw++;
	# else
		       				m77InBytes++;
	# endif /* M77_SW */
	#endif /* DBG */
	    				} /* else */
		       		}  /* while (readBytes) */
	    		} /* if (pChan->fifo_enabled) */
	    		else {
	    			while ((MREAD_D16(pChan->ma, M77_LSR_REG)) & M77_LSR_RXRDY){

	    				if (pChan->created) {
		       				rxHold = MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);
		       				if (ERROR == tyIRd (&pChan->tyDev, (char) rxHold)) {
		       					IDBGWRT_ERR(( DBH, ">>> *** m77Int: M77_ISR_RX_DATA_AVAIL Ringbuffer full\n"));
		       					pChan->lastErr |= (u_int8) M77_DRV_BUFF_OVERFLOW;
		       				}
							IDBGWRT_2( ( DBH, ">>> m77Int: M77_ISR_RX_: rxHold = 0x%02x\n",
							rxHold));
	#ifdef DBG
	# ifdef M77_SW
							m77InBytes_sw++;
	# else
							m77InBytes++;
	# endif /* M77_SW */
	#endif /* DBG */
						}

		   				else {
		       				MREAD_D16(pChan->ma, M77_HOLD_DLL_REG);
	#ifdef DBG
	# ifdef M77_SW
		       				m77InBytes_sw++;
	# else
		       				m77InBytes++;
	# endif /* M77_SW */
	#endif /* DBG */
		       			} /* else */
		       		} /* while */
		       	} /* else */
	#ifdef DBG
	# ifdef M77_SW
	       		m77IntCountIn_sw++;
	# else
	       		m77IntCountIn++;
	# endif /* M77_SW */
	#endif /* DBG */
	       		break;

	       	/* transmit interrupt */
		   	case M77_ISR_THR_EMPTY:
		   			/* OX16C950 FIFO trigger */
		   			if (pChan->fifo_enabled){
						/* read TFL twice a. compare the values to be sure data are valid */
		   				do {
		   					tflReg = readASRxFL(pChan, M77_TFL_OFFSET);
		   				} while (tflReg != readASRxFL(pChan, M77_TFL_OFFSET));
						/* calculate number of free bytes in FIFO */
	   					bytesFree = M77_FIFO_SIZE - tflReg;

		   				IDBGWRT_2( ( DBH, ">>> m77Int: M77_ISR_THR_EMPTY: bytesFree = %d\n",
		   				bytesFree));
		   			}
		   			else bytesFree = 1;


		   			/* put data in FIFO until trigger level reached */
		   			while (bytesFree--)
		   			{
		   				if ((pChan->created && tyITx (&pChan->tyDev, &outChar)) == OK)
		   				{
		       				MWRITE_D16(pChan->ma, M77_HOLD_DLL_REG, (u_int16) outChar);
	           			} else
		       			{
	           				ierReg = MREAD_D16(pChan->ma, M77_IER_DLM_REG);
	           				ierReg &= ~M77_IER_TBE;
	           				MWRITE_D16(pChan->ma, M77_IER_DLM_REG, ierReg);
		       				break;
		       			}
	#ifdef DBG
	# ifdef M77_SW
		       			m77OutBytes_sw++;
	# else
		       			m77OutBytes++;
	# endif /* M77_SW */
	#endif /* DBG */
		       		}
	#ifdef DBG
	# ifdef M77_SW
		       		m77IntCountOut_sw++;
	# else
		       		m77IntCountOut++;
	# endif /* M77_SW */
	#endif /* DBG */

		       	break;

		   	/* modem status interrupt */
		   	case M77_ISR_MODEM_STATE:
		   		msr = MREAD_D16(pChan->ma, M77_MSR_REG);
		   		IDBGWRT_2( ( DBH, ">>> m77Int: M77_ISR_MODEM_STATE: msr = 0x%04x\n",
		   					msr) );
				/*
				 * if CTS handshaking enabled and the CTS line changed,
				 * enable or disable tx interrupt accordingly.
			     */
			    if ( pChan->auto_CTS && msr & M77_MSR_DCTS )
				{
       				ierReg = MREAD_D16(pChan->ma, M77_IER_DLM_REG);

					if (msr & M77_MSR_CTS)
					    /* CTS was turned on */
					    ierReg |= M77_IER_TBE;
					else
					    /* CTS was turned off */
					    ierReg &= ~M77_IER_TBE;

       				MWRITE_D16(pChan->ma, M77_IER_DLM_REG, ierReg);
				}

	#ifdef DBG
	# ifdef M77_SW
		       	m77IntCountHs_sw++;
	# else
		       	m77IntCountHs++;
	# endif /* M77_SW */
	#endif /* DBG */
		       	break;

			default:
				IDBGWRT_ERR( ( DBH, ">>>*** m77Int: Unhandled INT intID=0x%04x\n",
				intID));

				break;

		} /*switch*/

isrEnd:
	    /* clear interrupt on M77 */
		if( (m77DevData[devNum]->modId == MOD_ID_M45N) && (uart > 3) )
			controllerNum = 1;

		isrReg = MREAD_D16( m77DevData[devNum]->ma,
							M77_IRQ_REG +
							(controllerNum * M45N_CTRL_REG_BLOCK_SIZE) );

		isrReg |= M77_IRQ_CLEAR;
		MWRITE_D16( m77DevData[devNum]->ma,
					M77_IRQ_REG + (controllerNum * M45N_CTRL_REG_BLOCK_SIZE),
					isrReg );

		/*
		 * If we exit the interrupt handler too early the interrupt request
		 * from 0X16C954 can be still pending and generate spurious interrupts.
		 * Then we check (maximum 2 times) if interrupt request is really
		 * cleared before exiting the interrupt handler.
		 */
		timedOut = 2;
		do {
			isrReg = MREAD_D16( m77DevData[devNum]->ma,
								M77_IRQ_REG +
								(controllerNum * M45N_CTRL_REG_BLOCK_SIZE) );
			timedOut--;
		} while(((isrReg & M77_IRQ_CLEAR) == M77_IRQ_CLEAR) && (timedOut > 0));

	} /* if */

	/* call BBIS IRQ-Exit function to clear int on base board */
	m77DevData[devNum]->pBbisEntry.irqSrvExit( m77DevData[devNum]->brdHdl,
											   m77DevData[devNum]->devSlot );

	return 0;
}


/******************************** m77Startup ***********************************
 *
 *  Description: Transmitter startup routine
 *
 *  Enable Tx-Interrupt, so transmission is started by ISR.
 *
 *------------------------------------------------------------------------------
 *  Input......:  pChan         pointer to channel device
 *
 *  Output.....:  Return 		always OK
 *
 *  Globals....:  m77Rsrc
 *******************************************************************************/
static int m77Startup
    (
    M77_TY_CO_DEV 			*pChan		/* tty device to start up */
    )
{

	int      lockKey;
	u_int16  ier;
	u_int16  msr;

	DBGWRT_1( ( DBH, "m77Startup\n"));
	/* check if actually in INT context */
	if( FALSE == intContext() ) {
		taskLock();
	}
	lockKey = intLock();

    ier = MREAD_D16(pChan->ma, M77_IER_DLM_REG);

    if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 )
	{
		ier &= ~M77_IER_MSI; /* disable modem status interrupt */
		ier |= M77_IER_TBE;  /* enable Tx interrupt */
	} else
	{
		ier |= M77_IER_MSI;    /* enable modem status interrupt */

		msr = MREAD_D16(pChan->ma, M77_MSR_REG);
		/* handshaking enabled and CTS line is asserted: enable Tx interrupt */
		if( pChan->auto_CTS ) {
			if( msr & M77_MSR_CTS )
				ier |= M77_IER_TBE;
			else
				ier &= (~M77_IER_TBE);
		} else { /* no automatic handshaking, always enable Tx interrupt */
			ier |= M77_IER_TBE;
		}
	}

    MWRITE_D16(pChan->ma, M77_IER_DLM_REG, ier);

	intUnlock(lockKey);
	/* check if actually in INT context */
	if( FALSE == intContext() ) {
		taskUnlock();
	}

	DBGWRT_2( ( DBH, "m77Startup: Set IER = 0x%02x\n", ier ));

    return (OK);
}

/******************************** m77PhysIntSet ******************************
 *
 *  Description: set pysical interface
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                drvMode       line driver mode RS232 etc.
 *
 *  Output.....:  Return        OK or ERROR
 *
 *  Globals....: m77DevData
 *****************************************************************************/
static int m77PhysIntSet(M77_TY_CO_DEV *pChan, u_int16 drvMode)
{
	int lockKey;
	int retVal = OK;

	DBGWRT_1 ((DBH, "m77PhysIntSet:\n"));

	taskLock();
	lockKey = intLock();

	DBGWRT_2 ((DBH, "m77PhysIntSet: M77_PHYS_INT_SET: mode = 0x%02x\n", drvMode));

	switch (drvMode) {
		case M77_RS423:

			MWRITE_D16 (m77DevData[pChan->device_num]->ma,
						M77_DCR_REG_BASE + (2 * pChan->ch_num),
						M77_RS423);
			break;

		case M77_RS422_HD:

			if (pChan->echo_suppress == TRUE) {
				/* set RS422HD mode */
				MWRITE_D16 (m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num),
							M77_RS422_HD);
			}
			else {
				/* set RS422HD mode + echo enable */
				MWRITE_D16 (m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num),
							M77_RS422_HD | M77_RX_EN);
			}
			break;

		case M77_RS422_FD:

			MWRITE_D16 (m77DevData[pChan->device_num]->ma,
						M77_DCR_REG_BASE + (2 * pChan->ch_num),
						M77_RS422_FD);
			break;

		case M77_RS485_HD:

			if (pChan->echo_suppress == TRUE) {
				/* set M77_RS485_HD mode */
				MWRITE_D16 (m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num),
							M77_RS485_HD);
			}
			else {
				/* set M77_RS485_HD mode + echo enable */
				MWRITE_D16 (m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num),
							M77_RS485_HD | M77_RX_EN);
			}
			break;

		case M77_RS485_FD:
			/* set M77_RS485_FD mode + echo enable */
			MWRITE_D16 (m77DevData[pChan->device_num]->ma,
						M77_DCR_REG_BASE + (2 * pChan->ch_num),
						M77_RS485_FD);
			break;

		case M77_RS232:
			if( m77DevData[pChan->device_num]->modId != MOD_ID_M45N ) {
				MWRITE_D16 (m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num),
							M77_RS232);
			}
			break;

		default:
				DBGWRT_ERR ((DBH, " *** m77PhysIntSet: Invalid mode\n"));
				drvMode = pChan->physInt;
				retVal = ERROR;

	}

	/* store new mode in device structure */
	pChan->physInt = drvMode;

	intUnlock(lockKey);
	taskUnlock();

	return (retVal);
}

/******************************** m77HSModeSet ********************************
 *
 *  Description: set handshake mode (hardware handshake only for M45N/M69N)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                hsMode        handshake mode to set
 *
 *  Output.....:  Return        OK or ENOTSUP
 *
 *  Globals....:
 *****************************************************************************/
static int m77HSModeSet(M77_TY_CO_DEV *pChan)
{
	int lockKey;
	int retVal = OK;
	u_int16 ier = 0;
	u_int16 msr = 0;

	DBGWRT_1 ((DBH, "m77HSModeSet:\n"));

	taskLock();
	lockKey = intLock();

	DBGWRT_2 ((DBH, "m77HSModeSet: mode = 0x%02x\n", pChan->hsMode));

	if( pChan->hsMode & M77_MODEM_HS_XONXOFF ) {
		/* set Xon and Xoff characters */
		write650(pChan, M77_XON1_OFFSET, M77_XON_CHAR);
		write650(pChan, M77_XOFF1_OFFSET, M77_XOFF_CHAR);
		write650(pChan, M77_XON2_OFFSET, M77_XON_CHAR);
		write650(pChan, M77_XOFF2_OFFSET, M77_XOFF_CHAR);

		setInBandFlowControlMode( pChan, (u_int)pChan->hsInBandMode );
	} else
		setInBandFlowControlMode( pChan, 0 );


	if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 )
	{
		DBGWRT_ERR ((DBH, " *** m77HSModeSet: M77 module doesn't support"
						  " hardware handshaking, flags ignored\n"));
		goto exit;
	}

	/* set flow control FIFO levels */
	writeICR(pChan, M77_FCH_OFFSET, (u_int16)(pChan->hsHighFifoLevel));
	writeICR(pChan, M77_FCL_OFFSET, (u_int16)(pChan->hsLowFifoLevel));

	if( pChan->hsMode & M77_MODEM_HS_AUTO_RTS )
		setAutoRTSEnable( pChan, 1 );
	else
		setAutoRTSEnable( pChan, 0 );

	if( pChan->hsMode & M77_MODEM_HS_AUTO_CTS )
		setAutoCTSEnable( pChan, 1 );
	else
		setAutoCTSEnable( pChan, 0 );

	if( pChan->hsMode & M77_MODEM_HS_AUTO_DSR )
		setAutoDSREnable( pChan, 1 );
	else
		setAutoDSREnable( pChan, 0 );

	if( pChan->hsMode & M77_MODEM_HS_AUTO_DTR )
		setAutoDTREnable( pChan, 1 );
	else
		setAutoDTREnable( pChan, 0 );

    ier = MREAD_D16(pChan->ma, M77_IER_DLM_REG);
	if( m77DevData[pChan->device_num]->modId == MOD_ID_M77 )
	{
		ier &= ~M77_IER_MSI; /* disable modem status interrupt */
		ier |= M77_IER_TBE; /* enable Tx interrupt */
	} else {
		ier |= M77_IER_MSI;    /* enable modem status interrupt */

		/* handshaking enabled and CTS line is asserted: enable Tx interrupt */
		if( pChan->auto_CTS ) {
			msr = MREAD_D16(pChan->ma, M77_MSR_REG);
			if( msr & M77_MSR_CTS )
				ier |= M77_IER_TBE;
			else
				ier &= (~M77_IER_TBE);
		}
	}

    MWRITE_D16(pChan->ma, M77_IER_DLM_REG, ier);

exit:
	intUnlock (lockKey);
	taskUnlock();

	return (retVal);
}

/******************************** m77ChannelRest ******************************
 *
 *  Description: Reset the m77Channel
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *  Output.....:  Return        -
 *
 *  Globals....:
 *****************************************************************************/
static void m77ChannelRest(M77_TY_CO_DEV *pChan)
{
	int lockKey;

	DBGWRT_1( ( DBH, "m77ChannelRest\n"));
	taskUnlock();
	lockKey = intLock();

	/* reset the channel usinfg CSR-register */
	writeICR (pChan, M77_CSR_INDEX, 0x00);

	intUnlock(lockKey);
	taskUnlock();

	/* initialize error Rx-Error flags */
	pChan->lastErr = 0;

	/* initialize error shadow ACR value */
	pChan->shadowACR = 0;

}

/******************************** readDivisor ********************************
 *
 *  Description: Read the baud rate divisor
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *  Output.....:  Return        the divisor
 *
 *  Globals....:
 *****************************************************************************/
static UINT16 readDivisor(M77_TY_CO_DEV *pChan)
{
	UINT16	dlldlm;
	u_int16	oldLCR;
	/* store the current value of LCR
	and then set the top bit to allow
	divisor latch access */
	oldLCR = MREAD_D16(pChan->ma, M77_LCR_REG);
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR | M77_LCR_DL_ACCESS_KEY);
	/* construct divisor latch, then restore old values*/
	dlldlm = (UINT16)(MREAD_D16(pChan->ma, M77_IER_DLM_REG)<<8);
	dlldlm += (MREAD_D16(pChan->ma, M77_HOLD_DLL_REG) & 0x00ff);
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR);
	return dlldlm;
}

/******************************** writeDivisor *******************************
 *
 *  Description: Write the baud rate divisor
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                divisor       the divisor to write
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void writeDivisor(M77_TY_CO_DEV *pChan, UINT16 divisor)
{
	u_int16	oldLCR;
	/* store old LCR and the access divisor */
	oldLCR = MREAD_D16(pChan->ma, M77_LCR_REG);
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR | M77_LCR_DL_ACCESS_KEY);
	/* write divisor then restore */
	MWRITE_D16(pChan->ma, M77_HOLD_DLL_REG, divisor & 0x00FF);
	MWRITE_D16(pChan->ma, M77_IER_DLM_REG, (divisor & 0xff00)>>8);
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR);
}

/********************************* read650 ***********************************
 *
 *  Description: Read the 650 specific registers
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                offset        register offset
 *
 *  Output.....:  Return        value of the register
 *
 *  Globals....:
 *****************************************************************************/
static u_int16 read650(M77_TY_CO_DEV *pChan, u_int16 offset)
{
	u_int16	result, oldLCR;
	/* store old LCR and write access code */
	oldLCR = MREAD_D16(pChan->ma, M77_LCR_REG);
	MWRITE_D16(pChan->ma, M77_LCR_REG, M77_LCR_650_ACCESS_KEY);
	/* read the register */
	result = MREAD_D16(pChan->ma, offset);
	/* restore LCR */
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR);
	return (result & 0x00ff);
}

/******************************** write650 ***********************************
 *
 *  Description: Write the 650 specific registers
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                offset        register offset
 *                value         value to write
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void write650(M77_TY_CO_DEV *pChan, u_int16 offset, u_int16 value)
{
	u_int16	oldLCR;
	/* store old LCR and write access code */
	oldLCR = MREAD_D16(pChan->ma, M77_LCR_REG);
	MWRITE_D16(pChan->ma, M77_LCR_REG, M77_LCR_650_ACCESS_KEY);
	/* write register */
	MWRITE_D16(pChan->ma, offset, value);
	/* restore LCR */
	MWRITE_D16(pChan->ma, M77_LCR_REG, oldLCR);
}

/******************************** writeICR ***********************************
 *
 *  Description: Write to the ICR with the specified index and value
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                index         index of the register
 *                value         value to write
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void writeICR(M77_TY_CO_DEV *pChan, u_int16 index, u_int16 value)
{
	/* writes the ICR set register
	   index by the index parameter */
	MWRITE_D16(pChan->ma, M77_SPR_REG, index);
#ifdef DBG
	/* DBGWRT_2( ( DBH, "writeICR: index 0x%04x, value 0x%04x\n", index, value )); */
#endif /* DBG */
	MWRITE_D16(pChan->ma, M77_ICR_OFFSET, value);
	/* record changes made to ACR */
	if (index==M77_ACR_OFFSET) pChan->shadowACR = value;
}

#ifdef DBG
/******************************* readICR *************************************
 *
 *  Description: Read the ICR register value of the specified index
 *               (not used by this driver)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                index         the index of the register
 *
 *  Output.....:  Return        the value of the register
 *
 *  Globals....:
 *****************************************************************************/
static u_int16 readICR(M77_TY_CO_DEV *pChan, u_int16 index)
{
	u_int16	retVal;

	writeICR(pChan, M77_ACR_OFFSET, (u_int16)(pChan->shadowACR | M77_ACR_ICR_READ_EN));
	MWRITE_D16(pChan->ma, M77_SPR_REG, index);
	retVal = MREAD_D16(pChan->ma, M77_ICR_OFFSET);
	writeICR(pChan, M77_ACR_OFFSET, (u_int16)(pChan->shadowACR & ~M77_ACR_ICR_READ_EN));
	return (retVal & 0x00ff);
}
#endif /* DBG */

/************************** UnlockAdditionalStatus ***************************
 *
 *  Description: Unlock the access to the 950 specific registers
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void UnlockAdditionalStatus(M77_TY_CO_DEV *pChan)
{
	/* Set the top bit of ACR to enable
	   950 specific register set access */
	pChan->shadowACR |= M77_ACR_950_READ_EN;
	writeICR(pChan, M77_ACR_OFFSET, pChan->shadowACR);
}

/**************************** LockAdditionalStatus ***************************
 *
 *  Description: Lock the access to the 950 specific registers
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void LockAdditionalStatus(M77_TY_CO_DEV *pChan)
{
	/* Clear the top bit of ACR to disable
	   950 specific register set access */
	pChan->shadowACR &= (~M77_ACR_950_READ_EN);
	writeICR(pChan, M77_ACR_OFFSET, pChan->shadowACR);
}


/******************************** readASRxFL **********************************
 *
 *  Description: Write to the ASR register
 *               NOTE: Only Bit[0] can be written
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                index         index of the register
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static u_int16 readASRxFL(M77_TY_CO_DEV *pChan, u_int16 offset)
{
	/* Returns the data stored in the ASR register */
	u_int16	retVal;
	UnlockAdditionalStatus(pChan);
#ifdef DBG
	/* DBGWRT_2( ( DBH, "readASRxFL: offset 0x%04x\n", offset )); */
#endif /* DBG */
	retVal = MREAD_D16(pChan->ma, offset);
	LockAdditionalStatus(pChan);
	return (retVal & 0x00ff);
}

/******************************** setManualRTS *******************************
 *
 *  Description: Set the state of the request to send pin
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                arg           1: set, 0: clear
 *
 *  Output.....:  Return  		OK
 *
 *  Globals....:
 *****************************************************************************/
static STATUS setManualRTS(M77_TY_CO_DEV *pChan, int arg)
{
	u_int16 mcr;

	mcr = MREAD_D16( pChan->ma, M77_MCR_REG );
	if(arg) {
		/* set RTS bit */
		MWRITE_D16( pChan->ma, M77_MCR_REG, (mcr | M77_MCR_RTS) );
	} else {
		/* clear RTS bit */
		MWRITE_D16( pChan->ma, M77_MCR_REG, (mcr & ~M77_MCR_RTS));
	}

	return OK;
}

/******************************** setManualDTR *******************************
 *
 *  Description: Set the state of the data terminal ready pin
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                arg           1: set, 0: clear
 *
 *  Output.....:  Return  		OK
 *
 *  Globals....:
 *****************************************************************************/
static STATUS setManualDTR(M77_TY_CO_DEV *pChan, int arg)
{
	u_int16 mcr;

	mcr = MREAD_D16(pChan->ma, M77_MCR_REG);
	if(arg) {
		/* set DTR bit */
		MWRITE_D16(pChan->ma, M77_MCR_REG, (mcr | M77_MCR_DTR));
	} else {
		/* set DTR bit */
		MWRITE_D16(pChan->ma, M77_MCR_REG, (mcr & ~M77_MCR_DTR));
	}
	return OK;
}

/******************************** getManualCTS *******************************
 *
 *  Description: Get the state of the clear to send pin
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *
 *  Output.....:  Return  		the state of the pin
 *
 *  Globals....:
 *****************************************************************************/
static int getManualCTS(M77_TY_CO_DEV *pChan)
{
	int	retVal;

	if ( MREAD_D16(pChan->ma, M77_MSR_REG) & M77_MSR_CTS )
		retVal = 1;
	else retVal = 0;
	return (retVal);
}

/******************************** getManualDSR *******************************
 *
 *  Description: Get the state of the data set ready pin
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *
 *  Output.....:  Return  		the state of the pin
 *
 *  Globals....:
 *****************************************************************************/
static int getManualDSR(M77_TY_CO_DEV *pChan)
{
	int	retVal;

	if ( MREAD_D16(pChan->ma, M77_MSR_REG) & M77_MSR_DSR )
		retVal = 1;
	else retVal = 0;
	return (retVal);
}

#if 0 /* currently not used */
/******************************** getManualDCD *******************************
 *
 *  Description: Get the state of the data carrier detect pin
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *
 *
 *  Output.....:  Return  		the state of the pin
 *
 *  Globals....:
 *****************************************************************************/
static int getManualDCD(M77_TY_CO_DEV *pChan)
{
	int	retVal;

	if ( MREAD_D16(pChan->ma, M77_MSR_REG) & M77_MSR_DCD )
		retVal = 1;
	else retVal = 0;
	return (retVal);
}
#endif   /* currently not used */
/***************************** setAutoCTSEnable ******************************
 *
 *  Description: Enables/Disables the automatic CTS mode (on chip)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                state         1: enable, 0: disable
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setAutoCTSEnable(M77_TY_CO_DEV *pChan, BOOL state)
{
	/* Sets the state of automatic CTS flow control enable bit to state */
	u_int16 efr = read650(pChan, M77_EFR_OFFSET);
	/* Set the bit according to the state requested */
	if(state) efr |= M77_EFR_FLOWCTL_CTS;
	else      efr &= ~M77_EFR_FLOWCTL_CTS;
	/* Write new value */
	write650(pChan, M77_EFR_OFFSET, efr);
}

/***************************** setAutoRTSEnable ******************************
 *
 *  Description: Enables/Disables the automatic RTS mode (on chip)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                state         1: enable, 0: disable
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setAutoRTSEnable(M77_TY_CO_DEV *pChan, BOOL state)
{
	/* Sets the state of automatic RTS flow control enable bit to state */
	u_int16 efr = read650(pChan, M77_EFR_OFFSET);
	/* Set the bit according to the state requested */
	if(state) efr |= M77_EFR_FLOWCTL_RTS;
	else      efr &= ~M77_EFR_FLOWCTL_RTS;
	/* Write new value */
	write650(pChan, M77_EFR_OFFSET, efr);
}

/***************************** setAutoDSREnable ******************************
 *
 *  Description: Enables/Disables the automatic DSR mode (on chip)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                state         1: enable, 0: disable
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setAutoDSREnable(M77_TY_CO_DEV *pChan, BOOL state)
{
	/* Sets the state of automatic DSR flow control enable bit to state */
	if(state) pChan->shadowACR |= M77_ACR_AUTO_DSR_EN;
	else      pChan->shadowACR &= ~M77_ACR_AUTO_DSR_EN;
	/* Write new value */
	writeICR(pChan, M77_ACR_OFFSET, pChan->shadowACR);
}

/***************************** setAutoDTREnable ******************************
 *
 *  Description: Enables/Disables the automatic DTR mode (on chip)
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                state         1: enable, 0: disable
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setAutoDTREnable(M77_TY_CO_DEV *pChan, BOOL state)
{
	/* Sets the state of automatic DTR flow control enable bit to state */
	if(state) pChan->shadowACR |= M77_ACR_DTR_RX_CTL;
	else      pChan->shadowACR &= ~M77_ACR_DTR_RX_CTL;
	/* Write new value */
	writeICR(pChan, M77_ACR_OFFSET, pChan->shadowACR);
}

/************************** setInBandFlowControlMode *************************
 *
 *  Description: Sets the automatic in band flow control to the given mode
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                mode          the desired mode (see OX16C954 manual)
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setInBandFlowControlMode(M77_TY_CO_DEV *pChan, u_int8 mode)
{
	/* Sets the automatic inband flow control mode to the */
	/* specified mode index in the table below */
	u_int8 modeTable[11]={0x00,0x02,0x01,
						  0x08,0x0A,0x09,0x0B,
						  0x04,0x06,0x05,0x07};
	u_int16 efr = read650(pChan, M77_EFR_OFFSET) & 0xF0;
	write650(pChan, M77_EFR_OFFSET, (u_int16)(efr | modeTable[mode]));
}

/************************** setM45nTristate **********************************
 *
 *  Description: Sets the M45N line drivers to tristate
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         the channel descriptor
 *                state         0: normal
 *                              1: tristate
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
static void setM45nTristate(M77_TY_CO_DEV *pChan, BOOL state)
{
	u_int8 mask = 0;
	u_int8 offset = 0x00;

	switch (pChan->ch_num )
	{
		case 0:
			mask  |= M45N_TCR1_CHAN0;
			offset = M45N_TCR1_REG;
			break;
		case 1:
			mask  |= M45N_TCR1_CHAN1;
			offset = M45N_TCR1_REG;
			break;
		case 2:
		case 3:
			mask  |= M45N_TCR1_CHAN23;
			offset = M45N_TCR1_REG;
			break;
		case 4:
		case 5:
			mask |= M45N_TCR2_CHAN45;
			offset = M45N_TCR2_REG;
			break;
		default:
			mask |= M45N_TCR2_CHAN67;
			offset = M45N_TCR2_REG;
			break;

	}

	if( state ) {
		MSETMASK_D16( m77DevData[pChan->device_num]->ma, offset,  mask );
	} else {
		MCLRMASK_D16( m77DevData[pChan->device_num]->ma, offset,  mask );
	}

	return;
}

/******************************** m77ReadDesc *********************************
 *
 *  Description: Read descriptor entries for M77
 *
 *----------------------------------------------------------------------------
 *  Input......:  pDesc         M77 descriptor specifier
 *                pModData      pointer to module data structure
 *                devNum        number of M77 in system
 *
 *  Output.....:  return        0 if success, otherwise error code
 *
 *  Globals....:  m_tyCoDv, m77DgbLev
 *****************************************************************************/
static int32 m77ReadDesc(DESC_SPEC *pDesc, MODSTAT *pModData, int devNum)
{
	int32        retCode = 0;
	int32        err2;
    u_int32      strLen = BK_MAX_BBIS_BRD_NAME;
    DESC_HANDLE  *descHdl = NULL;


	/* get descriptor access handle */
   if( ( retCode = DESC_Init( pDesc,
                             OSS_VXWORKS_OS_HDL,
                             &descHdl )
        )
      )
    {
        goto CLEANUP;
    }/*if*/

	/* get board name */
    if( ( retCode = DESC_GetString( descHdl ,
                                    "",
                                    &pModData->brdName[0],
                                    &strLen,
                                    "BOARD_NAME" )
        )
      )
    {
        DBGWRT_ERR( ( DBH, " *** m77ReadDesc: DESC_GetString BOARD_NAME\n"));
        goto CLEANUP;
    }/*if*/


	/* get device slot number */
    retCode = DESC_GetUInt32( descHdl , 0, &pModData->devSlot, "DEVICE_SLOT");
    if( retCode != 0 )
    {
        DBGWRT_ERR( ( DBH, " *** m77ReadDesc: DESC_GetUInt32 DEVICE_SLOT\n" ));
        goto CLEANUP;
    }

    /* get debug level */
    retCode = DESC_GetUInt32( descHdl,
                              OSS_DBG_DEFAULT,
                              &m77DgbLev,
                              "DEBUG_LEVEL" );
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
    {
        DBGWRT_ERR( ( DBH, " *** m77ReadDesc: DESC_GetUInt32 DEBUG_LEVEL\n"));
        goto CLEANUP;
    }

    /* module ID check */
    retCode = DESC_GetUInt32( descHdl,
                              1,
                              &pModData->idCheck,
                              "ID_CHECK" );
    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
    {
        DBGWRT_ERR( ( DBH, " *** m77ReadDesc: DESC_GetUInt32 ID_CHECK\n"));
        goto CLEANUP;
    }

	/* say OK */
	retCode = OK;

CLEANUP:
    /*-------------------------+
    |  llDescHdl was created   |
    +-------------------------*/
    if( descHdl )
    {
       err2 = DESC_Exit( &descHdl );

       /* store error code if no error before */
       if( !retCode )
           retCode = err2;
    }/*if*/

	return retCode;

}

/******************************** m77ReadChanDesc *****************************
 *
 *  Description: Read channel specific descriptor entries
 *
 *----------------------------------------------------------------------------
 *  Input......:  pDesc         M77 descriptor specifier
 *                pModData      pointer to module data structure
 *                devNum        number of M77 in system
 *
 *  Output.....:  return        0 if success, otherwise error code
 *
 *  Globals....:  m_tyCoDv
 *****************************************************************************/
static int32 m77ReadChanDesc(DESC_SPEC *pDesc, MODSTAT *pModData, int devNum)
{
	int32        retCode = 0;
	int32        err2;
	int32        i;
	u_int32      descVal;
    DESC_HANDLE  *descHdl = NULL;


	/* get descriptor access handle */
   if( ( retCode = DESC_Init( pDesc,
                             OSS_VXWORKS_OS_HDL,
                             &descHdl )
        )
      )
    {
        goto CLEANUP;
    }/*if*/

	/*----------------------------------------+
	|   channel specific descriptor entries   |
	+----------------------------------------*/
	for (i = 0; i < m77DevData[devNum]->numChannels; i++) {

		/* get SIO_HW_OPTS */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEF_SIO_HW_OPTS,
	                              &descVal,
	                              "CHANNEL_%d/SIO_HW_OPTS",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 SIO_HW_OPTS\n"));
	        goto CLEANUP;
	    }

		m_tyCoDv[devNum][i].sio_hw_opts = (u_int16) descVal;

		/* get FIOBAUDRATE */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEFAULT_BAUD,
	                              &descVal,
	                              "CHANNEL_%d/FIOBAUDRATE",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 FIOBAUDRATE\n"));
	        goto CLEANUP;
	    }

	    m_tyCoDv[devNum][i].baudrate = (UINT) descVal;

		/* get driver internal Rx buffer size */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEF_RX_BUFF_SIZE,
	                              &descVal,
	                              "CHANNEL_%d/DRV_RX_BUFF_SIZE",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 DRV_RX_BUFF_SIZE\n"));
	        goto CLEANUP;
	    }

	    m_tyCoDv[devNum][i].rdBufSize = (int) descVal;

		/* get driver internal Tx buffer size */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEF_TX_BUFF_SIZE,
	                              &descVal,
	                              "CHANNEL_%d/DRV_TX_BUFF_SIZE",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 DRV_TX_BUFF_SIZE\n"));
	        goto CLEANUP;
	    }

	    m_tyCoDv[devNum][i].wrtBufSize = (int) descVal;

		/* get TX_FIFO_LEVEL */
	    retCode = DESC_GetUInt32( descHdl,
	                              TX_FIFO_LEVEL_DEFAULT,
	                              &descVal,
	                              "CHANNEL_%d/TX_FIFO_LEVEL",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 TX_FIFO_LEVEL\n"));
	        goto CLEANUP;
	    }

	    /* check if descVal is in range */
		if (descVal >= M77_FIFO_SIZE) {
			DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 TX_FIFO_LEVEL "
			"not in range\n"));
			retCode = ERROR;
	        goto CLEANUP;
		}
		m_tyCoDv[devNum][i].transmit_level = (u_int16) descVal;

		/* get RX_FIFO_LEVEL */
	    retCode = DESC_GetUInt32( descHdl,
	                              RX_FIFO_LEVEL_DEFAULT,
	                              &descVal,
	                              "CHANNEL_%d/RX_FIFO_LEVEL",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 RX_FIFO_LEVEL\n"));
	        goto CLEANUP;
	    }

	    /* a value of 0 causes an interrupt which can not be cleared => min = 1 */
	    if (descVal < 1 || descVal >= M77_FIFO_SIZE) {
			DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 RX_FIFO_LEVEL "
			"not in range\n"));
			retCode = ERROR;
	        goto CLEANUP;
		}
	    m_tyCoDv[devNum][i].receive_level = (u_int16) descVal;

		/* get NO_FIFO */
	    retCode = DESC_GetUInt32( descHdl,
	                              0, /* FIFO enable */
	                              &descVal,
	                              "CHANNEL_%d/NO_FIFO",
	                               i);

	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 NO_FIFO\n"));
	        goto CLEANUP;
	    }

		if (descVal) {
			m_tyCoDv[devNum][i].fifo_enabled = FALSE;
		}
		else {
			m_tyCoDv[devNum][i].fifo_enabled = TRUE;
		}

		if( m77DevData[devNum]->modId == MOD_ID_M77 )
		{
			/* get PHYS_INT */
		    retCode = DESC_GetUInt32( descHdl,
		                              M77_RS422_FD,
		                              &descVal,
		                              "CHANNEL_%d/PHYS_INT",
		                              i);

		    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
		    {
		        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 PHYS_INT\n"));
		        goto CLEANUP;
		    }

			m_tyCoDv[devNum][i].physInt = (u_int16) descVal;

			/* get ECHO_SUPPRESS */
		    retCode = DESC_GetUInt32( descHdl,
		                              1, /* echo suppression enabled */
		                              &descVal,
		                              "CHANNEL_%d/ECHO_SUPPRESS",
		                               i);
		    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
		    {
		        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 ECHO_SUPPRESS\n"));
		        goto CLEANUP;
		    }
			if (descVal) {
				m_tyCoDv[devNum][i].echo_suppress = TRUE;
			}
			else {
				m_tyCoDv[devNum][i].echo_suppress = FALSE;
			}
		} else {
			m_tyCoDv[devNum][i].physInt = M77_RS232;
		}

		/* get HANDSHAKE_HIGH_FIFO_LEVEL */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEF_FCH,
	                              &descVal,
	                              "CHANNEL_%d/HANDSHAKE_HIGH_FIFO_LEVEL",
	                               i);
	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
	        					"HANDSHAKE_HIGH_FIFO_LEVEL\n"));
	        goto CLEANUP;
	    }
		m_tyCoDv[devNum][i].hsHighFifoLevel = (u_int16)descVal;

		/* get HANDSHAKE_LOW_FIFO_LEVEL */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_DEF_FCL,
	                              &descVal,
	                              "CHANNEL_%d/HANDSHAKE_LOW_FIFO_LEVEL",
	                               i);
	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
	        					"HANDSHAKE_LOW_FIFO_LEVEL\n"));
	        goto CLEANUP;
	    }
		m_tyCoDv[devNum][i].hsLowFifoLevel  = (u_int16)descVal;

		/* get HANDSHAKE_XONXOFF */
	    retCode = DESC_GetUInt32( descHdl,
	                              M77_MODEM_HS_NONE, /* no handshake */
	                              &descVal,
	                              "CHANNEL_%d/HANDSHAKE_XONXOFF",
	                               i);
	    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
	    {
	        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
	        					"HANDSHAKE_XONXOFF\n"));
	        goto CLEANUP;
	    }
		m_tyCoDv[devNum][i].hsMode |= descVal ? M77_MODEM_HS_XONXOFF : 0;

		/* M45N/M69N: get HANDSHAKE settings */
		if( m77DevData[devNum]->modId == MOD_ID_M45N ||
			m77DevData[devNum]->modId == MOD_ID_M69N )
		{
		    retCode = DESC_GetUInt32( descHdl,
		                              M77_MODEM_HS_NONE, /* no handshake */
		                              &descVal,
		                              "CHANNEL_%d/HANDSHAKE_AUTO_CTS",
		                               i);
		    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
		    {
		        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
		        					"HANDSHAKE_AUTO_CTS\n"));
		        goto CLEANUP;
		    }
			m_tyCoDv[devNum][i].hsMode |= descVal ? M77_MODEM_HS_AUTO_CTS : 0;

		    retCode = DESC_GetUInt32( descHdl,
		                              M77_MODEM_HS_NONE, /* no handshake */
		                              &descVal,
		                              "CHANNEL_%d/HANDSHAKE_AUTO_DSR",
		                               i);
		    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
		    {
		        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
		        					"HANDSHAKE_AUTO_DSR\n"));
		        goto CLEANUP;
		    }
			m_tyCoDv[devNum][i].hsMode |= descVal ? M77_MODEM_HS_AUTO_DSR : 0;

			/* full handshaking only on two channels of M45N/M69N */
			if( (m77DevData[devNum]->modId == MOD_ID_M45N && i <= 1 )||
				(m77DevData[devNum]->modId == MOD_ID_M69N && i >  1 ) )
			{
			    retCode = DESC_GetUInt32( descHdl,
			                              M77_MODEM_HS_NONE, /* no handshake */
			                              &descVal,
			                              "CHANNEL_%d/HANDSHAKE_AUTO_RTS",
			                               i);
			    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
			    {
			        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
			        					"HANDSHAKE_AUTO_RTS\n"));
			        goto CLEANUP;
			    }
				m_tyCoDv[devNum][i].hsMode |= descVal ? M77_MODEM_HS_AUTO_RTS : 0;

			    retCode = DESC_GetUInt32( descHdl,
			                              M77_MODEM_HS_NONE, /* no handshake */
			                              &descVal,
			                              "CHANNEL_%d/HANDSHAKE_AUTO_DTR",
			                               i);
			    if( retCode != 0 && retCode != ERR_DESC_KEY_NOTFOUND )
			    {
			        DBGWRT_ERR( ( DBH, " *** m77ReadChanDesc: DESC_GetUInt32 "
			        					"HANDSHAKE_AUTO_DTR\n"));
			        goto CLEANUP;
			    }
				m_tyCoDv[devNum][i].hsMode |= descVal ? M77_MODEM_HS_AUTO_DSR : 0;
			}
		}
	}

	/* say OK */
	retCode = OK;

CLEANUP:
    /*-------------------------+
    |  llDescHdl was created   |
    +-------------------------*/
    if( descHdl )
    {
       err2 = DESC_Exit( &descHdl );

       /* store error code if no error before */
       if( !retCode )
           retCode = err2;
    }/*if*/

	return retCode;

}

#ifdef DBG
/******************************** m77OxRegDump ********************************
 *
 *  Description: Read all registers of Oxford Chip
 *
 *----------------------------------------------------------------------------
 *  Input......:  pChan         pointer to device descriptor
 *
 *  Output.....:  -
 *
 *  Globals....:  -
 *****************************************************************************/
static void m77OxRegDump(M77_TY_CO_DEV *pChan)
{
	int lockKey;
	u_int16 dummy;
	u_int32 controllerNum = 0;

	taskLock();
	lockKey = intLock();

	DBGWRT_3 ((DBH, "m77OxRegDump: %s ch %d\n",
				m77DevData[pChan->device_num]->pModName, pChan->ch_num ));

	dummy = MREAD_D16(pChan->ma, M77_IER_DLM_REG);
	DBGWRT_3 ((DBH, " IER: 0x%02x\n", (u_int8) dummy));

	dummy = MREAD_D16(pChan->ma, M77_ISR_FCR_REG);
	DBGWRT_3 ((DBH, " ISR: 0x%02x\n", (u_int8) dummy));

	dummy = MREAD_D16(pChan->ma, M77_LCR_REG);
	DBGWRT_3 ((DBH, " LCR: 0x%02x\n", (u_int8) dummy));

	dummy = MREAD_D16(pChan->ma, M77_MCR_REG);
	DBGWRT_3 ((DBH, " MCR: 0x%02x\n", (u_int8) dummy));

	dummy = MREAD_D16(pChan->ma, M77_LSR_REG);
	DBGWRT_3 ((DBH, " LSR: 0x%02x\n", (u_int8) dummy));

	dummy = MREAD_D16(pChan->ma, M77_MSR_REG);
	DBGWRT_3 ((DBH, " MSR: 0x%02x\n", (u_int8) dummy));

	dummy = readDivisor(pChan);
	DBGWRT_3 ((DBH, " Divisor: 0x%04x = %d\n", dummy, dummy));

	dummy = read650(pChan, M77_EFR_OFFSET);
	DBGWRT_3 ((DBH, " EFR: 0x%02x\n", (u_int8) dummy));

	dummy = readASRxFL(pChan, M77_ASR_OFFSET);
	DBGWRT_3 ((DBH, " ASR: 0x%02x\n", (u_int8) dummy));

	dummy = readASRxFL(pChan, M77_RFL_OFFSET);
	DBGWRT_3 ((DBH, " RFL: 0x%02x\n", (u_int8) dummy));

	dummy = readASRxFL(pChan, M77_TFL_OFFSET);
	DBGWRT_3 ((DBH, " TFL: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_ACR_OFFSET);
	DBGWRT_3 ((DBH, " ACR: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_CPR_OFFSET);
	DBGWRT_3 ((DBH, " CPR: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_TCR_OFFSET);
	DBGWRT_3 ((DBH, " TCR: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_TTL_OFFSET);
	DBGWRT_3 ((DBH, " TTL: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_RTL_OFFSET);
	DBGWRT_3 ((DBH, " RTL: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_FCL_OFFSET);
	DBGWRT_3 ((DBH, " FCL: 0x%02x\n", (u_int8) dummy));

	dummy = readICR (pChan, M77_FCH_OFFSET);
	DBGWRT_3 ((DBH, " FCH: 0x%02x\n", (u_int8) dummy));

	switch( m77DevData[pChan->device_num]->modId ){
	case MOD_ID_M45N:
	{
		DBGWRT_3 ((DBH, " M45N PLD Register:\n"));
		if( pChan->ch_num > 3 )
			controllerNum = 1;
		dummy = MREAD_D16 ( m77DevData[pChan->device_num]->ma,
							M45N_TCR1_REG + controllerNum * M45N_CTRL_REG_BLOCK_SIZE );
		DBGWRT_3 ((DBH, "   TCR: 0x%02x\n", (u_int8) dummy));
		break;
	}
	case MOD_ID_M69N:
	{
		DBGWRT_3 ((DBH, " M69N PLD Register:\n"));
		break;
	}
	default: /* M77 */
	{
		DBGWRT_3 ((DBH, " M77 PLD Register:\n"));
		dummy = MREAD_D16 ( m77DevData[pChan->device_num]->ma,
							M77_DCR_REG_BASE + (2 * pChan->ch_num));
		DBGWRT_3 ((DBH, "   DCR: 0x%02x\n", (u_int8) dummy));
		break;
	}
	}

	dummy = MREAD_D16 (m77DevData[pChan->device_num]->ma,
					   M77_IRQ_REG + controllerNum * M45N_CTRL_REG_BLOCK_SIZE);
	DBGWRT_3 ((DBH, "   IRQ: 0x%02x\n", (u_int8) dummy));

	intUnlock(lockKey);
	taskUnlock();

}
#endif /* DBG */


