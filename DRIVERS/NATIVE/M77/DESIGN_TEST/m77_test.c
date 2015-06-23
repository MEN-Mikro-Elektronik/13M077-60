/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m77_test.c
 *      Project: M77 VxWorks Driver
 *
 *       Author: ag
 *        $Date: 2007/06/04 15:03:33 $
 *    $Revision: 1.5 $
 *
 *  Description: M77 dirver test module
 *
 *
 *     Required: MDIS, BBIS
 *     Switches:
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m77_test.c,v $
 * Revision 1.5  2007/06/04 15:03:33  CKauntz
 * added:
 *   - m77Testck test for M45N/M69N/M77 with diverse hardware options
 *   - m77TestSend test for M45N/M69N/M77 with several options
 * modified:
 *   - m45nHs auto hanshakes adapted to each module
 *
 * Revision 1.4  2006/08/22 10:25:17  cs
 * added:
 *    - m77Hs test for M45N/M69N handshakes
 *
 * Revision 1.3  2005/05/13 14:09:06  AGromann
 * changed interface of m77Klima tool
 * now read out M77 error byte if m77Klima test fails
 * now tools print out revision numbers
 *
 * Revision 1.2  2003/10/31 11:58:27  Agromann
 * updated
 *
 * Revision 1.1  2002/07/18 12:05:21  agromann
 * Initial Revision
 *
 *
 *---------------------------------------------------------------------------
 * (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
 ****************************************************************************/

#include <stdio.h>
#include <string.h>
#include <vxWorks.h>
#include <taskLib.h>
#include <ioLib.h>
#include <sysLib.h>
#include <MEN/men_typs.h>
#include <MEN/usr_oss.h>
#include <MEN/oss.h>
#include <MEN/dbg.h>
#include <MEN/desc.h>
#include <MEN/maccess.h>

#include "../m77_drv.h"

/*--------------------------------------+
|   DEFINES                             |
+--------------------------------------*/

#undef M77_VERBOSE

#define M77_TEST_REVISION  "$Revision: 1.5 $"

#define M77_MAX_DUMMY  2

#define	SERIAL_BUF_SIZE		2048
#define FILE_BUF_SIZE       140
#define STD_BAUD			9600
#define M77_KLIMA_BAUD      1152000
#define M77_OUT_BUFF_SIZE  0x1000 /* 4k */

/*-----------------------------------------+
|  GLOBALS                                 |
+------------------------------------------*/
long m77_testIn = 0;

int m77_01_err = 0;
int m77_10_err = 0;
int m77_23_err = 0;
int m77_32_err = 0;

int m77_01_run = 1;
int m77_10_run = 1;
int m77_23_run = 1;
int m77_32_run = 1;


/*-----------------------------------------+
|  STATICS                                 |
+------------------------------------------*/
static char outPattern[]=

"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -"
" the quick brown fox jumps over the lazy dog\r\n"
"the quick brown fox jumps over the lazy dog -"
" THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG\r\n"
"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -"
" the quick brown fox jumps over the lazy dog\r\n"
"the quick brown fox jumps over the lazy dog -"
" THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG\r\n"
"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -"
" the quick brown fox jumps over the lazy dog\r\n"
"the quick brown fox jumps over the lazy dog -"
" THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG\r\n"
"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -"
" the quick brown fox jumps over the lazy dog\r\n"
"the quick brown fox jumps over the lazy dog -"
" THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG\r\n"
"THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -"
" the quick brown fox jumps over the lazy dog\r\n"
"the quick brown fox jumps over the lazy dog -"
" THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG\r\n";


/*-----------------------------------------+
|  PROTOTYPES                              |
+------------------------------------------*/
void showActivity(void);
int m77SerTrans( int outFd, int inFd, int baud);


int m77Test(int mode)

{
	int fd;
	int i;
	char dummyBuff[M77_MAX_DUMMY];

	if (ERROR == (fd = open("/m77_0/0", O_RDWR, 0644))) {
		printf(" *** m77Test: Error opening /m77_0/0\n");
		return ERROR;
	}

	switch (mode) {


		case 0:
			printf("\n\n Output on fd = %d  ", fd);

			for (i = 0; i < 500; i++) {
				if (ERROR == write(fd, "a", 1)) {
					printf(" *** m77Test: Error writing to /m77_0/0\n");
					return ERROR;
				}
				taskDelay(1);
				showActivity();
			}

			printf("\n");
			break;

		case 1:

			printf("\n\n Driver Loopback\n");
			while(1) {
				read(fd, dummyBuff, 1);
				write(fd, dummyBuff, 1);
			}
			break;

		case 2:

			printf("\n\nReceived characters are echoed below:\n\n");

			while(1) {
				if (1 == read(fd, dummyBuff, 1))
					printf("%c\n", dummyBuff[0]);
			}

			break;
	}

	close(fd);
	return OK;
}


/********************************* m77SerTrans ********************************
 *
 *  Description: Check serial lines input and output
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor of transmitting device
 *                inFd    file descriptor of receiving device
 *                baud    baudrate for transmission
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77SerTrans( int outFd, int inFd, int baud)
{
char	*inBuf=NULL;
int		error;
int		retVal=-1;
int		numOutBytes;
int		numInBytes=0;
ULONG   timeoutTick;

	printf("=== Serial Test TX-Fd = %d, RX-Fd = %d baud = %d\n", outFd, inFd, baud);

	if( !outFd )
	{
		fprintf(stderr, "\n*** out file descriptor missing (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	if( !inFd )
	{
		fprintf(stderr, "\n*** in file descriptor missing (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	inBuf=malloc( SERIAL_BUF_SIZE );
	if( !inBuf )
	{
		fprintf(stderr, "\n*** Not enough memory (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	error = ioctl( outFd, FIOFLUSH, 0 );
	error |= ioctl( inFd, FIOFLUSH, 0 );

	error = 0;


	/* set options to RAW to avoid echo */
	(void) ioctl (outFd, FIOSETOPTIONS, OPT_RAW);
	(void) ioctl (inFd, FIOSETOPTIONS, OPT_RAW);

	/* set baudrate; if invalid set default */
	if (OK != ioctl (outFd, FIOBAUDRATE, baud)) {
		ioctl (outFd, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
	if (OK != ioctl (inFd, FIOBAUDRATE, baud)) {
		ioctl (inFd, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}

	numOutBytes = write( outFd, outPattern, sizeof(outPattern) );
	if( numOutBytes != sizeof(outPattern) )
	{
		fprintf(stderr, "\n*** write failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	/* wait until characters are received */
	timeoutTick = (ULONG) (tickGet() + (4 * sysClkRateGet()));
	while ( numInBytes != numOutBytes  )
	{
		taskDelay(1);
		error |= ioctl( inFd, FIONREAD, (int)&numInBytes );
		if( error )
		{
			fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n", __FILE__, __LINE__ );
			goto CLEANUP;
		}/*if*/
		if (timeoutTick < tickGet()) {
			break;
		}

	} /* while */


	error |= ioctl( inFd, FIONREAD, (int)&numInBytes );
	if( error )
	{
		fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n", __FILE__, __LINE__);
		goto CLEANUP;
	}/*if*/

	if( numInBytes != numOutBytes )
	{
		fprintf(stderr, "\n*** number of received bytes=%d not equal %d number of sent bytes\n",
				numInBytes, numOutBytes );
		goto CLEANUP;
	}/*if*/

	numInBytes = read( inFd, inBuf, numInBytes );
	if( numInBytes != numOutBytes )
	{
		fprintf(stderr, "\n*** number of read bytes=%d not equal %d number of sent bytes\n",
				numInBytes, numOutBytes );
		goto CLEANUP;
	}/*if*/

	/* check pattern */
	error = strncmp( inBuf, outPattern, numOutBytes );
	if( error )
	{
		fprintf(stderr, "\n*** pattern mismatch\n" );
		fprintf(stderr, "\n\n%s\n", inBuf);
		error = -1;
		goto CLEANUP;
	}/*if*/

	/* say OK */
	retVal = 0;

CLEANUP:
	free( inBuf);
	if( !retVal ) {
#ifdef M77_VERBOSE
		printf(stderr, "%d ==> OK\n", numInBytes);
#endif /* M77_VERBOSE */
	}
	else
		fprintf(stderr, " ==> FAILED %d -> %d\n", outFd, inFd);
	return( retVal );
}

/********************************* m77SerStress *******************************
 *
 *  Description: Transmitt testpattern from outFd to inFd
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor of transmitting device
 *                inFd    file descriptor of receiving device
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77SerStress(int outFd, int inFd)
{
char	*inBuf=NULL;
int		error = 0;
int		retVal=-1;
int		numOutBytes;
int		numInBytes=0;
ULONG   timeoutTick;

/*	printf("=== Serial Test TX-Fd = %d, RX-Fd = %d baud = %d\n", outFd, inFd, baud); */

	if( !outFd )
	{
		fprintf(stderr, "\n*** out file descriptor missing (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	if( !inFd )
	{
		fprintf(stderr, "\n*** in file descriptor missing (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	inBuf=malloc( SERIAL_BUF_SIZE );
	if( !inBuf )
	{
		fprintf(stderr, "\n*** Not enough memory (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	numOutBytes = write( outFd, outPattern, sizeof(outPattern) );
	if( numOutBytes != sizeof(outPattern) )
	{
		fprintf(stderr, "\n*** write failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	/* wait until characters are received */
	timeoutTick = (ULONG) (tickGet() + (4 * sysClkRateGet()));
	while ( numInBytes != numOutBytes  )
	{
		taskDelay(1);
		error |= ioctl( inFd, FIONREAD, (int)&numInBytes );
		if( error )
		{
			fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n", __FILE__, __LINE__ );
			goto CLEANUP;
		}/*if*/
		if (timeoutTick < tickGet()) {
			break;
		}

	} /* while */


	error |= ioctl( inFd, FIONREAD, (int)&numInBytes );
	if( error )
	{
		fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n", __FILE__, __LINE__);
		goto CLEANUP;
	}/*if*/

	if( numInBytes != numOutBytes )
	{
		fprintf(stderr, "\n*** number of received bytes=%d not equal %d number of sent bytes\n",
				numInBytes, numOutBytes );
		goto CLEANUP;
	}/*if*/

	numInBytes = read( inFd, inBuf, numInBytes );
	if( numInBytes != numOutBytes )
	{
		fprintf(stderr, "\n*** number of read bytes=%d not equal %d number of sent bytes\n",
				numInBytes, numOutBytes );
		goto CLEANUP;
	}/*if*/

	/* check pattern */
	error = strncmp( inBuf, outPattern, numOutBytes );
	if( error )
	{
		fprintf(stderr, "\n*** pattern mismatch\n" );
		fprintf(stderr, "\n\n%s\n", inBuf);
		error = -1;
		goto CLEANUP;
	}/*if*/

	/* say OK */
	retVal = 0;

CLEANUP:
	free( inBuf);
	if(retVal)
		fprintf(stderr, " ==> FAILED %d -> %d\n", outFd, inFd);

	return( retVal );
}

/********************************* m77SendFile ********************************
 *
 *  Description: Check serial lines input and output
 *
 *----------------------------------------------------------------------------
 *  Input......:  fileName   name of file to send
 *                device     device name to send file to
 *                baud       baudrate
 *
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77SendFile( char * fileName, char * device, int baud)
{
int		error;
int		retVal=-1;
int		numOutBytes;
int     numQBytes;
int     readBytes;
FILE   *fp;
long    fileSize;
int     outFd;
int     send;
char   *fileBuff;

	if (fileName == NULL || device == NULL || baud == 0) {

		printf("\n\nUsage: m77SendFile [fileName], [devName], [baud]\n"
		           "       fileName: Name of input file\n"
		           "       devName:  Name of output device\n"
		           "       baud:     transmission baudrate\n");

		return (-1);
	}

	printf("\n\n\n ++++++++ m77SendFile ++++++++\n");
	printf("\n %s\n\n", M77_TEST_REVISION);
	printf("===> Sending %s to %s \n", fileName, device);

	fp = fopen(fileName, "r");

	if (fp == NULL) {
		fprintf(stderr, "\n*** Error opening file %s (%s line %d)\n",
		fileName, __FILE__, __LINE__ );
		return( -1 );

	}

	outFd = open (device, O_RDWR, 0644);

	if( !outFd )
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	fseek (fp, 0, SEEK_END);
	fileSize = ftell(fp);
	fseek (fp, 0, SEEK_SET);

	if (fileSize < 0) {
		fprintf(stderr, "\n*** Error getting file size (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}

	fileBuff = malloc( FILE_BUF_SIZE );
	if( !fileBuff )
	{
		fprintf(stderr, "\n*** Not enough memory (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/


	/*------------------------- ----+
	|  Test FIFO-Mode switching     |
	+------------------------------*/

	/* switch to no FIFO */
	error = ioctl( outFd, M77_NO_FIFO, 0 );
	if( error )
	{
		fprintf(stderr, "\n*** M77_NO_FIFO failed: error = 0x%08x (%s line %d)\n", error, __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	error = ioctl( outFd, M77_TX_FIFO_LEVEL, 64 );
	if( error )
	{
		fprintf(stderr, "\n*** M77_RX_FIFO_LEVEL failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/


	error = ioctl( outFd, FIOFLUSH, 0 );

	/* SIO_HW_OPTS_SET to 8E1 */
	ioctl(outFd, SIO_HW_OPTS_SET, 0x4f);

	/* set options to RAW to avoid echo */
	(void) ioctl (outFd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM); /* Xon/Xoff flow controll */

	/* set baudrate if invalid set detfault */
	if (OK != ioctl (outFd, FIOBAUDRATE, baud))
		ioctl (outFd, FIOBAUDRATE, STD_BAUD);

	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed (%s line %d)\n", __FILE__, __LINE__);
		goto CLEANUP;
	}/*if*/


	while (fileSize) {
		showActivity();
		ioctl( outFd, FIONWRITE, (int)&numQBytes );
		if (FILE_BUF_SIZE < (M77_OUT_BUFF_SIZE - numQBytes))
			send = FILE_BUF_SIZE;
		else
			send = M77_OUT_BUFF_SIZE - numQBytes;

		readBytes = fread (fileBuff, sizeof(char), send, fp);
		numOutBytes = write( outFd, fileBuff, readBytes );
		showActivity();
		if (readBytes != numOutBytes) {
			fprintf(stderr, "\n*** Transmission error (%s line %d)\n", __FILE__, __LINE__ );
			goto CLEANUP;

		}
		fileSize -= readBytes;
	}


	retVal = 0;

CLEANUP:
	free( fileBuff);
	fclose (fp);
	close(outFd);
	if( !retVal )
		fprintf(stderr, "==> DONE\n");
	else
		fprintf(stderr, " ==> FAILED\n");
	return( retVal );

}

/********************************* m77ReceiveFile *****************************
 *
 *  Description: Check serial lines input and output
 *
 *----------------------------------------------------------------------------
 *  Input......:  fileName   name of file to write to
 *                device     name of device to receive from
 *                baud       baudrate
 *
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77ReceiveFile( char * fileName, char * device, int baud)
{
int		error;
int		retVal=-1;
int		numOutBytes;
int     numQBytes;
int     readBytes;
FILE   *fp;
int     inFd;
int     receive;
char   *fileBuff;
int     errFlag;

	if (fileName == NULL || device == NULL || baud == 0) {

		printf("\n\nUsage: m77ReceiveFile [fileName], [devName], [baud]\n"
		           "       fileName: Name of file to write to\n"
		           "       devName:  Name of device to receive from\n"
		           "       baud:     baudrate\n");
		return (-1);
	}

	printf("\n\n\n ++++++++ m77ReceiveFile ++++++++\n");
	printf("\n %s\n\n", M77_TEST_REVISION);

	printf("===> Write received data from %s to file %s \n", device, fileName);

	fp = fopen(fileName, "w");

	if (fp == NULL) {
		fprintf(stderr, "\n*** Error opening file %s (%s line %d)\n",
		fileName, __FILE__, __LINE__ );
		return( -1 );

	}

	inFd = open (device, O_RDWR, 0644);

	if( !inFd )
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/


	fileBuff = malloc( FILE_BUF_SIZE );
	if( !fileBuff )
	{
		fprintf(stderr, "\n*** Not enough memory (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/


	/*------------------------------+
	|  Test FIFO-Mode switching     |
	+------------------------------*/
	/* switch to no FIFO */
	error = ioctl( inFd, M77_NO_FIFO, 0 );
	if( error )
	{
		fprintf(stderr, "\n*** M77_NO_FIFO failed: error = 0x%08x (%s line %d)\n", error, __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	error = ioctl( inFd, M77_RX_FIFO_LEVEL, 64 );
	if( error )
	{
		fprintf(stderr, "\n*** M77_RX_FIFO_LEVEL failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	error = ioctl( inFd, FIOFLUSH, 0 );
	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/

	/* SIO_HW_OPTS_SET to 8E1 */
	ioctl(inFd, SIO_HW_OPTS_SET, 0x4f);

	/* set options to RAW to avoid echo */
	(void) ioctl (inFd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM);  /* Xon Xoff flow controll OPT_CRMOD? */

	/* set baudrate if invalid set detfault */
	if (OK != ioctl (inFd, FIOBAUDRATE, baud))
		ioctl (inFd, FIOBAUDRATE, STD_BAUD);


	while (UOS_KeyPressed() != 27 /* ESC */) {
		taskDelay(20);

		ioctl( inFd, FIONREAD, (int)&numQBytes );
		if (FILE_BUF_SIZE < numQBytes)
			receive = FILE_BUF_SIZE;
		else
			receive = numQBytes;

		if (receive == 0){
			taskDelay(1);
		}
		else {
			showActivity();
			readBytes = read( inFd, fileBuff, receive );
			numOutBytes = fwrite (fileBuff, sizeof(char), readBytes, fp);
			m77_testIn += readBytes;
			if (receive != numOutBytes) {
				fprintf(stderr, "\n*** Reciev error (%s line %d)\n", __FILE__, __LINE__ );
				goto CLEANUP;

			}
		}

	}


	retVal = 0;

CLEANUP:

	ioctl(inFd, M77_GET_LAST_ERR, (int) &errFlag);
	if (errFlag) {
		printf("\n Error Flag = 0x%02x\n", (u_int8) errFlag);
		error = 1;
	}

	free( fileBuff);
	fclose (fp);
	close(inFd);
	if( !retVal ) {
		printf("==> DONE\n");
	}
	else
		fprintf(stderr, "==> FAILED\n");
	return( retVal );
}

/********************************* xoff  **************************************
 *
 *  Description: Send Xoff character
 *
 *----------------------------------------------------------------------------
 *  Input......:  fd         file descriptor of device
 *
 *  Output.....:  return 0
 *
 *  Globals....:
 *****************************************************************************/
int xoff(int fd)
{
	int bytesInQueue;
	u_int8 dummy[]= {0x13, 0};

	ioctl( fd, FIONWRITE, (int)&bytesInQueue );

	if (M77_OUT_BUFF_SIZE - bytesInQueue) {
		write( fd, dummy, 1 );
	}
	else {
	 printf("\n Send buffer of device with fd = %d full !", fd);
	}
	return 0;
}

int m77Fd;

/********************************* m77Redirect *******************************
 *
 *  Description: Redirect vxWorks console to M77 channel.
 *               Path to M77 is opened.
 *
 *----------------------------------------------------------------------------
 *  Input......:  channel   M77 channel to redirect console
 *
 *  Output.....:  return    ERROR or OK
 *
 *  Globals....:  m77Fd
 *****************************************************************************/
STATUS m77Redirect(char* tyName)
{

	/*----------------+
	| wait for input  |
	+----------------*/

	m77Fd = open (tyName, O_RDWR, 0);

	if (m77Fd < 0) {
		printf("*** m77Redirect: Error opening %s", tyName);
		return ERROR;
	}

    (void) ioctl (m77Fd, 0x106, 7); /* RS232 Mode */
	(void) ioctl (m77Fd, FIOBAUDRATE, 9600);
    (void) ioctl (m77Fd, FIOSETOPTIONS, OPT_TERMINAL);
    ioGlobalStdSet (STD_IN,  m77Fd);
    ioGlobalStdSet (STD_OUT, m77Fd);
    ioGlobalStdSet (STD_ERR, m77Fd);

	return OK;
}

/********************************* m77Direct **********************************
 *
 *  Description: Redirect vxWorks console to standard fd.
 *               Path to M77 is closed.
 *
 *----------------------------------------------------------------------------
 *  Input......:  fd        standard fd to direct console to
 *
 *  Output.....:  return    ERROR or OK
 *
 *  Globals....:  m77Fd
 *****************************************************************************/
STATUS m77Direct(int fd)
{

	ioGlobalStdSet (STD_IN,  fd);
    ioGlobalStdSet (STD_OUT, fd);
    ioGlobalStdSet (STD_ERR, fd);

    close (m77Fd);

	return OK;
}

/********************************* m77Klima ***********************************
 *
 *  Description: Check serial lines input and output in RS422 HD mode
 *               Prior to calling this tool the M77 devices must be created
 *               using m77Drv or m77Drv_sw.
 *
 *----------------------------------------------------------------------------
 *  Input......:  dev        base name of M77 devices eg. /m77_0 or /m77sw_0
 *                m77Desc    M77-Module descriptor
 *                numRuns    number of test passes (0: run endless)
 *                baudrate   baudrate to use for test
 *                           (0: default M77_KLIMA_BAUD)
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77Klima(char* dev, int numRuns, unsigned long baudrate)
{

	int ch0Fd;
	int ch1Fd;
	int ch2Fd;
	int ch3Fd;
	int numReceived;
	int error;
	int errorSuccess = ERROR;
	int runCount = 0;
	char receiveBuff[2];
	char devName[16];
	int  errFlag = 0;

	/* if baudrate = 0 set baudrate to default */
	if( !baudrate ) {
		baudrate = M77_KLIMA_BAUD;
	}

	printf("\n\n\n ++++++++ M77 Klima Test ++++++++\n");
	printf("\n %s\n\n", M77_TEST_REVISION);
	printf("\n Baudrate: %ld\n\n", baudrate);
	printf(" Test Running  ");
	sprintf( devName, "%s/0", dev );
	ch0Fd = open (devName, O_RDWR, 0644);
	sprintf( devName, "%s/1", dev );
	ch1Fd = open (devName, O_RDWR, 0644);
	sprintf( devName, "%s/2", dev );
	ch2Fd = open (devName, O_RDWR, 0644);
	sprintf( devName, "%s/3", dev );
	ch3Fd = open (devName, O_RDWR, 0644);

	if( ERROR == ch0Fd || ERROR == ch1Fd || ERROR == ch2Fd || ERROR == ch3Fd)
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n",
		__FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	ioctl (ch0Fd, M77_PHYS_INT_SET, M77_RS422_HD);
	ioctl (ch1Fd, M77_PHYS_INT_SET, M77_RS422_HD);
	ioctl (ch2Fd, M77_PHYS_INT_SET, M77_RS422_HD);
	ioctl (ch3Fd, M77_PHYS_INT_SET, M77_RS422_HD);

	/* wait until spurious characters disappear */
	taskDelay(5);

	ioctl( ch0Fd, FIOFLUSH, 0 );
	ioctl( ch1Fd, FIOFLUSH, 0 );
	ioctl( ch2Fd, FIOFLUSH, 0 );
	ioctl( ch3Fd, FIOFLUSH, 0 );

	while (1) {

		if (0 != m77SerTrans( ch0Fd, ch1Fd, (int) baudrate)) {
			fprintf(stderr, "\n*** Error Transmission failed ch0Fd -> ch1Fd\n");
			goto Error;
		}
		error = ioctl( ch0Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch0Fd -> ch1Fd\n");
			goto Error;
		}
		if (numReceived) {
			fprintf(stderr, "\n*** Error Echo-Suppression failed ch0Fd -> ch1Fd:\n"
			"Received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch0Fd, receiveBuff, 1);
				printf("0x%02x\n", receiveBuff[0]);
			}
			goto Error;

		}
		showActivity();

		if (0 != m77SerTrans( ch1Fd, ch0Fd, (int) baudrate)) {
			fprintf(stderr, "\n*** Error Transmission failed ch1Fd -> ch0Fd\n");
			goto Error;
			return( -1 );
		}
		error = ioctl( ch1Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch1Fd -> ch0Fd\n");
			goto Error;
		}
		if (numReceived) {
			fprintf(stderr, "\n*** Error Echo-Suppression failed ch1Fd -> ch0Fd:\n"
			"Received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch1Fd, receiveBuff, 1);
				printf("0x%02x\n", receiveBuff[0]);
			}
			goto Error;

		}
		showActivity();

		if (0 != m77SerTrans( ch2Fd, ch3Fd, (int) baudrate)) {
			fprintf(stderr, "\n*** Error Transmission failed ch2Fd -> ch3Fd\n");
			goto Error;
		}
		error = ioctl( ch2Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch2Fd -> ch3Fd\n");
			goto Error;
		}
		if (numReceived) {
			fprintf(stderr, "\n*** Error Echo-Suppression failed ch2Fd -> ch3Fd:\n"
			"Received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch2Fd, receiveBuff, 1);
				printf("0x%02x\n", receiveBuff[0]);
			}
			goto Error;

		}
		showActivity();

		if (0 != m77SerTrans( ch3Fd, ch2Fd, (int) baudrate)) {
			fprintf(stderr, "\n*** Error Transmission failed ch3Fd -> ch2Fd\n");
			goto Error;
		}
		error = ioctl( ch3Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch3Fd -> ch2Fd\n");
			goto Error;
		}
		if (numReceived) {
			fprintf(stderr, "\n*** Error Echo-Suppression failed ch3Fd -> ch2Fd:\n"
			"Received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch3Fd, receiveBuff, 1);
				printf("0x%02x\n", receiveBuff[0]);
			}
			goto Error;

		}
		showActivity();


		if (numRuns != 0)
			if (numRuns <= runCount++)
				goto End;


	}
End:
	/* say OK */
	errorSuccess = OK;

Error:
	ioctl(ch0Fd, M77_GET_LAST_ERR, (int) &errFlag);
	if (errFlag) {
		printf("\nCH0: Error Flag = 0x%02x\n", (u_int8) errFlag);
	}
	ioctl(ch1Fd, M77_GET_LAST_ERR, (int) &errFlag);
	if (errFlag) {
		printf("\nCH1: Error Flag = 0x%02x\n", (u_int8) errFlag);
	}
	ioctl(ch2Fd, M77_GET_LAST_ERR, (int) &errFlag);
	if (errFlag) {
		printf("\nCH2: Error Flag = 0x%02x\n", (u_int8) errFlag);
	}
	ioctl(ch3Fd, M77_GET_LAST_ERR, (int) &errFlag);
	if (errFlag) {
		printf("\nCH3: Error Flag = 0x%02x\n", (u_int8) errFlag);
	}

	close (ch0Fd);
	close (ch1Fd);
	close (ch2Fd);
	close (ch3Fd);

	if (errorSuccess == OK) {
		printf("\n -> TEST OK\n");
	}
	else {
		printf("\n -> TEST FAILED\n");
		printf("\nPlease check if driver is run without Debug-Informations\n");

	}

	return errorSuccess;
}


/********************************* m77CheckReceived ***************************
 *
 *  Description: Check for reception of spurious characters.
 *               Read all channels and report any received character
 *
 *----------------------------------------------------------------------------
 *  Input......:  bbDesc     baseboard descriptor
 *                m77Desc    M77-Module descriptor
 *                numRuns    number of runns (0 = endless)
 *
 *
 *  Output.....: return      OK or ERROR
 *
 *  Globals....:
 *****************************************************************************/
int m77CheckReceived(DESC_SPEC * bbDesc , DESC_SPEC * m77Desc, int numRuns)
{

	int ch0Fd;
	int ch1Fd;
	int ch2Fd;
	int ch3Fd;
	int numReceived;
	int error;
	int errorSuccess = ERROR;
	int runCount = 0;
	static int devsCreated = 1;
	char receiveBuff[2];

	if (devsCreated) {
		m77Drv (bbDesc, m77Desc);
		devsCreated = 0;
	}

	printf("\n\n\n ++++++++ m77CheckReceived ++++++++\n");
	printf("\n %s\n\n", M77_TEST_REVISION);
	printf(" Test Running  ");

	ch0Fd = open ("/m77_0/0", O_RDWR, 0644);
	ch1Fd = open ("/m77_0/1", O_RDWR, 0644);
	ch2Fd = open ("/m77_0/2", O_RDWR, 0644);
	ch3Fd = open ("/m77_0/3", O_RDWR, 0644);

	if( ERROR == ch0Fd || ERROR == ch1Fd || ERROR == ch2Fd || ERROR == ch3Fd)
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	/* flush all channels */
	ioctl( ch0Fd, FIOFLUSH, 0 );
	ioctl( ch1Fd, FIOFLUSH, 0 );
	ioctl( ch2Fd, FIOFLUSH, 0 );
	ioctl( ch3Fd, FIOFLUSH, 0 );


	while (1) {

		error = ioctl( ch0Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch0Fd\n");
			goto Error;
		}
		if (numReceived) {
			printf("CH0 received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch0Fd, receiveBuff, 1);
				/* printf("0x%02x\n", (char) receiveBuff[0]); */
			}
		}
		showActivity();

		error = ioctl( ch1Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch1Fd\n");
			goto Error;
		}
		if (numReceived) {
			printf("CH1 received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch1Fd, receiveBuff, 1);
				/* printf("0x%02x\n", receiveBuff[0]); */
			}
		}
		showActivity();

		error = ioctl( ch2Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch2Fd\n");
			goto Error;
		}
		if (numReceived) {
			printf("CH2 received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch2Fd, receiveBuff, 1);
				/* printf("0x%02x\n", receiveBuff[0]); */
			}
		}
		showActivity();


		error = ioctl( ch3Fd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch3Fd\n");
			goto Error;
		}
		if (numReceived) {
			printf("CH3 received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( ch3Fd, receiveBuff, 1);
				/* printf("0x%02x\n", receiveBuff[0]); */
			}
		}
		showActivity();


		if (numRuns != 0)
			if (numRuns <= runCount++)
				goto End;


	}
End:
	/* say OK */
	errorSuccess = OK;

Error:

	close (ch0Fd);
	close (ch1Fd);
	close (ch2Fd);
	close (ch3Fd);

	if (errorSuccess == OK) {
		printf("\n -> TEST OK\n");
	}
	else {
		printf("\n -> TEST FAILED\n");

	}

	return errorSuccess;
}

/********************************* m77_01_task ********************************
 *
 *  Description: Task wich transmitts from CH0 -> CH1
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor to transmit
 *                inFd    file descriptor to receive
 *
 *  Output.....:
 *
 *  Globals....:  m77_01_err, m77_01_run
 *****************************************************************************/
static void m77_01_task(int outFd, int inFd)
{

	while (m77_01_run++ && !m77_01_err) {
		m77_01_err = m77SerStress(outFd, inFd);
	}

}

/********************************* m77_10_task ********************************
 *
 *  Description: Task wich transmitts from CH1 -> CH0
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor to transmit
 *                inFd    file descriptor to receive
 *
 *  Output.....:
 *
 *  Globals....:  m77_10_err, m77_10_run
 *****************************************************************************/
static void m77_10_task(int outFd, int inFd)
{

	while (m77_10_run++ && !m77_10_err) {
		m77_10_err = m77SerStress(outFd, inFd);
	}

}

/********************************* m77_23_task ********************************
 *
 *  Description: Task wich transmitts from CH2 -> CH0
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor to transmit
 *                inFd    file descriptor to receive
 *
 *  Output.....:
 *
 *  Globals....:  m77_23_err, m77_23_run
 *****************************************************************************/
static void m77_23_task(int outFd, int inFd)
{

	while (m77_23_run++ && !m77_23_err) {
		m77_23_err = m77SerStress(outFd, inFd);
	}

}

/********************************* m77_32_task ********************************
 *
 *  Description: Task wich transmitts from CH1 -> CH0
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor to transmit
 *                inFd    file descriptor to receive
 *
 *  Output.....:
 *
 *  Globals....:  m77_32_err, m77_32_run
 *****************************************************************************/
static void m77_32_task(int outFd, int inFd)
{

	while (m77_32_run++ && !m77_32_err) {
		m77_32_err = m77SerStress(outFd, inFd);
	}

}

/********************************* m77Stress **********************************
 *
 *  Description: Transmitt and receive on all channels
 *
 *----------------------------------------------------------------------------
 *  Input......:  fileName   name of file to write to
 *                device     name of device to receive from
 *                baud       baudrate
 *
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77Stress(char * devName, int baud)
{

	int ch0Fd;
	int ch1Fd;
	int ch2Fd;
	int ch3Fd;
	int error = 0;
	int errorSuccess = ERROR;
	char dummayBuff[80];
	int chann = 0;
	int errFlag;


	if (devName == 0 || baud == 0) {
		printf("\n\nUsage: m77Stress [devName], [baud]\n"
		           "       devName:  Name of M77 device without channel number e.g. /m77_0/\n"
		           "       baud:     transmission baudrate\n");
	}

	/* setting test pattern to 0
	OSS_MemFill(OSS_VXWORKS_OS_HDL, sizeof(outPattern), (char*) outPattern, 0 );
	*/

	sprintf (dummayBuff, "%s%d", devName, chann++);
	ch0Fd = open (dummayBuff, O_RDWR, 0644);
	printf("\n Dev: %s ->Fd: %d\n", dummayBuff, ch0Fd);

	sprintf (dummayBuff, "%s%d", devName, chann++);
	ch1Fd = open (dummayBuff, O_RDWR, 0644);
	printf("\n Dev: %s ->Fd: %d\n", dummayBuff, ch1Fd);

	sprintf (dummayBuff, "%s%d", devName, chann++);
	ch2Fd = open (dummayBuff, O_RDWR, 0644);
	printf("\n Dev: %s ->Fd: %d\n", dummayBuff, ch2Fd);

	sprintf (dummayBuff, "%s%d", devName, chann++);
	ch3Fd = open (dummayBuff, O_RDWR, 0644);
	printf("\n Dev: %s ->Fd: %d\n", dummayBuff, ch3Fd);

	if( ERROR == ch0Fd || ERROR == ch1Fd || ERROR == ch2Fd || ERROR == ch3Fd)
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	m77_01_err = 0;
	m77_10_err = 0;
	m77_23_err = 0;
	m77_32_err = 0;

	m77_01_run = 1;
	m77_10_run = 1;
	m77_23_run = 1;
	m77_32_run = 1;

	if (OK != ioctl (ch0Fd, FIOBAUDRATE, baud)) {
		ioctl (ch0Fd, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
	if (OK != ioctl (ch1Fd, FIOBAUDRATE, baud))
		ioctl (ch1Fd, FIOBAUDRATE, STD_BAUD);

	if (OK != ioctl (ch2Fd, FIOBAUDRATE, baud))
		ioctl (ch2Fd, FIOBAUDRATE, STD_BAUD);

	if (OK != ioctl (ch3Fd, FIOBAUDRATE, baud))
		ioctl (ch3Fd, FIOBAUDRATE, STD_BAUD);


	ioctl(ch0Fd, SIO_HW_OPTS_SET, 0x0e);
	ioctl(ch1Fd, SIO_HW_OPTS_SET, 0x0e);
	ioctl(ch2Fd, SIO_HW_OPTS_SET, 0x0e);
	ioctl(ch3Fd, SIO_HW_OPTS_SET, 0x0e);

	ioctl( ch0Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch1Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch2Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch3Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );

	ioctl( ch0Fd, FIOFLUSH, 0 );
	ioctl( ch1Fd, FIOFLUSH, 0 );
	ioctl( ch2Fd, FIOFLUSH, 0 );
	ioctl( ch3Fd, FIOFLUSH, 0 );

 	sp (m77_01_task, ch0Fd, ch1Fd, 0, 0, 0, 0, 0, 0, 0);
/*	taskDelay(2); */
 	sp (m77_10_task, ch1Fd, ch0Fd, 0, 0, 0, 0, 0, 0, 0);
/*	taskDelay(5); */
 	sp (m77_23_task, ch2Fd, ch3Fd, 0, 0, 0, 0, 0, 0, 0);
/* 	taskDelay(1); */
 	sp (m77_32_task, ch3Fd, ch2Fd, 0, 0, 0, 0, 0, 0, 0);

	taskDelay(10);

	printf("\n\n\n ++++++++ M77 Stress Test ++++++++\n");
	printf("\n %s\n\n", M77_TEST_REVISION);
	printf(" Transmitting full duplex CH0 <-> CH1, CH2 <-> CH3\n"
	       " Baudrate = %d\n"
	       " Test Running  ", baud);


	while (UOS_KeyPressed() != 27 /* ESC */) {

		if (m77_01_err) {
			printf("\n m77SerTrans CH0 -> CH1 failed\n");
			error = 1;
		}

		if (m77_10_err) {
			printf("\n m77SerTrans CH1 -> CH0 failed\n");
			error = 1;
		}

		if (m77_23_err) {
			printf("\n m77SerTrans CH2 -> CH3 failed\n");
			error = 1;
		}

		if (m77_32_err) {
			printf("\n m77SerTrans CH3 -> CH2 failed\n");
			error = 1;
		}

		ioctl(ch0Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH0 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		ioctl(ch1Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH1 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		ioctl(ch2Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH2 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		ioctl(ch3Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH3 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		if (error) {
			goto Error;
		}

		taskDelay(30);
		showActivity();
	}

	/* say OK */
	errorSuccess = OK;

Error:

	m77_01_run = 0;
	m77_10_run = 0;
	m77_23_run = 0;
	m77_32_run = 0;

	/* wait for task to be stopped */
	taskDelay(60);

	close (ch0Fd);
	close (ch1Fd);
	close (ch2Fd);
	close (ch3Fd);

	if (errorSuccess == OK) {
		printf("\n -> TEST OK\n");
	}
	else {
		printf("\n -> TEST FAILED\n");

	}

	return errorSuccess;
}


/********************************* m77StressP10 **********************************
 *
 *  Description: Transmitt and receive on all channels
 *
 *----------------------------------------------------------------------------
 *  Input......:  fileName   name of file to write to
 *                device     name of device to receive from
 *                baud       baudrate
 *
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m77StressP10(int baud)
{

	int ch0Fd;
	int ch1Fd;
	int ch2Fd;
	int ch3Fd;
	int error = 0;
	int errorSuccess = ERROR;
/* 	char dummayBuff[80]; */
	int errFlag;


	if (baud == 0) {
		printf("\n\nUsage: m77Stress [baud]\n"
		           "       baud:     transmission baudrate\n");
	}

	/* setting test pattern to 0
	OSS_MemFill(OSS_VXWORKS_OS_HDL, sizeof(outPattern), (char*) outPattern, 0 );
	*/

	ch0Fd = open ("/p11_0/0", O_RDWR, 0644);

	ch1Fd = open ("/m77_0/0", O_RDWR, 0644);

	ch2Fd = open ("/p11_0/1", O_RDWR, 0644);

	ch3Fd = open ("/m77_0/1", O_RDWR, 0644);

	if( ERROR == ch0Fd || ERROR == ch1Fd || ERROR == ch2Fd || ERROR == ch3Fd)
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	m77_01_err = 0;
	m77_10_err = 0;
	m77_23_err = 0;
	m77_32_err = 0;

	m77_01_run = 1;
	m77_10_run = 1;
	m77_23_run = 1;
	m77_32_run = 1;

	ioctl(ch0Fd, SIO_HW_OPTS_SET, 0x0f);
	ioctl(ch1Fd, SIO_HW_OPTS_SET, 0x0e);
	ioctl(ch2Fd, SIO_HW_OPTS_SET, 0x0f);
	ioctl(ch3Fd, SIO_HW_OPTS_SET, 0x0e);

	ioctl( ch0Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch1Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch2Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );
	ioctl( ch3Fd, FIOSETOPTIONS, OPT_RAW | OPT_TANDEM );

	ioctl( ch0Fd, FIOFLUSH, 0 );
	ioctl( ch1Fd, FIOFLUSH, 0 );
	ioctl( ch2Fd, FIOFLUSH, 0 );
	ioctl( ch3Fd, FIOFLUSH, 0 );

 	sp (m77_01_task, ch0Fd, ch1Fd, baud, 0, 0, 0, 0, 0, 0);
/*	taskDelay(2); */
 	sp (m77_10_task, ch1Fd, ch0Fd, baud, 0, 0, 0, 0, 0, 0);
/*	taskDelay(5); */
	sp (m77_23_task, ch2Fd, ch3Fd, baud, 0, 0, 0, 0, 0, 0);
/* 	taskDelay(1); */
	sp (m77_32_task, ch3Fd, ch2Fd, baud, 0, 0, 0, 0, 0, 0);

	taskDelay(10);

	printf("\n\n\n ++++++++ M77 Stress Test ++++++++\n"
	              " Transmitting full duplex CH0 <-> CH1, CH2 <-> CH3\n"
	              " Baudrate = %d\n"
	              " Test Running  ", baud);


	while (UOS_KeyPressed() != 27 /* ESC */) {

		if (m77_01_err) {
			printf("\n m77SerTrans CH0 -> CH1 failed\n");
			error = 1;
		}

		if (m77_10_err) {
			printf("\n m77SerTrans CH1 -> CH0 failed\n");
			error = 1;
		}

		if (m77_23_err) {
			printf("\n m77SerTrans CH2 -> CH3 failed\n");
			error = 1;
		}

		if (m77_32_err) {
			printf("\n m77SerTrans CH3 -> CH2 failed\n");
			error = 1;
		}

		ioctl(ch1Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH1 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		ioctl(ch3Fd, M77_GET_LAST_ERR, (int) &errFlag);
		if (errFlag) {
			printf("\n Error Flag CH3 = 0x%02x\n", (u_int8) errFlag);
			error = 1;
		}

		if (error) {
			goto Error;
		}

		taskDelay(30);
		showActivity();
	}

	/* say OK */
	errorSuccess = OK;

Error:

	m77_01_run = 0;
	m77_10_run = 0;
	m77_23_run = 0;
	m77_32_run = 0;

	/* wait for task to be stopped */
	taskDelay(60);

	close (ch0Fd);
	close (ch1Fd);
	close (ch2Fd);
	close (ch3Fd);

	if (errorSuccess == OK) {
		printf("\n -> TEST OK\n");
	}
	else {
		printf("\n -> TEST FAILED\n");

	}

	return errorSuccess;
}

/*********************************** m77Hs ***********************************
 *
 *  Description: Test Handshake lines of M45N/M69N
 *
 *----------------------------------------------------------------------------
 *  Input......:  devName0   name of device 1
 *                devName1   name of device 2
 *                mode       test mode:
 *                           0: manual toggle of HS signals
 *                           1: no handshakes
 *                           2: XON/XOFF
 *                           3: RTS/CTS
 *                dir        direction:
 *                           0: send    (dev0)
 *                           1: receive (dev0)
 *                           2: bidirectional   (dev0<-->dev1)
 *                           3: driver loopback (dev0<-->dev1)
 *                num        number of passes
 *
 *  Output.....:
 *
 *  Globals....:
 *****************************************************************************/
int m45nHs(char *devName0, char *devName1, int mode, int baud, int dir, int num)
{
	int fd0, fd1, j;
	int retVal, end = num;
	fd0 = open ( devName0, O_RDWR, 0644);
	fd1 = open ( devName1, O_RDWR, 0644);
	if( fd0 == ERROR || fd1 == ERROR )
	{
		fprintf(stderr, "\n*** Error opening device (%s line %d)\n", __FILE__, __LINE__);
		return( -1 );
	}/*if*/

	ioctl (fd0, M77_REG_DUMP, 0);

	if( baud <= 0 ) baud = STD_BAUD;

	/* set baudrate if invalid set default */
	if (OK != ioctl (fd0, FIOBAUDRATE, baud)) {
		ioctl (fd0, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
	if (OK != ioctl (fd1, FIOBAUDRATE, baud)) {
		ioctl (fd1, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}

	/* flush buffers */
	ioctl( fd0, FIOFLUSH, 0 );
	ioctl( fd1, FIOFLUSH, 0 );

	ioctl( fd0, SIO_MCTRL_ISIG_MASK, (int) &retVal );
	fprintf(stdout, "\n %s: ISIG_MASK 0x%04x\n", devName0, retVal);

	ioctl( fd0, SIO_MCTRL_OSIG_MASK, (int) &retVal );
	fprintf(stdout, " %s: OSIG_MASK 0x%04x\n", devName0, retVal);

	ioctl( fd0, SIO_MSTAT_GET, (int) &retVal );
	fprintf(stdout, " %s: MSTAT_GET 0x%04x\n", devName0, retVal);

	if(mode == 0 ){
		fprintf(stdout, " %s: simple test for handshake lines\n", devName0);
		/* disable automatic handshaking */
		ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_CTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_DTR, 0);
		ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
		ioctl (fd0, M77_MODEM_XONXOFF, 0);
		do{
			fprintf(stdout, " %s:  set DTR\n", devName0);
			ioctl( fd0, SIO_MCTRL_BITS_SET, SIO_MODEM_DTR );
			taskDelay( sysClkRateGet()/4 + 1 );
			ioctl( fd0, SIO_MSTAT_GET, (int) &retVal );
			fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", devName0, retVal);

			fprintf(stdout, " %s:  set RTS\n", devName0);
			ioctl( fd0, SIO_MCTRL_BITS_SET, SIO_MODEM_RTS );
			taskDelay( sysClkRateGet()/4 + 1 );
			ioctl( fd0, SIO_MSTAT_GET, (int) &retVal );
			fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", devName0, retVal);

			fprintf(stdout, " %s:  clear DTR\n", devName0);
			ioctl( fd0, SIO_MCTRL_BITS_CLR, SIO_MODEM_DTR );
			taskDelay( sysClkRateGet()/4 + 1 );
			ioctl( fd0, SIO_MSTAT_GET, (int) &retVal );
			fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", devName0, retVal);

			fprintf(stdout, " %s:  clear RTS\n", devName0);
			ioctl( fd0, SIO_MCTRL_BITS_CLR, SIO_MODEM_RTS );
			taskDelay( sysClkRateGet()/4 + 1 );
			ioctl( fd0, SIO_MSTAT_GET, (int) &retVal );
			fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", devName0, retVal);
		} while( end-- > 0 );
		goto CLEANUP;
	}
	if(mode == 1) {   /* no Hs */
		/* disable automatic handshaking */
		ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
		ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_CTS, 0);
		ioctl (fd1, M77_MODEM_AUTO_CTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_DTR, 0);
		ioctl (fd1, M77_MODEM_AUTO_DTR, 0);
		ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
		ioctl (fd1, M77_MODEM_AUTO_DSR, 0);
		ioctl (fd0, M77_MODEM_XONXOFF, 0);
		ioctl (fd1, M77_MODEM_XONXOFF, 0);
		fprintf(stdout, " %s: extended test, no handshakes\n", devName0);
	}
	if(mode == 2) {   /* XON / XOFF */
		fprintf(stdout, " %s: extended test, XON/XOFF\n", devName0);

		/* disable automatic HW handshaking */
		ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
		ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_CTS, 0);
		ioctl (fd1, M77_MODEM_AUTO_CTS, 0);
		ioctl (fd0, M77_MODEM_AUTO_DTR, 0);
		ioctl (fd1, M77_MODEM_AUTO_DTR, 0);
		ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
		ioctl (fd1, M77_MODEM_AUTO_DSR, 0);

		/* set FIFO Interrupt levels */
		ioctl( fd0, M77_TX_FIFO_LEVEL, 7 );
		ioctl( fd0, M77_RX_FIFO_LEVEL, 7 );
		ioctl( fd1, M77_TX_FIFO_LEVEL, 7 );
		ioctl( fd1, M77_RX_FIFO_LEVEL, 7 );

		/* set handshake FIFO levels */
		ioctl( fd0, M77_HS_HIGH_FIFO_LEVEL, 5 );
		ioctl( fd0, M77_HS_LOW_FIFO_LEVEL,  5 );
		ioctl( fd1, M77_HS_HIGH_FIFO_LEVEL, 5 );
		ioctl( fd1, M77_HS_LOW_FIFO_LEVEL,  5 );

		/* enable inband flow control */
		ioctl( fd0, M77_MODEM_XONXOFF,  4 ); /* XON1/XOFF1 for rec+transm */
		ioctl( fd1, M77_MODEM_XONXOFF,  4 ); /* XON1/XOFF1 for rec+transm */
	}
	if(mode == 3) {   /* RTS/CTS */
		fprintf(stdout, " %s: extended test, RTS/CTS \n", devName0);

		/* set FIFO Interrupt levels */
		ioctl( fd0, M77_TX_FIFO_LEVEL, 7 );
		ioctl( fd0, M77_RX_FIFO_LEVEL, 7 );
		ioctl( fd1, M77_TX_FIFO_LEVEL, 7 );
		ioctl( fd1, M77_RX_FIFO_LEVEL, 7 );

		/* set handshake FIFO levels */
		ioctl( fd0, M77_HS_HIGH_FIFO_LEVEL, 5 );
		ioctl( fd0, M77_HS_LOW_FIFO_LEVEL,  5 );
		ioctl( fd1, M77_HS_HIGH_FIFO_LEVEL, 5 );
		ioctl( fd1, M77_HS_LOW_FIFO_LEVEL,  5 );

		/* disable automatic SW handshaking */
		ioctl (fd0, M77_MODEM_XONXOFF, 0);
		ioctl (fd1, M77_MODEM_XONXOFF, 0);

		/* enable automatic HW handshaking */
		if ( !(strncmp(devName0, "/m45n_sw",8)))	/* M45N_Swapped Module */
		{
			char device[13];
			char symb;
			strncpy(device, devName0,12);
			symb = device[11] - '0';
			printf("/M45N_SW, Channel %d \n",symb);
			if ( symb > 1){
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 1);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 1);
			}	
		}
		else if ( !(strncmp(devName0, "/m45n",5)))			/* M45N Module */
		{
			char device[9];
			char symb;
			strncpy(device, devName0,9);
			symb = device[8] - '0';
			printf("/M45N, Channel %d \n",symb);
			if ( symb > 1){
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 1);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 1);
			}	
		}
		else if ( !(strncmp(devName0, "/m69n_sw",8)))			/* M69N_Swapped Module */
		{
			char device[13];
			strncpy(device, devName0,12);
			device[11] = device[11] - '0';
			printf("/M69N_SW, Channel %d \n",device[11]);
			if ( device[11] > 1){
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 1);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 1);
			}
		} 
		else if(!(strncmp(devName0,"/m69n",5)))			/* M69N Module */
		{
			char device[9];
			strncpy(device, devName0,9);
			device[8] = device[8] - '0';
			printf("/M69N, Channel %d \n",device[8]);
			if ( device[8] > 1){
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 1);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 1);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fd0, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd0, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd0, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd0, M77_MODEM_AUTO_DSR, 0);
				ioctl (fd1, M77_MODEM_AUTO_RTS, 0);
				ioctl (fd1, M77_MODEM_AUTO_CTS, 1);
				ioctl (fd1, M77_MODEM_AUTO_DTR, 1);
				ioctl (fd1, M77_MODEM_AUTO_DSR, 0);
			}
		}	
	}
	/*ioctl (fd0, M77_REG_DUMP, 0);*/

	if( mode != 0 )
	{
		int		numOutBytes, i;
		int		numInBytes=0;
		char	*inBuf=NULL, *inChar=NULL;

		if( dir==0 ) { /* transmit */
			fprintf(stdout, " %s: extended test: transmit\n", devName0);
			do {
				numOutBytes = write( fd0, outPattern, sizeof(outPattern) );
				if( numOutBytes != sizeof(outPattern) )
				{
					fprintf(stderr, "\n*** write failed (%s line %d)\n",
							__FILE__, __LINE__ );
					goto CLEANUP;
				}/*if*/
				taskDelay( sysClkRateGet() );
				fprintf(stdout, " %s: extended test: transmit end=%d\n", devName0, end);
			} while( end-- > 0 );
		}

		if( dir==1 ) { /* receive */
			fprintf(stdout, " %s: extended test: receive\n", devName0);
			inBuf=malloc( SERIAL_BUF_SIZE );
			if( !inBuf )
			{
				fprintf(stderr, "\n*** Not enough memory (%s line %d)\n",
						__FILE__, __LINE__ );
				return( -1 );
			}/*if*/
			do {
				for( j=0; j<100; j++ ){
					if( ioctl( fd0, FIONREAD, (int)&numInBytes ) )
					{
						fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n",
								__FILE__, __LINE__);
						goto CLEANUP;
					}/*if*/

					if( numInBytes )
						numInBytes = read( fd0, inBuf, numInBytes );

					inChar = inBuf;
					for(i = 0; i < numInBytes; i++){
						fprintf(stdout, " 0x%02x %c\r\n", *inChar, *inChar++);
					}
					taskDelay( sysClkRateGet()/100 + 1 );
				}
			} while( end-- > 0 );
			free( inBuf );
		}

		if( dir==2 ) do { /* birectional */
			fprintf(stdout, " %s <--> %s: extended test: bidir\n",
					devName0, devName1);
			if (OK != m77SerTrans( fd0, fd1, (int) baud)) {
				fprintf(stderr, "\n*** Error Transmission failed %s -> %s\n",
						devName0, devName1);
				goto CLEANUP;
			}
		} while( end-- > 0 );
		if(dir==3) {
			fprintf(stdout, "%s: Driver Loopback\n", devName0);
			inBuf=malloc( SERIAL_BUF_SIZE );
			if( !inBuf )
			{
				fprintf(stderr, "\n*** Not enough memory (%s line %d)\n",
						__FILE__, __LINE__ );
				return( -1 );
			}/*if*/
			while(1) {
				if( 1 == read(fd0, inBuf, 1) ) {
					write(fd0, inBuf, 1);
				}
			}
		}
	}

CLEANUP:
	/* ioctl (fd0, M77_REG_DUMP, 0); */

	close (fd0);
	close (fd1);
	return 0;
}

int m77 = 1;

int m77OxRegTest(int base)
{
	u_int16 valueLCR;
	u_int16 valueDLL;
	u_int16 valueMCR;
	u_int16 valueIER;
	u_int16 valueSPR;

	u_int16 retDLL;
	u_int16 retIER;
	u_int16 retSPR;

	int baseWrap = base + 0x30;

	do {
/*
		valueLCR = (u_int16) rand();
		valueLCR |= 0x80;
		valueLCR &= 0xff;
*/
		valueMCR = (u_int16) rand();
		valueMCR &= 0xff;
		valueDLL = (u_int16) rand();
		valueDLL &= 0xff;
		valueIER = (u_int16) rand();
		valueIER &= 0xff;
		valueSPR = (u_int16) rand();
		valueSPR &= 0xff;


		valueLCR = 0x80;
		MWRITE_D16(base, M77_LCR_REG, valueLCR);
/*
		MWRITE_D16(base, M77_MCR_REG, valueMCR);
*/
		MWRITE_D16(base, M77_HOLD_DLL_REG, valueDLL);
		MWRITE_D16(base, M77_IER_DLM_REG, valueIER);
		MWRITE_D16(base, M77_SPR_REG, valueSPR);

/*
		retLCR = (0xff & MREAD_D16(base, M77_LCR_REG));
	   	if (valueLCR != retLCR) {
	   		printf("\n m77OxRegTest LCR is 0x%02x should be 0x%02x\n", retLCR, valueLCR);
	   	}
*/
/*
		retMCR = (0xff & MREAD_D16(base, M77_MCR_REG));
	   	if (valueMCR != retMCR) {
	   		printf("\n m77OxRegTest MCR is 0x%02x should be 0x%02x\n", retMCR, valueMCR);
	   	}
*/
		retDLL = (0xff & MREAD_D16(base, M77_HOLD_DLL_REG));
	   	if (valueDLL != retDLL) {
	   		printf("\n m77OxRegTest DLL is 0x%02x should be 0x%02x\n", retDLL, valueDLL);
	   	}

		retIER = (0xff & MREAD_D16(base, M77_IER_DLM_REG));
	   	if (valueIER != retIER) {
	   		printf("\n m77OxRegTest IER is 0x%02x should be 0x%02x\n", retIER, valueIER);
	   	}

		retSPR = (0xff & MREAD_D16(base, M77_SPR_REG));
	   	if (valueSPR != retSPR) {
	   		printf("\n m77OxRegTest SPR is 0x%02x should be 0x%02x\n", retSPR, valueSPR);
	   	}

	   	if (base == baseWrap) {
	   		base = baseWrap - 0x30;
	   	}
	   	else {
	   		base += 0x10;
	   	}

	} while(m77);
	return 0;
}

void showActivity(void)
{
	static int count = 0;

	switch(count) {
		case 0:
			printf("\b|");
			break;

		case 1:
			printf("\b/");
			break;

		case 2:
			printf("\b-");
			break;

		case 3:
			printf("\b\\");
			break;

		case 4:
			printf("\b|");
			break;

		case 5:
			printf("\b/");
			break;

		case 6:
			printf("\b-");
			break;

		case 7:
			printf("\b\\");
			break;

	}

	if (++count > 7) {
		count = 0;
	}
	return;
}



void m77IntLock( int freq ) {
	int oldLevel;

	if( !freq ) freq = 100;
	do {
    oldLevel = intLock();
    OSS_MikroDelay(NULL, 100);
    intUnlock(oldLevel);

    taskDelay( sysClkRateGet()/freq + 1);
	} while(1) ;
}







/********************************* m77Testck *********************************
 *
 *  Description: Sends a string at the output device and checks the received 
 *				 chracter at the input device.
 *
 *----------------------------------------------------------------------------
 *  Input......:  
 *		OutputDevice, 
 *		InputDevice, 
 *		baud rate, 
 * 		M77_RS423|M77_RS422_HD|M77_RS422_FD|M77_RS485_HD|M77_RS485_FD|M77_RS232,
 *		hardware options 
 * 													    
 *  Output.....:  OK | Error
 *  Globals....:  TxCount_G , RxCount_G
 *****************************************************************************/
int m77Testck( char * outDev,char * inDev, int baud, int rsxxx, int hwopt )
{
	int i;
	int fdOut,fdIn;
	int error;
	char sendtxt[]= "Test communication of the UART!"
	                " Hopefully this test will du to our complete satisfaction. ";
	char recvtxt[strlen(sendtxt)+1];
	int receive_count;
	int send_count;
	
	if (outDev == NULL ||  baud == 0) {
		printf("\n\nUsage: m77Testck [outDev], [inDev], [baud],[rsxxx],[hwopt]\n"
		           "      outDev:  Name of M77 output device\n"
               	   "       inDev:  Name of M77 input device\n"
		           "        baud:  Transmission baudrate\n"
		           "       rsxxx:  UART stanard RS423-RS232(0x00-0x07)\n"
		           "       hwopt:  Value of the SIO_HW_OPTS_SET 0xXX\n");
		return ERROR;
	}
	
	printf("\n\n ++++ m77Testck ++++\n");
	printf(" Characters sent from device %s to device %s.\n", outDev,inDev);
	printf(" Baudrate: %d\n", baud);
	

	if (ERROR == (fdOut = open(outDev, O_WRONLY , 0644))) {
		printf(" *** m77Test: Error opening %s\n", outDev);
		return ERROR;
	}
	if (ERROR == (fdIn = open(inDev, O_RDONLY , 0644))) {
		printf(" *** m77Test: Error opening %s\n", inDev);
		return ERROR;
	}
	if( ( strncmp(outDev,"/m69n",5)) && (strncmp(outDev,"/m45n",5)))
	{
		/*---- M77_PHYS_INT_SET 0x00 (default) --------------------------------------+
		|											0x00 RS423
		|											0x01 RS422 half duplex(automatic Rx/Tx direction control)
		|											0x02 RS422 full duplex
		|											0x03 RS485 half duplex(automatic Rx/Tx direction control)
		|											0x04 RS485 full duplex
		|											0x07 RS232
		+---------------------------------------------------------------------------*/
		/* set pysical interface */
		error = ioctl (fdOut, M77_PHYS_INT_SET, rsxxx); /*M77_RS232*/
		error |= ioctl (fdIn, M77_PHYS_INT_SET, rsxxx); /*M77_RS232*/
		if (error) {
			fprintf(stderr, "\n*** Physical interface can not be set\n");	
			goto errorEnd;
		}
	}
	
	/*--SIO_HW_OPTS_SET 0..0xff 0x0e(default) Set up data transmit format:  -----+
	|														Bit [0]: CLOCAL (not supported)
	|														Bit [1]: CREAD (Receiver Enable Bit)
	|														0 = receiver disabled
	|														1 = receiver enabled (default)
	|														Bit [3...2]: CSIZE (Set Word Length)
	|														0 0 = 5 bits
	|														0 1 = 6 bits
	|														1 0 = 7 bits
	|														1 1 = 8 bits (default)
	|														Bit [4]: HUP (not supported)
	|														Bit [5]: STB (Number of Stop Bits)
	|														0 = 1 stop bit (default)
	|														1 = 2 stop bits
	|														Bit [6]: PEN (Parity Enable Bit)
	|														0 = parity disabled (default)
	|														1 = parity enabled
	|														Bit [7]: PODD (Parity ODD Enable Bit)
	|														0 = even parity (default)
	|														1 = odd parity
	+---------------------------------------------------------------------------*/
	/* Set HW Options to 8N1 */
	error = ioctl (fdOut, SIO_HW_OPTS_SET, hwopt); /*0x0e*/
	error |= ioctl (fdIn, SIO_HW_OPTS_SET, hwopt); /*0x0e*/
	if (error) {
		fprintf(stderr, "\n*** SIO_HW_OPTS_SET can not be set\n");	
		goto errorEnd;
	}

	/* disable FIFOS */
	error = ioctl (fdOut, M77_NO_FIFO, 0); 
	error |= ioctl (fdIn, M77_NO_FIFO, 0); 
	if (error) {
		fprintf(stderr, "\n*** FIFOS can not be disabled\n");	
		goto errorEnd;
	}
	
	/* set options to RAW no flow control */
	error = ioctl (fdOut, FIOSETOPTIONS, OPT_RAW);
	error |= ioctl (fdIn, FIOSETOPTIONS, OPT_RAW);
	if (error) {
		fprintf(stderr, "\n*** FIOSETOPTIONS can not be set\n");	
		goto errorEnd;
	}
	
	/*--- FIOBAUDRATE 0..1.152 Mbaud 9600 baud (default)-------------------------+
	|		Set baud rate
	|		The following baudrate values are allowed for this call:
	|		300, 600, 1200, 2400, 4800, 9600, 14400, 19200,
	|		28800, 38400, 57600, 115200, 230400, 576000, 1152000
	|		Note: For RS232/RS423 the max. value is 115200.
	|		Higher values may cause transmission errors.
	+----------------------------------------------------------------------------*/
	/* set baudrate if invalid set detfault */
	if (OK != ioctl (fdOut, FIOBAUDRATE, baud)) {
		ioctl (fdOut, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
  	if (OK != ioctl (fdIn, FIOBAUDRATE, baud))
		ioctl (fdIn, FIOBAUDRATE, STD_BAUD);
		
	/* clear driver buffers */
	error = ioctl( fdOut, FIOFLUSH, 0 );
	error |= ioctl( fdIn, FIOFLUSH, 0 );
	
	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed\n");
		goto errorEnd;
	}/*if*/
  
  	/* 5 and 6 bit data transfere */
	if ( ((hwopt & 0xc) == 0  )|| ((hwopt & 0xc) == 4))
	{
	  for ( i = 0; i < strlen(sendtxt); i++) 
		{
			sendtxt[i]=i%32;
		}
	}
	receive_count = 0;
	send_count = 0;
	
	/* send string */
	for (i=0; i<strlen(sendtxt); i++) 
	{
		int n;
		n = write(fdOut, &sendtxt[i], 1);
		if( n > 0 )
			send_count += n;
	
	  	n = read(fdIn, &recvtxt[receive_count],1);
		if ( receive_count >= strlen(sendtxt)){
			receive_count = 0;
		}
		else{
			if( n > 0)
				receive_count += n;
		}
	}

	
  if ( strncmp(sendtxt,recvtxt,strlen(sendtxt))){
  	printf("\n Send and receive buffer unequal!\n"
  	         " send_count: %d     receive_count: %d\n",send_count,receive_count);
  	printf(" Send buffer: %s\n"
  	       " Receive buffer: %s\n",sendtxt,recvtxt);
	}
	else{
		printf(" Buffer equal!");
	}
	
errorEnd:
	close(fdOut);
	close(fdIn);
	return (ERROR);
}	


/********************************* m77TestSend *******************************
 *
 *  Description: Test to send characters with and without handshakes. This
 * 				 function can set/clr the tristate at the M45N module and the
 *               Echo_Suppression of the M77.
 *
 *----------------------------------------------------------------------------
 *  Input......:   	OutputDevice , 
 *					Baud Rate, 
 *					Hardware Options, 
 *					Mode      	0: manual toggle of HS signals
 *                          	1: no handshakes
 *                           	2: XON/XOFF
 *                           	3: RTS/CTS
 *						     	4: Get State
 *								5: Tristate
 *								6: No Tristate
 *								7: Enless Loop send received character 
 * 									with enabled hardware handshakes
 *								8: Echo Suppression active
 *								9: Echo Suppression inactive
 *							   10: NO FIFO
 *							   11: FIFO active
 *									 					
 *  Output.....:  OK | Error
 *  Globals....:  TxCount_G , RxCount_G
 *****************************************************************************/
int m77TestSend( char * outDev, int baud, int hwopt,int mode)
{
	int fdOut;
	int error;
	int i;
	int retVal;
			
	if (outDev == NULL ||  baud == 0) {
		printf("\n\nUsage: m77TestSend [outDev], [baud],[hwopt]\n"
		          "      outDev:  Name of M77 output device\n"
                  "        baud:  Transmission baudrate\n"
		          "  	  hwopt:  Hardware Options SIO_HW_OPTS_SET in 0xXX\n"
		          "        mode:  0: manual toggle of HS signals \n"
 				  "          	  1: no handshakes\n"
                  "           	  2: XON/XOFF\n"
                  "           	  3: RTS/CTS\n"
 				  "		     	  4: Get State\n"
 				  "				  5: Tristate\n"
 				  "				  6: No Tristate\n"
 				  "				  7: Enless Loop send received character \n"
  				  "					  with enabled hardware handshakes\n"
 				  "				  8: Echo Suppression active\n"
  				  "			      9: Echo Suppression inactive\n"
 			 	  "			     10: NO FIFO\n"
 				  "			     11: FIFO active\n");
		return ERROR;
	}
	
	printf("\n\n ++++ m77TestSend ++++\n");
	printf(" Characters sent to device %s.\n", outDev);
	printf(" Baudrate: %d\n", baud);
	

	if (ERROR == (fdOut = open(outDev, O_WRONLY , 0644))) {
		printf(" *** m77Test: Error opening %s\n", outDev);
		return ERROR;
	}
	
	if( ( strncmp(outDev,"/m69n",5)) && (strncmp(outDev,"/m45n",5)))
	{
		/* set pysical interface */
		error = ioctl (fdOut, M77_PHYS_INT_SET, M77_RS232); 
		if (error) {
			fprintf(stderr, "\n*** Physical interface can not be set\n");	
			goto errorEnd;
		}
	}	
	
	/* Set HW Options to 8N1 */
	error = ioctl (fdOut, SIO_HW_OPTS_SET, hwopt);
	if (error) {
		fprintf(stderr, "\n*** SIO_HW_OPTS_SET can not be set\n");	
		goto errorEnd;
	}


	/* disable FIFOS */
	error = ioctl (fdOut, M77_NO_FIFO, 0); 
	if (error) {
		fprintf(stderr, "\n*** FIFOS can not be disabled\n");	
		goto errorEnd;
	}
	
	/* set options to RAW no flow control */
/*	error = ioctl (fdOut, FIOSETOPTIONS, OPT_RAW);
	if (error) {
		fprintf(stderr, "\n*** FIOSETOPTIONS can not be set\n");	
		goto errorEnd;
	}
	*/
	/*--- FIOBAUDRATE 0..1.152 Mbaud 9600 baud (default)-----------------------+
	|		Set baud rate
	|		The following baudrate values are allowed for this call:
	|		300, 600, 1200, 2400, 4800, 9600, 14400, 19200,
	|		28800, 38400, 57600, 115200, 230400, 576000, 1152000
	|		Note: For RS232/RS423 the max. value is 115200.
	|		Higher values may cause transmission errors.
	+-------------------------------------------------------------------------*/
	/* set baudrate if invalid set detfault */
	if (OK != ioctl (fdOut, FIOBAUDRATE, baud)) {
		ioctl (fdOut, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
  		
	/* clear driver buffers */
	error = ioctl( fdOut, FIOFLUSH, 0 );
		
	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed\n");
		goto errorEnd;
	}/*if*/
	
	
	
	if(mode == 0 ){
		fprintf(stdout, " %s: simple test for handshake lines\n", outDev);
		/* disable automatic handshaking */
		ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_CTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DTR, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
		ioctl (fdOut, M77_MODEM_XONXOFF, 0);
		
		fprintf(stdout, " %s:  set DTR\n", outDev);
		ioctl( fdOut, SIO_MCTRL_BITS_SET, SIO_MODEM_DTR );
		taskDelay( sysClkRateGet()/4 + 1 );
		ioctl( fdOut, SIO_MSTAT_GET, (int) &retVal );
		fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", outDev, retVal);

		fprintf(stdout, " %s:  set RTS\n", outDev);
		ioctl( fdOut, SIO_MCTRL_BITS_SET, SIO_MODEM_RTS );
		taskDelay( sysClkRateGet()/4 + 1 );
		ioctl( fdOut, SIO_MSTAT_GET, (int) &retVal );
		fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", outDev, retVal);

		fprintf(stdout, " %s:  clear DTR\n", outDev);
		ioctl( fdOut, SIO_MCTRL_BITS_CLR, SIO_MODEM_DTR );
		taskDelay( sysClkRateGet()/4 + 1 );
		ioctl( fdOut, SIO_MSTAT_GET, (int) &retVal );
		fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", outDev, retVal);

		fprintf(stdout, " %s:  clear RTS\n", outDev);
		ioctl( fdOut, SIO_MCTRL_BITS_CLR, SIO_MODEM_RTS );
		taskDelay( sysClkRateGet()/4 + 1 );
		ioctl( fdOut, SIO_MSTAT_GET, (int) &retVal );
		fprintf(stdout, " %s:     MSTAT_GET 0x%04x\n", outDev, retVal);
		goto errorEnd;
	}
	if(mode == 1) {   /* no Hs */
		/* disable automatic handshaking */
		ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_CTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DTR, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
		ioctl (fdOut, M77_MODEM_XONXOFF, 0);
		fprintf(stdout, " %s: extended test, no handshakes\n", outDev);
	}
	if(mode == 2) {   /* XON / XOFF */
		fprintf(stdout, " %s: extended test, XON/XOFF\n", outDev);

		/* disable automatic HW handshaking */
		ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_CTS, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DTR, 0);
		ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);

		/* set FIFO Interrupt levels */
		ioctl( fdOut, M77_TX_FIFO_LEVEL, 7 );
		ioctl( fdOut, M77_RX_FIFO_LEVEL, 7 );

		/* set handshake FIFO levels */
		ioctl( fdOut, M77_HS_HIGH_FIFO_LEVEL, 5 );
		ioctl( fdOut, M77_HS_LOW_FIFO_LEVEL,  5 );

		/* enable inband flow control */
		ioctl( fdOut, M77_MODEM_XONXOFF,  4 ); /* XON1/XOFF1 for rec+transm */
	}
	if( ( mode == 3 ) ||( mode == 7 )|| ( mode == 10) || (mode == 11)) {   /* RTS/CTS */
		fprintf(stdout, " %s: extended test, RTS/CTS \n", outDev);
		
		if ( mode == 10)
		{
			mode = 7;
		}
		else if ( mode == 11){
			/* set FIFO Interrupt levels */
			ioctl( fdOut, M77_TX_FIFO_LEVEL, 16 );
			ioctl( fdOut, M77_RX_FIFO_LEVEL, 16 );
				
			/* set handshake FIFO levels */
			ioctl( fdOut, M77_HS_HIGH_FIFO_LEVEL, 10 );
			ioctl( fdOut, M77_HS_LOW_FIFO_LEVEL,  10 );
			
			mode = 7;
		}
		else{
			/* set FIFO Interrupt levels */
			ioctl( fdOut, M77_TX_FIFO_LEVEL, 7 );
			ioctl( fdOut, M77_RX_FIFO_LEVEL, 7 );
			
			/* set handshake FIFO levels */
			ioctl( fdOut, M77_HS_HIGH_FIFO_LEVEL, 5 );
			ioctl( fdOut, M77_HS_LOW_FIFO_LEVEL,  5 );
		}
		
		
		/* disable automatic SW handshaking */
		ioctl (fdOut, M77_MODEM_XONXOFF, 0);
			
		/* enable automatic HW handshaking */
		if ( !(strncmp(outDev, "/m45n_sw",8)))	/* M45N_Swapped Module */
		{
			char device[13];
			char symb;
			strncpy(device, outDev,12);
			symb = device[11] - '0';
			printf("/M45N_SW, Channel %d \n",symb);
			if ( symb > 1){
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 1);
			}	
		}
		else if ( !(strncmp(outDev, "/m45n",5)))			/* M45N Module */
		{
			char device[9];
			char symb;
			strncpy(device, outDev,9);
			symb = device[8] - '0';
			printf("/M45N, Channel %d \n",symb);
			if ( symb > 1){
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 1);
			}	
		}
		else if ( !(strncmp(outDev, "/m69n_sw",8)))		/* M69N_Swapped Module */
		{
			char device[13];
			strncpy(device, outDev,12);
			device[11] = device[11] - '0';
			printf("/M69N_SW, Channel %d \n",device[11]);
			if ( device[11] > 1){
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 1);
			}
		} 
		else if(!(strncmp(outDev,"/m69n",5)))				/* M69N Module */
		{
			char device[9];
			strncpy(device, outDev,9);
			device[8] = device[8] - '0';
			printf("/M69N, Channel %d \n",device[8]);
			if ( device[8] > 1){
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 1);
			}
			else{
				/* enable automatic HW handshaking */
				ioctl (fdOut, M77_MODEM_AUTO_RTS, 0);
				ioctl (fdOut, M77_MODEM_AUTO_CTS, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DTR, 1);
				ioctl (fdOut, M77_MODEM_AUTO_DSR, 0);
			}
		}	
		else printf("/M77 \n");
	}/* end if(mode == 3)*/
	if (mode == 4 )
	{
		int val = 0;
		
		printf("0x%04x  = SIO_HW_OPTS_GET  : 0x%08x\n",ioctl(fdOut,SIO_HW_OPTS_GET,&val),val);
		printf("0x%04x  = SIO_BAUD_GET     : 0x%08x\n",ioctl(fdOut,SIO_BAUD_GET,&val),val);
		printf("0x%04x  = SIO_MSTAT_GET    : 0x%08x\n",ioctl(fdOut,SIO_MSTAT_GET,&val),val);
		printf("0x%04x  = M77_GET_LAST_ERR : 0x%08x\n",ioctl(fdOut,M77_GET_LAST_ERR,&val),val);
	 	goto errorEnd;
	
	}
	if ( mode == 5)		/*Tristate*/
	{
		printf("Tristate mode\n");
		if ( !strncmp(outDev,"/m45n",5)){
			error = ioctl (fdOut,M77_TRISTATE,1);
		}
		else printf("No M45N-Module, %s doesn't support Tristate!\n",outDev);
	}
	if ( mode == 6)		/*No Tristate*/
	{
			printf("No Tristate mode\n");
		if ( !strncmp(outDev,"/m45n",5)){
			error = ioctl (fdOut,M77_TRISTATE,0);
		}
		else printf("No M45N-Module, %s doesn't support Tristate!\n",outDev);
	}
	if ( mode == 7)
	{
		char dummyBuff[M77_MAX_DUMMY];
		while ( 1 ) 
		{
			read(fdOut,  dummyBuff ,1);
			write(fdOut, dummyBuff , 1);		
			
		}
	}
	
	if ( ( mode == 8  ) || ( mode == 9 ) )  /* Echo Supression in/active */
	{
		if ( mode == 9 ){
			error = ioctl(fdOut,M77_ECHO_SUPPRESS,0);	/* inactive */
			printf(" ECHO_SUPPRESS disabled\n");
		}
		else {
			error = ioctl(fdOut,M77_ECHO_SUPPRESS,1);	/* active */
			printf(" ECHO_SUPPRESS enabled\n");
		}
		if(error) {
			fprintf(stderr, "\n*** M77_ECHO_SUPPRESS can not be set\n");	
				goto errorEnd;
		}
		if( ( strncmp(outDev,"/m69n",5)) && (strncmp(outDev,"/m45n",5)))
		{
			int numInBytes = 0;
			
			/* set pysical interface */
		error = ioctl (fdOut, M77_PHYS_INT_SET, M77_RS422_HD); 
			if (error) {
				fprintf(stderr, "\n*** Physical interface can not be set\n");	
				goto errorEnd;
			}
			
			if( ioctl( fdOut, FIONREAD, (int)&numInBytes ) )
			{
				fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n",
						__FILE__, __LINE__);
				goto errorEnd;
			}/*if*/
			printf(" %d bytes received before sending.\n",numInBytes);
		}
	} /* end ECHO_SUPPRESS */

	/* send string */
	i = write(fdOut, outPattern, sizeof(outPattern));
	if( i != sizeof(outPattern))
		fprintf(stdout, " %s: extended test, size of String not eqaual \n", outDev);
	
	if ( ( mode == 9 ) || ( mode == 8) )
	{
		int numInBytes = 0;
		if( ioctl( fdOut, FIONREAD, (int)&numInBytes ) )
		{
			fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n",
					__FILE__, __LINE__);
			goto errorEnd;
		}/*if*/
		printf(" %d bytes received, %d bytes send.\n",numInBytes,i);
	}

errorEnd:
	close(fdOut);
	return error;
}	