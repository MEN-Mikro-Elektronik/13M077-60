/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m77Endpruf.c
 *      Project: M77 VxWorks Driver
 *
 *       Author: ag
 *        $Date: 2003/10/31 11:58:29 $
 *    $Revision: 1.2 $
 *
 *  Description: Final inspection test for M77
 *
 *
 *     Required: MDIS, BBIS
 *     Switches: 
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m77Endpruf.c,v $
 * Revision 1.2  2003/10/31 11:58:29  Agromann
 * updated
 *
 * Revision 1.1  2002/07/18 12:05:23  agromann
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
#include <tickLib.h>
#include <ioLib.h>
#include <sysLib.h>
#include <MEN/men_typs.h>
#include <MEN/oss.h>
#include <MEN/dbg.h>
#include <MEN/desc.h>
#include <MEN/usr_oss.h>
#include <MEN/usr_utl.h>


#undef M77_VERBOSE

#define	SERIAL_BUF_SIZE		1024
#define STD_BAUD			9600
#define M77_KLIMA_BAUD      9600

/*-----------------------------------------+
|  STATICS                                 |
+------------------------------------------*/
static char outPattern[]="THE QUICK BROWN FOX JUMPS OVER THE LAZY DOG -\
 the quick brown fox jumps over the lazy dog\n";


/*-----------------------------------------+
|  PROTOTYPES                              |
+------------------------------------------*/
static void showActivity(void);
static int m77Ser( int outFd, int inFd, int mode, int baud, int numRuns);



/********************************* m77Ser *************************************
 *
 *  Description: Check serial lines input and output 
 *
 *----------------------------------------------------------------------------
 *  Input......:  outFd   file descriptor of transmitting device
 *                inFd    file descriptor of receiving device
 *                baud    baudrate for transmission
 *                numRuns number of runs
 *
 *  Output.....:  
 *
 *  Globals....:  
 *****************************************************************************/
static int m77Ser( int outFd, int inFd, int mode , int baud, int numRuns)
{
char	*inBuf=NULL;
int		error;
int		retVal=-1;
int		numOutBytes;
int		numInBytes=0;
ULONG   timeoutTick;
int     numReceived;
char    receiveBuff[2];
int     runCount = 1;


	if( !outFd )
	{
		fprintf(stderr, "\n*** out file descriptor missing (%s line %d)\n"
		, __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	if( !inFd )
	{
		fprintf(stderr, "\n*** in file descriptor missing (%s line %d)\n"
		, __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	inBuf=malloc( SERIAL_BUF_SIZE );
	if( !inBuf )
	{
		fprintf(stderr, "\n*** Not enough memory (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/

	/* set pysical interface */
	error = ioctl (outFd, 0x106 /* M77_PHYS_INT_SET */, mode); 
	error |= ioctl (inFd, 0x106 /* M77_PHYS_INT_SET */, mode); 
	if (error) {
		fprintf(stderr, "\n*** Physical interface can not be set (%s line %d)\n"
		, __FILE__, __LINE__ );	
	}
	
	/* set options to RAW to avoid echo */
	(void) ioctl (outFd, FIOSETOPTIONS, OPT_RAW); 
	(void) ioctl (inFd, FIOSETOPTIONS, OPT_RAW); 
	
	/* set baudrate if invalid set detfault */
	if (OK != ioctl (outFd, FIOBAUDRATE, baud)) {
		ioctl (outFd, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
	if (OK != ioctl (inFd, FIOBAUDRATE, baud))
		ioctl (inFd, FIOBAUDRATE, STD_BAUD);


/* /* delay zur Verhinderung von Phantomzeichen einfügen ?? */

	error = ioctl( outFd, FIOFLUSH, 0 );	
	error |= ioctl( inFd, FIOFLUSH, 0 );
	
	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed (%s line %d)\n", __FILE__, __LINE__ );
		goto CLEANUP;
	}/*if*/


	/* transmit test pattern */
	while (1) {
		numInBytes = 0;
		showActivity();
	
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
			taskDelay(60);
			error |= ioctl( inFd, FIONREAD, (int)&numInBytes );
			if( error )
			{
				fprintf(stderr, "\n*** FIONREAD failed (%s line %d)\n", __FILE__, __LINE__ );
				goto CLEANUP;
			}/*if*/
			if (timeoutTick < tickGet()) {
				break;
			}
			showActivity();
	
		} /* while */
		
		if (numInBytes != numOutBytes) {
			fprintf(stderr, "\n*** number of received bytes=%d not equal %d number of sent bytes\n",
					numInBytes, numOutBytes );
			goto CLEANUP;
		}

		showActivity();
	
		numInBytes = read( inFd, inBuf, numInBytes );
		if( numInBytes != numOutBytes )
		{
			fprintf(stderr, "\n*** number of read bytes=%d not equal %d number of sent bytes\n",
					numInBytes, numOutBytes );
			goto CLEANUP;
		}/*if*/

		showActivity();
	
		/* check pattern */
		error = strncmp( inBuf, outPattern, numOutBytes );
		if( error )
		{
			fprintf(stderr, "\n*** pattern mismatch\n" );
			error = -1;
			goto CLEANUP;
		}/*if*/
	
		/* check output device for received characters */
		error = ioctl( outFd, FIONREAD, (int)&numReceived );
		if (error) {
			fprintf(stderr, "\n*** Error ioctl failed ch0Fd -> ch1Fd\n");
			goto CLEANUP;
		}
		if (numReceived) {
			fprintf(stderr, "\n*** Error Echo-Suppression failed:\n"
			"Received %d bytes:\n", numReceived);
			while (numReceived--) {
				read( outFd, receiveBuff, 1);	
				printf("0x%02x\n", receiveBuff[0]);
			}
			goto CLEANUP;
						
		}
		showActivity();

		if (numRuns != 0)
			if (numRuns <= runCount++)
				goto END_OK;


	} /* while(1) */

END_OK:
	/* say OK */
	retVal = 0;

CLEANUP:
	free( inBuf);
	if( !retVal ) {
#ifdef M77_VERBOSE
		printf("%d ==> OK\n", numInBytes);
#endif /* M77_VERBOSE */
	}
	else
		fprintf(stderr, " ==> FAILED\n");
	return( retVal );	
}







/********************************* main ***************************************
 *
 *  Description: Final inspection test
 *
 *----------------------------------------------------------------------------
 *  Input......:  argc    argument count value
 *                argv    pointer to argument value string
 *
 *  Output.....:  -
 *
 *  Globals....:  OK or ERROR
 *****************************************************************************/
int main( int argc, char *argv[] )
{

	int outFd;
	int inFd;
	char * devOut;
	char * devIn;
	int mode;
	int baud;
	int numRuns;
	int errorSuccess = ERROR;
	char dummy[2];
	
	if (argc < 6 || argc > 6) {
		fprintf(stderr, "\n Syntax  : m77Endpruf <devOut> <devIn> <mode> <baud> <numRuns>\n\n"
		                   " <devOut>   name of device wich sends test pattern\n"
		                   " <devIn>    name of device wich receives test pattern\n"
		                   " <mode>     line driver mode: 0 = RS423\n"
		                   "                              1 = RS422 HD\n"
		                   "                              2 = RS422 FD\n"
		                   "                              3 = RS485 HD\n"
		                   "                              4 = RS485 FD\n"
		                   "                              7 = RS232\n"
		                   " <baud>     baudrate used for transfer\n"
		                   " <numRuns>  number of runns (0 = endless)\n");
		return -1;
	}
	
	devOut  = argv[1];
	devIn   = argv[2];
	mode    = atoi(argv[3]);
	baud    = atoi(argv[4]);
	numRuns = atoi(argv[5]);
	

	printf("\n\n\n ++++++++ M77 Final Inspection Test ++++++++\n"
	              " Testing %s -> %s \n"
	              " Mode    = %d\n"
	              " baud    = %d\n"
	              " numRuns = %d\n"
	              " Test Running: ", devOut, devIn, mode, baud, numRuns);
	
	outFd = open (devOut, O_RDWR, 0644);

	inFd  = open (devIn, O_RDWR, 0644);

	if( ERROR == outFd || ERROR == inFd)
	    
	{
		fprintf(stderr, "\n*** Error opening output device (%s line %d)\n", __FILE__, __LINE__ );
		return( -1 );
	}/*if*/


	if (0 != m77Ser( outFd, inFd, mode, baud, numRuns)) {
		goto Error;						
	}
		
	/* say OK */
	errorSuccess = OK;

Error:
	printf("\n");
	close (outFd);
	close (inFd);

	if (errorSuccess == OK) {
		printf("\n -> TEST OK\n");
	}
	else {
		printf("\n -> TEST FAILED\n");
		printf("\nPress ENTER to continue\n");
		fgets(dummy, 2, stdin);
	}
	
	return errorSuccess;	
}



static void showActivity(void) 
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
