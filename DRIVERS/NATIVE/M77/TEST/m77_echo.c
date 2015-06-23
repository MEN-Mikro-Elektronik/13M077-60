/*********************  P r o g r a m  -  M o d u l e ***********************
 *
 *         Name: m77_echo.c
 *      Project: M77 VxWorks Driver
 *
 *       Author: ag
 *        $Date: 2004/10/15 14:58:19 $
 *    $Revision: 1.2 $
 *
 *  Description: M77 simple dirver test module
 *
 *
 *     Required:
 *     Switches: 
 *
 *-------------------------------[ History ]---------------------------------
 *
 * $Log: m77_echo.c,v $
 * Revision 1.2  2004/10/15 14:58:19  ufranke
 * added
 *   + m77EchoShow for dispay statistics
 *
 * Revision 1.1  2002/07/18 12:05:19  agromann
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
#include "../m77_drv.h"

#define STD_BAUD			9600


/*-----------------------------------------+
|  STATICS                                 |
+------------------------------------------*/
static int RxCount_G = 0;
static int TxCount_G = 0;

/*-----------------------------------------+
|  PROTOTYPES                              |
+------------------------------------------*/

/********************************* m77Echo ************************************
 *
 *  Description: Echo input characters receiced from input device to output 
 *               device.
 *
 *     This function uses the fixed interface type RS232 !
 *
 *----------------------------------------------------------------------------
 *  Input......:  inDev      name of input device to read from
 *                outDev     name of output device to write received data to
 *                baud       baudrate
 *
 *  Output.....:  
 *
 *  Globals....:  RxCount_G , TxCount_G 
 *****************************************************************************/
int m77Echo(char * inDev, char * outDev, int baud)

{
	int fdIn;
	int fdOut;
	int error;
	char dummyBuff[2];

	if (inDev == NULL || outDev == NULL || baud == 0) {
		printf("\n\nUsage: m77Echo [inDev], [outDev], [baud]\n"
		           "       inDev:  Name of M77 input device\n"
		           "      outDev:  Name of M77 output device\n"
		           "        baud:  Transmission baudrate\n");
		return ERROR;
	}


	printf("\n\n ++++ m77Echo ++++\n");
	printf(" Characters sent to device %s will be echoed on device %s\n", inDev, outDev);
	printf(" Interface: RS232, 8-N-1 no flow control\n");
	printf(" Baudrate: %d\n", baud);
	
	if (ERROR == (fdIn = open(inDev, O_RDONLY , 0644))) {
		printf(" *** m77Echo: Error opening %s\n", inDev);
		return ERROR;
	}

	if (ERROR == (fdOut = open(outDev, O_WRONLY , 0644))) {
		printf(" *** m77Echo: Error opening %s\n", outDev);
		return ERROR;
	}

	/* set pysical interface */
	error = ioctl (fdIn, M77_PHYS_INT_SET, M77_RS232); 
	error |= ioctl (fdOut, M77_PHYS_INT_SET, M77_RS232); 
	if (error) {
		fprintf(stderr, "\n*** Physical interface can not be set\n");	
		goto errorEnd;
	}

	/* Set HW Options to 8N1 */
	error = ioctl (fdIn, SIO_HW_OPTS_SET, 0x0e); 
	error |= ioctl (fdOut, SIO_HW_OPTS_SET, 0x0e); 
	if (error) {
		fprintf(stderr, "\n*** SIO_HW_OPTS_SET can not be set\n");	
		goto errorEnd;
	}


	/* disable FIFOS */
	error = ioctl (fdIn, M77_NO_FIFO, 0); 
	error |= ioctl (fdOut, M77_NO_FIFO, 0); 
	if (error) {
		fprintf(stderr, "\n*** FIFOS can not be disabled\n");	
		goto errorEnd;
	}
	
	/* set options to RAW no flow control */
	error = ioctl (fdIn, FIOSETOPTIONS, OPT_RAW);
	error |= ioctl (fdOut, FIOSETOPTIONS, OPT_RAW);
	if (error) {
		fprintf(stderr, "\n*** FIOSETOPTIONS can not be set\n");	
		goto errorEnd;
	}
	
	/* set baudrate if invalid set detfault */
	if (OK != ioctl (fdIn, FIOBAUDRATE, baud)) {
		ioctl (fdIn, FIOBAUDRATE, STD_BAUD);
		fprintf(stderr, "\n*** FIOBAUDRATE failed set baud rate to %d baud", STD_BAUD);
	}
	if (OK != ioctl (fdOut, FIOBAUDRATE, baud))
		ioctl (fdOut, FIOBAUDRATE, STD_BAUD);

	/* clear driver buffers */
	error = ioctl( fdIn, FIOFLUSH, 0 );	
	error |= ioctl( fdOut, FIOFLUSH, 0 );
	
	if( error )
	{
		fprintf(stderr, "\n*** FIOFLUSH failed\n");
		goto errorEnd;
	}/*if*/

	/* endless loopback */
	while(1) 
	{
		int n;
		n = read(fdIn, dummyBuff, 1);
		if( n > 0 )
			RxCount_G += n;
		n = write(fdOut, dummyBuff, 1);
		if( n > 0 )
			TxCount_G += n;
	}

errorEnd:
	close(fdIn);
	close(fdOut);
	return (ERROR);
}


/********************************* m77EchoShow ********************************
 *
 *  Description: Displays statistics.
 *
 *----------------------------------------------------------------------------
 *  Input......:  -
 *  Output.....:  -
 *  Globals....:  RxCount_G , TxCount_G 
 *****************************************************************************/
void m77EchoShow( int onceOnly )
{
	do
	{
		printf("Rx/Tx %d / %d\n", RxCount_G, TxCount_G );
		taskDelay( sysClkRateGet()*10 );
	}while( !onceOnly );
	
}

