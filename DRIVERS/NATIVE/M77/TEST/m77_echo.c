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
 * 2015/07/31 channoyer
 * R: 1. Not possible to use FIFO
 *    2. No error handling in the endless loop
 *    3. Incorrect 10 secondes delay in m77EchoShow when it is called for not
 *       endless loop (only once)
 *    4. Cosmetic
 * M: 1. Add to the function m77Echo() the arguments rxFifoLevel and
 *       txFifoLevel
 *    2. a) Check error case
 *       b) Do not call the write in the case the read return a length of 0
 *    3. Call taskDelay only in case we do endless loop
 *    4. Remove unecesary blank
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
#define BUF_SIZE			256

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
int m77Echo(char * inDev, char * outDev, int baud, int rxFifoLevel, int txFifoLevel)

{
	int fdIn;
	int fdOut;
	int error;
	char dummyBuff[BUF_SIZE];

	if (inDev == NULL || outDev == NULL || baud == 0 || (txFifoLevel < 0)
			|| (txFifoLevel > M77_FIFO_SIZE - 1) || (rxFifoLevel < 0)
			|| (rxFifoLevel > M77_FIFO_SIZE - 1)) {
		printf("\n\nUsage: m77Echo inDev, outDev, baud[, rxFifoLevel[, txFifoLevel]])\n"
						"       inDev:  Name of M77 input device\n"
						"      outDev:  Name of M77 output device\n"
						"        baud:  Transmission baudrate\n"
						" rxFifoLevel:  RX fifo level (0 .. %d; 0 means fifo off)\n"
						" txFifoLevel:  TX fifo level (0 .. %d; 0 means fifo off)\n",
						M77_FIFO_SIZE - 1, M77_FIFO_SIZE - 1);
		return ERROR;
	}


	printf("\n\n ++++ m77Echo ++++\n");
	printf(" Characters sent to device %s will be echoed on device %s\n", inDev, outDev);
	printf(" Interface: RS232, 8-N-1 no flow control\n");
	printf(" Baudrate: %d\n", baud);

	if (ERROR == (fdIn = open(inDev, O_RDONLY , 0644))) {
		fprintf(stderr, "\n*** m77Echo: Error opening %s\n", inDev);
		return ERROR;
	}

	if (ERROR == (fdOut = open(outDev, O_WRONLY , 0644))) {
		fprintf(stderr, "\n*** m77Echo: Error opening %s\n", outDev);
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

	/* setup TX fifo */
	if (txFifoLevel) {
		error = ioctl(fdOut, M77_TX_FIFO_LEVEL, txFifoLevel);

		if (error) {
			fprintf(stderr, "\n*** Failed to set TX fifo level\n");
			goto errorEnd;
		}
	} else {
		/* 0 means fifos off */
		error = ioctl(fdOut, M77_NO_FIFO, 0);

		if (error) {
			fprintf(stderr, "\n*** Failed to disable TX fifo\n");
			goto errorEnd;
		}
	}

	/* setup RX fifo */
	if (rxFifoLevel) {
		error = ioctl(fdIn, M77_RX_FIFO_LEVEL, rxFifoLevel);

		if (error) {
			fprintf(stderr, "\n*** Failed to set RX fifo level\n");
			goto errorEnd;
		}
	} else {
		/* 0 means fifos off */
		error = ioctl(fdIn, M77_NO_FIFO, 0);

		if (error) {
			 fprintf(stderr, "\n*** Failed to disable RX fifo\n");
			 goto errorEnd;
		 }
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
		n = read(fdIn, dummyBuff, BUF_SIZE);
		if( ERROR == n )
		{
			fprintf(stderr, "\n*** read failed\n");
			goto errorEnd;
		}/*if*/
		if (n > 0) {
			RxCount_G += n;
			n = write(fdOut, dummyBuff, n);
			if( ERROR == n )
			{
				fprintf(stderr, "\n*** write failed\n");
				goto errorEnd;
			}/*if*/
			TxCount_G += n;
		}
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
void m77EchoShow(int periodic)
{
	while (TRUE) {
		printf("Rx/Tx %d / %d\n", RxCount_G, TxCount_G);

		if (!periodic) {
			break;
		} else {
			taskDelay(sysClkRateGet() * 10);
		}
	}
}

