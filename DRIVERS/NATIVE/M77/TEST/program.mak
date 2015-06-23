#***************************  M a k e f i l e  *******************************
#
#         Author: ag
#          $Date: 2002/07/18 12:05:20 $
#      $Revision: 1.1 $
#        $Header: /dd2/CVSR/VXWORKS/DRIVERS/NATIVE/M77/TEST/program.mak,v 1.1 2002/07/18 12:05:20 agromann Exp $
#
#    Description: makefile descriptor file for common
#                 modules MDIS 4.x   e.g. low level driver
#
#---------------------------------[ History ]---------------------------------
#   $Log: program.mak,v $
#   Revision 1.1  2002/07/18 12:05:20  agromann
#   Initial Revision
#
#   
#-----------------------------------------------------------------------------
#   (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=m77Echo

MAK_INCL=$(MEN_MOD_DIR)../m77_drv.h     \

MAK_OPTIM=$(OPT_1)

MAK_INP1=m77_echo$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
