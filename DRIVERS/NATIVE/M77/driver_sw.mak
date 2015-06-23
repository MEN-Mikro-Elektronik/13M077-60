#***************************  M a k e f i l e  *******************************
#
#         Author: ag
#          $Date: 2002/07/18 12:05:15 $
#      $Revision: 1.1 $
#
#    Description: Makefile definitions for the M77 driver with 
#                 SWAPED ACCESS !
#
#---------------------------------[ History ]---------------------------------
#
#   $Log: driver_sw.mak,v $
#   Revision 1.1  2002/07/18 12:05:15  agromann
#   Initial Revision
#
#
#-----------------------------------------------------------------------------
#   (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
#*****************************************************************************

MAK_NAME=m77_sw

MAK_SWITCH=$(SW_PREFIX)MAC_MEM_MAPPED \
           $(SW_PREFIX)MAC_BYTESWAP \
           $(SW_PREFIX)ID_SW \
           $(SW_PREFIX)PLD_SW \
           $(SW_PREFIX)m77Drv=m77Drv_sw \
           $(SW_PREFIX)M77_SW \


MAK_LIBS=$(LIB_PREFIX)$(MEN_LIB_DIR)/desc$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/oss$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/id_sw$(LIB_SUFFIX)	\
         $(LIB_PREFIX)$(MEN_LIB_DIR)/dbg$(LIB_SUFFIX)	\


MAK_INCL=$(MEN_MOD_DIR)/m77_drv.h \



MAK_INP1=m77_drv$(INP_SUFFIX)

MAK_INP=$(MAK_INP1)
