@echo off
REM ***********************************************************************
REM
REM          Author: ag
REM           $Date: 2006/08/22 10:24:45 $
REM       $Revision: 1.3 $
REM
REM     Description: script to make different CPU objects
REM
REM ---------------------------------[ History ]----------------------------
REM
REM    $Log: mk.bat,v $
REM    Revision 1.3  2006/08/22 10:24:45  cs
REM    added:
REM       - support for VxW 6.x
REM       - support for VxW Version dependant dir/file structure in .../LIB/... dir
REM
REM    Revision 1.2  2005/05/13 14:09:20  AGromann
REM    updated
REM
REM    Revision 1.1  2002/07/18 12:05:24  agromann
REM    Initial Revision
REM
REM
REM ------------------------------------------------------------------------
REM    (c) Copyright 2002 by MEN mikro elektronik GmbH, Nuernberg, Germany
REM
REM ************************************************************************
@echo "START"   > out

set WIND_HOST_TYPE=x86-win32
set GNUMAKE=%WIND_BASE%\host\%WIND_HOST_TYPE%\bin\make.exe
rem set GCC_EXEC_PREFIX=%WIND_BASE%\host\%WIND_HOST_TYPE%\lib\gcc-lib\

set TOOL=gnu
set TOOL_FAMILY=gnu
set MEN_DIR=
set MEN_VX_DIR=s:/work/VXWORKS_M77
set CONFIG_NAME=MEN_A12_M77
set USE_PCI=-DPCI

rem REM ====== DBG ==========
rem set DBG=-DDBG -DDBG_DUMP
rem rem -g -O0
rem set DBGDIR=test
rem REM
rem
rem set CPU=I80486
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL
rem
rem goto ENDEND



REM #### Build non Debug version ####
set DBG=
set DBGDIR=

rem set CPU=I80386
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=I80486
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=PENTIUM
rem  %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=PPC403
rem  %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

set CPU=PPC603
 %GNUMAKE% -r -f makef CPU=%CPU% TOOL=%TOOL%  >> out
 if errorlevel 1 GOTO FAIL

rem set CPU=PPC604
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=PPC860
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=MC68040
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

set USE_PCI=

rem set CPU=MC68060
rem  %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

REM #### Build Debug version ####
set DBG= -g -O0
set DBGDIR=test
REM

set USE_PCI=-DPCI

rem set CPU=I80386
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=I80486
rem  %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=PENTIUM
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=MC68040
rem  %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

set USE_PCI=

rem set CPU=MC68060
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL


rem set DBG=-DDBG -gdwarf -O0

rem set CPU=PPC403
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

set USE_PCI=-DPCI
set CPU=PPC603
 %GNUMAKE% -r -f makef  CPU=%CPU% TOOL=%TOOL% DBGDIR=test >> out
 if errorlevel 1 GOTO FAIL
set USE_PCI=

rem set CPU=PPC604
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

rem set CPU=PPC860
rem %GNUMAKE% -r -f makef   >> out
rem if errorlevel 1 GOTO FAIL

goto ENDEND

:FAIL
  echo ===========================
  echo "=> ERRORs detected"
  echo ===========================
goto ENDEND

:ENDEND

