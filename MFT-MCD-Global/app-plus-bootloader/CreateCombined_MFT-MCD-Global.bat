@ECHO off

SET OBJCOPY="C:\SiliconLabs\SimplicityStudio\v4\developer\toolchains\gnu_arm\7.2_2017q4\bin\arm-none-eabi-objcopy.exe"
SET COMMANDER="C:\SiliconLabs\SimplicityStudio\v4\developer\adapter_packs\commander\commander"
SET BOOTLOADER="..\..\MFT-bootloader\IAR ARM - Default\MFT-bootloader-combined.s37"
SET APP_OUT="..\IAR ARM - Default\MFT-MCD-Global.out"
SET APP_SREC="app.srec"
SET STACK_SREC="stack.srec"
SET SIGNED_STACK="MFT-MCD-Global-stack-signed.s37"
SET SIGNED_APP="MFT-MCD-Global-app-signed.s37"
SET BL_STACK_APP="MFT-MCD-Global.s37"
SET SIGN_KEY="..\..\keys\app-sign-key.pem"

ECHO Separating stack from app...

CALL %OBJCOPY% -O srec -j .text_stack* %APP_OUT% %STACK_SREC%
CALL %OBJCOPY% -O srec -j .text_app* %APP_OUT% %APP_SREC%

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error separating stack from app, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO Done separating stack from app.
ECHO.
ECHO Signing stack...

CALL %COMMANDER% convert %STACK_SREC% --secureboot --keyfile %SIGN_KEY% -o %SIGNED_STACK%

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error signing stack, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO Done signing stack.
ECHO.
ECHO Signing app...

CALL %COMMANDER% convert %APP_SREC% --secureboot --keyfile %SIGN_KEY% -o %SIGNED_APP%

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error signing app, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)
ECHO Done signing app.
ECHO.


ECHO Creating combined .s37 from signed app, signed stack, and combined bootloader files...
ECHO.

CALL %COMMANDER% convert %BOOTLOADER% %SIGNED_STACK% %SIGNED_APP% --outfile %BL_STACK_APP%

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error combining files, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)
ECHO Done combining files.
ECHO.
ECHO !!!SUCCESS!!!
ECHO.

DEL %APP_SREC%
DEL %STACK_SREC%
DEL %SIGNED_STACK%
DEL %SIGNED_APP%

ECHO 

PAUSE