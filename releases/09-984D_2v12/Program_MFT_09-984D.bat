@ECHO OFF

::For development PC
::SET COMMANDER="C:\SiliconLabs\SimplicityStudio\v4\developer\adapter_packs\commander\commander"

::For production PC
SET COMMANDER="C:\Middlefield\Simplicity Commander\commander"

::For Eben
::SET COMMANDER="C:\Program Files (x86)\SEGGER\Commander_win32_1v7p7b561\Simplicity Commander\commander"

SET BL_STACK_APP="09-984D.s37"
SET DEVICE=EFR32BG1B232F256GM56
SET ENCRYPTION_KEY=encryption-key
SET SIGNING_KEY=signing-key-tokens.txt

ECHO Unlocking device
ECHO.

CALL %COMMANDER% device lock --debug disable --device %DEVICE%

SET EXIT_CODE=%ERRORLEVEL% 
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error unlocking debug interface, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO Successfully unlocked device
ECHO.

ECHO Programming Knight with combined app, stack, and bootloader...
ECHO.

CALL %COMMANDER% flash %BL_STACK_APP% --device %DEVICE% --masserase

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error programming Knight, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO Done programming Knight.
ECHO.
ECHO Flashing encryption and signing keys to Knight...

CALL %COMMANDER% flash --device %DEVICE% --tokengroup znet --tokenfile %ENCRYPTION_KEY% --tokenfile %SIGNING_KEY%

SET EXIT_CODE=%ERRORLEVEL%
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error flashing keys to Knight, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO Done flashing keys to Knight.
ECHO.
ECHO Locking debug interface

CALL %COMMANDER% device lock --debug enable --device %DEVICE%

SET EXIT_CODE=%ERRORLEVEL% 
IF %EXIT_CODE% NEQ 0 (  
    ECHO Error locking debug interface, return code = %EXIT_CODE%
	PAUSE
    GOTO:eof
)

ECHO.
ECHO !!!SUCCESS!!!
ECHO.

PAUSE