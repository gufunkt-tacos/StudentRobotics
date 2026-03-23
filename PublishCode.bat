@echo off
setlocal

set "SOURCE_DIR=%~dp0robot-code"

set "USB_DRIVE=D:\"


set "DEST_DIR=%USB_DRIVE%"


if not exist "%SOURCE_DIR%" (
    echo ERROR: Robot code folder was not found.
    pause
    exit /b
)


xcopy "%SOURCE_DIR%" "%DEST_DIR%" /E

echo Folder copied to USB successfully.
powershell -command "$driveEject = New-Object -ComObject Shell.Application; $driveEject.Namespace(17).ParseName('D:').InvokeVerb('Eject')"
echo USB ejected.
pause