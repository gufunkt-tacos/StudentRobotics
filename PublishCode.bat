@echo off
setlocal

set "SOURCE_DIR=%~dp0robot-code"
set "USB_DRIVE=D:\"
set "DEST_DIR=%USB_DRIVE%robot-code"

if not exist "%SOURCE_DIR%" (
    echo ERROR: Robot code folder was not found.
    pause
    exit /b
)

if not exist "%USB_DRIVE%" (
    echo ERROR: USB drive not found at %USB_DRIVE% - check the drive letter.
    pause
    exit /b
)

echo Copying files...
xcopy "%SOURCE_DIR%" "%DEST_DIR%" /E /I /Y

if errorlevel 1 (
    echo ERROR: Copy failed.
    pause
    exit /b
)

echo Folder copied to USB successfully.

echo Ejecting USB...
powershell -command "(New-Object -comObject Shell.Application).Namespace(17).ParseName('D:\').InvokeVerb('Eject')"

echo Done.
pause