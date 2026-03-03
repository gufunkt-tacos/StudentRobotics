@echo off
setlocal

set "SOURCE_DIR=%~dp0"

set "SOURCE_FILE=%SOURCE_DIR%Ledge Pickup.py"


set "USB_DRIVE=D:\"


set "DEST_FILE=%USB_DRIVE%robot.py"


if not exist "%SOURCE_FILE%" (
    echo ERROR: "Ledge Pickup.py" not found in this folder.
    pause
    exit /b
)


copy /Y "%SOURCE_FILE%" "%DEST_FILE%"

echo File copied and renamed to "robot" successfully.
powershell -command "$driveEject = New-Object -ComObject Shell.Application; $driveEject.Namespace(17).ParseName('D:').InvokeVerb('Eject')"
pause