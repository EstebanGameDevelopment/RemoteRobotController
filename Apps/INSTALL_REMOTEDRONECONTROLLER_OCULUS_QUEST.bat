REM ************************************************************
REM INSTALL ADB TOOLS
REM https://androidmtk.com/download-minimal-adb-and-fastboot-tool
REM ************************************************************
adb devices
adb shell am force-stop com.yourvrexperience.remotedronecontroller
adb uninstall com.yourvrexperience.remotedronecontroller
adb install RemoteDroneController_OculusQuest.apk
pause