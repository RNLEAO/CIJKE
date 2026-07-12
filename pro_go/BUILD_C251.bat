@echo off
chcp 65001 >nul
set "ROOT=%~dp0"
set "UV4=D:\C251\UV4\UV4.exe"
set "PROJECT=%ROOT%Project\mdk\seekfree.uvproj"
set "LOG=%ROOT%Project\mdk\desktop_build.log"

if not exist "%UV4%" (
  echo Cannot find Keil uVision at %UV4%
  exit /b 2
)

start /wait "" "%UV4%" -b "%PROJECT%" -o "%LOG%"
set "BUILD_RC=%errorlevel%"
type "%LOG%"
exit /b %BUILD_RC%
