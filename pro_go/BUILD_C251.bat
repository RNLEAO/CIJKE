@echo off
chcp 65001 >nul
set "ROOT=%~dp0"
set "UV4=D:\C251\UV4\UV4.exe"
set "PROJECT=%ROOT%Project\mdk\CIJIANKE_EM.uvproj"
set "LOG_DIR=%ROOT%Project\mdk\logs"
set "LOG=%LOG_DIR%\latest_build.log"

if not exist "%LOG_DIR%" mkdir "%LOG_DIR%"
if exist "%LOG%" del /q "%LOG%"

if not exist "%UV4%" (
  echo Cannot find Keil uVision at %UV4%
  exit /b 2
)

start /wait "" "%UV4%" -r "%PROJECT%" -o "%LOG%"
set "BUILD_RC=%errorlevel%"
type "%LOG%"
exit /b %BUILD_RC%
