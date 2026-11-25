@echo off
REM Wrapper: Windows blocks .ps1 execution by default, so run with bypass.
REM Usage:
REM   fetch_refs.cmd            download into this folder
REM   fetch_refs.cmd -DryRun    print the plan only
setlocal
set SCRIPT=%~dp0fetch_refs.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%SCRIPT%" %*
if errorlevel 1 (
  echo.
  echo [fetch_refs] finished with errors. See download_log.txt
)
endlocal
