@echo off
rem Uses the Windows PowerShell already supplied with Windows 10/11.
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0run-renode.ps1" %*
exit /b %ERRORLEVEL%
