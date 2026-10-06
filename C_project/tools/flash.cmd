@echo off
chcp 65001 >nul
cd /d "%~dp0.."
echo ============================================
echo   Flash C_project.elf and run
echo ============================================
JLink.exe -NoGui 1 -ExitOnError 1 -CommanderScript "tools\jlink-flash.jlink"
echo.
echo If you see "Downloading file ... O.K." and "Reset", the board is programmed.
pause
