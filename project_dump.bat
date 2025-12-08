@echo off
setlocal enabledelayedexpansion

REM Output file name
set OUTPUT=project_dump.txt

REM Clear previous output
del "%OUTPUT%" 2>nul

echo Dumping project files into "%OUTPUT%"...

REM Function-like block using a label
call :dumpFolder "include"
call :dumpFolder "src"
call :dumpFolder "tests"

echo Done!
exit /b


:dumpFolder
set FOLDER=%~1
if exist "%FOLDER%" (
    echo Processing folder "%FOLDER%"...
    for /r "%FOLDER%" %%F in (*) do (
        echo.>>"%OUTPUT%"
        echo ================================================ >>"%OUTPUT%"
        echo FILE: %%F >>"%OUTPUT%"
        echo ================================================ >>"%OUTPUT%"
        echo.>>"%OUTPUT%"
        type "%%F" >>"%OUTPUT%"
        echo.>>"%OUTPUT%"
    )
) else (
    echo Folder "%FOLDER%" not found, skipping.
)
exit /b
