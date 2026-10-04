@echo off
rem Compile verification for final/ + older/ + wrapper/ against FTC SDK 12.0.0 + Pedro Pathing 3.x.
rem Uses the local JDK and the Maven/AAR-extracted jars staged in ftc_compile.
rem
rem The Pedro 3 jars here were BUILT FROM SOURCE from the official v3.0.1 tag
rem (github.com/Pedro-Pathing/PedroPathing, modules :core and :revhub), because
rem com.pedropathing:revhub / :tuning were not yet published to
rem https://repo.dairy.foundation/releases/ at the time of migration.
rem In a real Android Studio project just use:
rem     maven { url 'https://repo.dairy.foundation/releases/' }
rem     implementation 'com.pedropathing:revhub:x.y.z'
rem     implementation 'com.pedropathing:tuning:1.0.0'
rem
rem NOTE: the root-level BallChaseOpMode.java is intentionally NOT compiled - it is
rem a near-identical duplicate of older/BallChaseOpMode.java (same FQCN) and would
rem be a duplicate-class error.
rem
rem Usage: compile_check.cmd              (compile only)
rem         compile_check.cmd -runmath    (also run the BallMath Java self-test)
rem         compile_check.cmd -runwrapper (also run the wrapper logic self-test)

setlocal enabledelayedexpansion
set ROOT=%~dp0
set COMPILE_DIR=%LOCALAPPDATA%\ftc_compile
set JDK=%COMPILE_DIR%\jdk\jdk-17.0.20.1+1\bin
set LIB=%COMPILE_DIR%\mvn\lib
if not exist "%JDK%\javac.exe" (echo JDK not found under %COMPILE_DIR% & exit /b 1)
if not exist "%LIB%\ftc_robotcore_12.jar" (echo FTC SDK 12 jars not found under %LIB% & exit /b 1)
if not exist "%LIB%\pedro3_core.jar" (echo Pedro 3 core jar not found under %LIB% & exit /b 1)
if not exist "%LIB%\pedro3_revhub.jar" (echo Pedro 3 revhub jar not found under %LIB% & exit /b 1)

set CP=%LIB%\ftc_robotcore_12.jar;%LIB%\ftc_hardware_12.jar;%LIB%\pedro3_core.jar;%LIB%\pedro3_revhub.jar
set OUT=%COMPILE_DIR%\out_pedro3
if not exist "%OUT%" mkdir "%OUT%"

set SRC=
for /r "%ROOT%final" %%f in (*.java) do set SRC=!SRC! "%%f"
for /r "%ROOT%older" %%f in (*.java) do set SRC=!SRC! "%%f"
for /r "%ROOT%wrapper" %%f in (*.java) do set SRC=!SRC! "%%f"

"%JDK%\javac.exe" -nowarn -encoding UTF-8 -cp "%CP%" -d "%OUT%" %SRC%
if errorlevel 1 (echo COMPILE FAILED & exit /b 1)
echo COMPILE OK - FTC SDK 12.0.0 + Pedro Pathing 3.x - classes in %OUT%

if /i "%~1"=="-runmath" (
    "%JDK%\java.exe" -cp "%OUT%;%CP%" org.firstinspires.ftc.teamcode.BallMath
)
if /i "%~1"=="-runwrapper" (
    "%JDK%\java.exe" -cp "%OUT%;%CP%" org.firstinspires.ftc.teamcode.wrapper.WrapperLogicTest
)
endlocal