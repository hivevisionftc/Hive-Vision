@echo off
rem Assemble the beta-tester drop from this repo into a target FTC project.
rem
rem   install_beta.cmd  <path-to-your-FTC-project>
rem
rem Produces <project>\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\...
rem containing ONLY the 16 files a tester needs, in their correct package
rem directories, plus PEDRO3_BETA_README.md at the project root.
rem
rem The rest of this repo (older demos, superseded OpModes, the duplicate
rem root-level BallChaseOpMode.java) is deliberately left behind: copying the
rem whole tree gives a duplicate-class error, because the root
rem BallChaseOpMode.java and older/BallChaseOpMode.java share an FQCN.

setlocal
set ROOT=%~dp0
if "%~1"=="" (echo usage: install_beta.cmd ^<path-to-FTC-project^> & exit /b 1)
set PROJ=%~1
set DEST=%PROJ%\TeamCode\src\main\java\org\firstinspires\ftc\teamcode

rem arg1 = source file, arg2 = destination directory (filename is preserved)
call :cp "%ROOT%final\BallTracker.java"          "%DEST%"
call :cp "%ROOT%final\BallChaseController.java"  "%DEST%"
call :cp "%ROOT%final\BallHunt.java"             "%DEST%"
call :cp "%ROOT%final\BallChaseFollower.java"    "%DEST%"
call :cp "%ROOT%older\BallHunter.java"           "%DEST%"
call :cp "%ROOT%older\BallMath.java"             "%DEST%"
call :cp "%ROOT%older\pedroPathing\Constants.java" "%DEST%\pedroPathing"

call :cp "%ROOT%wrapper\BallColor.java"          "%DEST%\wrapper"
call :cp "%ROOT%wrapper\Target.java"             "%DEST%\wrapper"
call :cp "%ROOT%wrapper\RobotPose.java"          "%DEST%\wrapper"
call :cp "%ROOT%wrapper\MecanumWrangler.java"    "%DEST%\wrapper"
call :cp "%ROOT%wrapper\BallWrangler.java"       "%DEST%\wrapper"
call :cp "%ROOT%wrapper\PedroWrangler.java"      "%DEST%\wrapper"
call :cp "%ROOT%wrapper\BallHunt.java"           "%DEST%\wrapper"

call :cp "%ROOT%tests\MockMotor.java"            "%DEST%\tests"
call :cp "%ROOT%tests\Pedro3LibSelfTest.java"   "%DEST%\tests"

call :cp "%ROOT%tests\README.md"                 "%PROJ%"

echo.
echo Done - 16 sources into %DEST%
echo.
echo Next:
echo   1. add implementation 'com.pedropathing:revhub:3.0.1' to TeamCode/build.gradle
echo   2. edit teamcode\pedroPathing\Constants.java for your robot (names, pods, ticks)
echo   3. build, then run "Pedro3 Lib Self-Test" from the Hive Vision group
echo.
endlocal
exit /b 0

:cp
if not exist "%~2" mkdir "%~2"
if not exist "%~2\" (echo FAILED to create %~2 & exit /b 1)
copy /y "%~1" "%~2\" >nul
if errorlevel 1 (echo FAILED to copy %~1 & exit /b 1)
exit /b 0