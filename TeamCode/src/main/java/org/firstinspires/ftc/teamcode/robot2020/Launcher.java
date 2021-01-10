package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;

@Config
public class Launcher {




    ///////////////////
    //other variables//
    ///////////////////
    //servo and motor
    double spinMultiplier;

    //calibration
    protected ArrayList<List<Double>> calibrationValues;
    protected ArrayList<List<Double>> formattedCalibrationValues;

    //other
    boolean runWheelOnTrigger = false;
    boolean moveIntakeMotorForward = false;
    float intakeMotorPower = 0;
    double targetWheelRpm = 0;
    Robot robot;
    LauncherSettings launcherSettings;

    Launcher(Robot robot)
    {
        launcherSettings = new LauncherSettings();
        this.robot = robot;
        spinMultiplier = 60 / launcherSettings.ticksPerRev * launcherSettings.gearRatio;
    }

    Launcher(Robot robot, LauncherSettings launcherSettings)
    {
        this.launcherSettings = launcherSettings;
        this.robot = robot;
        spinMultiplier = 60 / launcherSettings.ticksPerRev * launcherSettings.gearRatio;
    }

    ///////////////
    //Calibration//
    ///////////////
    void getCalibration()
    {
        try
        {
            InputStream is = getClass().getClassLoader().getResourceAsStream(launcherSettings.calibrationFileDir + "/" + launcherSettings.calibrationFileName);
            if(is == null) throw new Exception("file directory or name are incorrect");
            calibrationValues = readFile(is);
        }
        catch (Exception e) {robot.addTelemetry("error", e.toString());}
    }

    ArrayList<List<Double>> readFile(InputStream is)
    {
        ArrayList<List<Double>> out = new ArrayList<>();
        try
        {
            BufferedReader reader = new BufferedReader(new InputStreamReader(is));
            String Line;
            while ((Line = reader.readLine()) != null)
            {
                List<Double> values = new ArrayList<>();
                boolean valid = true;
                String[] elements = Line.split(",");
                for(String element:elements)
                {
                    try { values.add(Double.parseDouble(element)); }
                    catch (Exception e){ valid = false; }
                }
                if(valid) out.add(values);
            }
        }
        catch (IOException e)
        {
            if(robot.robotSettings.debug_methods)robot.addTelemetry("error", e.toString());
        }
        return out;
    }

    double getRPMFromCalibration(int goalNum, double distance)
    {
        if(calibrationValues.size() == 0) robot.addTelemetry("error in Launcher.getRPMFromCalibration: ", "calibration values have not been loaded");
        else if(goalNum < 1 || goalNum >  3) robot.addTelemetry("error in Launcher.getRPMFromCalibration: ", "goalNum has not been set correctly");
        else
        {
            int row = 0;
            for(int i = 0; i < calibrationValues.size(); i++)
            {
                if(calibrationValues.get(i).get(0) == distance) return calibrationValues.get(i).get(goalNum);
                else if(calibrationValues.get(i).get(0) > distance) row = i;
            }

            double RPMPerInch;
            if(row == 0)
            {
                RPMPerInch = calibrationValues.get(0).get(goalNum) / calibrationValues.get(0).get(0);
            }
            else
            {
                RPMPerInch = (calibrationValues.get(row).get(goalNum) - calibrationValues.get(row - 1).get(goalNum)) / (calibrationValues.get(row).get(0) - calibrationValues.get(row - 1).get(0));
            }

            return RPMPerInch*distance;
        }

        return -1;
    }

    ///////////////////////////
    //launcher opmode control//
    ///////////////////////////
    void telemetryDataOut()
    {
        if(runWheelOnTrigger) robot.addTelemetry("Mode: ", "Run on trigger");
        else robot.addTelemetry("Mode: ", "Run using RPM");
        robot.addTelemetry("RPM", getPRM());
        robot.addTelemetry("Set RPM", targetWheelRpm);
        robot.sendTelemetry();
    }

    void setLauncherServo(Gamepad gamepad)
    {
        if(launcherSettings.launchButton.getButtonHeld(gamepad,launcherSettings.buttonHoldTime)) autoLaunch();
        else if(launcherSettings.launchButton.getButtonReleased(gamepad)) moveLaunchServo();
    }

    void setLauncherWheelMotor(Gamepad gamepad)
    {
        //inputs
        if(launcherSettings.revDecreaseButton.getButtonPressed(gamepad) || launcherSettings.revDecreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime))
        {
            targetWheelRpm -= launcherSettings.RPMIncrements;
            if(targetWheelRpm < 0) targetWheelRpm = 0;
        }

        if(launcherSettings.revIncreaseButton.getButtonPressed(gamepad) || launcherSettings.revIncreaseButton.getButtonHeld(gamepad, launcherSettings.buttonHoldTime))
        {
            targetWheelRpm += launcherSettings.RPMIncrements;
            if(targetWheelRpm > launcherSettings.maxRPM) targetWheelRpm = launcherSettings.maxRPM;
        }

        if(launcherSettings.revModeButton.getButtonPressed(gamepad)) { runWheelOnTrigger =! runWheelOnTrigger; }

        //setting motor
        if (runWheelOnTrigger) robot.motorConfig.launcherWheelMotor.setPower(launcherSettings.revPowerSlide.getSliderValue(gamepad));
        else setRPM();
    }

    void setLauncherIntakeMotor(Gamepad gamepad)
    {
        intakeMotorPower = 0;

        if(launcherSettings.intakeInButton.getButtonPressed(gamepad)) moveIntakeMotorForward = !moveIntakeMotorForward;
        if(launcherSettings.intakeOutSlider.getSliderValue(gamepad) > launcherSettings.sliderTolerance){
            moveIntakeMotorForward = false;
            intakeMotorPower = -launcherSettings.intakeOutSlider.getSliderValue(gamepad);
        }
        if(moveIntakeMotorForward) intakeMotorPower = 1;

        robot.motorConfig.launcherMotors.get(1).setPower(intakeMotorPower);
    }

    void opModeRun(Gamepad gamepad, boolean telemetry)
    {
        setLauncherServo(gamepad);
        setLauncherWheelMotor(gamepad);
        setLauncherIntakeMotor(gamepad);
        if(telemetry)telemetryDataOut();
    }

    ///////////////////////////////
    //autonomous launcher control//
    ///////////////////////////////
    void autonomousLaunchDisk()
    {
        double RPM = getRPMFromCalibration(3, getDistanceToGoal(true));
        if(RPM == -1) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "unable to get RPM");
        else if(robot.movement == null) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to move");
        else if(!robot.robotUsage.usePositionTracking) robot.addTelemetry("error in Launcher.autonomousLaunchDisk: ", "robot is unable to track position");
        else
        {
            setRPM(RPM);
            goToShootingPos();
            waitForRPMInTolerance(1000);
            autoLaunch();
        }
    }

    void goToShootingPos()
    {
        if(robot.robotUsage.usePositionTracking && robot.movement != null)
        {
            if (robot.position.currentRobotPosition[1] > launcherSettings.minLaunchDistance) { robot.movement.moveToPosition(new double[]{robot.position.currentRobotPosition[0], launcherSettings.minLaunchDistance, getAngleToPointToPosition()}, new double[]{.5, .5, .5}, 10, 20000, .75); }
            else { robot.movement.turnToAngle(getAngleToPointToPosition(), .5, 10, 20000, .75); }
        }
    }

    ////////////////
    //calculations//
    ////////////////
    double getAngleToPointToPosition(double xPos, double yPos, double angleOffset, boolean useMinLaunchDis)
    {
        if(robot.robotUsage.usePositionTracking)
        {
            double XDiff = xPos - robot.position.currentRobotPosition[0];
            double YDiff;
            if (useMinLaunchDis && robot.position.currentRobotPosition[1] > launcherSettings.minLaunchDistance)
                YDiff = -launcherSettings.minLaunchDistance;
            else YDiff = yPos - robot.position.currentRobotPosition[1];

            return robot.scaleAngle(Math.toDegrees(Math.atan(XDiff / YDiff)) + angleOffset);
        }
        return 0;
    }

    double getAngleToPointToPosition()
    {
        return getAngleToPointToPosition(0,0,0, true);
    }

    boolean isRPMInTolerance(double targetWheelRpm, double RPMTolerance, double maxRPMAcceleration)
    {
        double lastRPM = getPRM();
        robot.delay(10);
        double RPM = getPRM();
        return Math.abs(RPM - targetWheelRpm) <= RPMTolerance && Math.abs(lastRPM - RPM) * 100 <= maxRPMAcceleration;
    }

    boolean isRPMInTolerance()
    {
        return isRPMInTolerance(targetWheelRpm, launcherSettings.RPMTolerance, launcherSettings.maxRPMAcceleration);
    }

    double getPRM()
    {
        return  robot.motorConfig.launcherWheelMotor.getVelocity() * spinMultiplier;
    }

    double getDistanceToGoal(boolean useMinLaunchDistance)
    {
        if(robot.robotUsage.usePositionTracking)
        {
            if (robot.position.currentRobotPosition[1] > launcherSettings.minLaunchDistance || !useMinLaunchDistance)
                return Math.sqrt(Math.pow(robot.position.currentRobotPosition[0], 2) + Math.pow(robot.position.currentRobotPosition[1], 2));
            return Math.sqrt(Math.pow(robot.position.currentRobotPosition[0], 2) + Math.pow(launcherSettings.minLaunchDistance, 2));
        }
        if(robot.robotSettings.debug_methods) robot.addTelemetry("error in Launcher.getDistanceToGoal: ", "robot cannot find distance because it does not know its position");
        return -1;
    }

    /////////
    //other//
    /////////
    void setRPM(double RPM)
    {
        targetWheelRpm = RPM;
        robot.motorConfig.launcherWheelMotor.setVelocity(RPM / spinMultiplier);
    }

    void setRPM()
    {
        setRPM(targetWheelRpm);
    }

    void waitForRPMInTolerance(long maxMs, double targetWheelRpm, double RPMTolerance, double maxRPMVelocity)
    {
        long startTime = System.currentTimeMillis();
        while(!robot.stop() && System.currentTimeMillis() - startTime < maxMs)
        {
            if(isRPMInTolerance(targetWheelRpm, RPMTolerance, maxRPMVelocity)) break;
        }
    }

    void waitForRPMInTolerance(long maxMs)
    {
        waitForRPMInTolerance(maxMs, targetWheelRpm, launcherSettings.RPMTolerance, launcherSettings.maxRPMAcceleration);
    }

    void moveLaunchServo(long actuatorTime)
    {
        robot.motorConfig.launcherServo.setPosition(launcherSettings.servoLaunchAngle);
        robot.delay(actuatorTime);
        robot.motorConfig.launcherServo.setPosition(launcherSettings.servoRestAngle);
    }

    void moveLaunchServo()
    {
        moveLaunchServo(launcherSettings.servoMoveTime);
    }

    void autoLaunch()
    {
        if(isRPMInTolerance())
        {
            moveLaunchServo();
            robot.delay(launcherSettings.servoMoveTime);
        }
    }
}// class end

class LauncherSettings
{
    //////////////////
    //user variables//
    //////////////////
    //controls
    GamepadButtons revIncreaseButton = GamepadButtons.B;
    GamepadButtons revDecreaseButton = GamepadButtons.X;
    GamepadButtons revPowerSlide = GamepadButtons.leftTRIGGER;
    GamepadButtons revModeButton = GamepadButtons.Y;
    GamepadButtons launchButton = GamepadButtons.A;
    GamepadButtons intakeInButton = GamepadButtons.rightBUMPER;
    GamepadButtons intakeOutSlider = GamepadButtons.rightTRIGGER;
    double sliderTolerance = .1;
    int buttonHoldTime = 500;

    //servo and motor config
    double ticksPerRev = 28;
    double gearRatio = 1;
    double maxRPM = 6000;
    double servoRestAngle = 0;
    double servoLaunchAngle = 1;
    int servoMoveTime = 250;

    //calibration data
    protected String calibrationFileDir = "assets";
    protected String calibrationFileName =  "Launcher Config - Test.csv";

    //other
    double RPMIncrements = 50;
    double RPMTolerance = 100;
    double maxRPMAcceleration = 10; // acceleration measured in RPM/s
    double minLaunchDistance = -52; //this is how far the robot has to be from goal to launch - IN INCHES!!!

    LauncherSettings(){};
}