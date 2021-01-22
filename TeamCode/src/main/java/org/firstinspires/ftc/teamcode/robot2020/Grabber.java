package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Grabber {

    //user variables below

    /////////////
    //variables//
    /////////////
    Robot robot;
    GrabberSettings grabberSettings;

    protected int setEncoderPos;
    protected double[] setServoPositions = new double[2];
    protected boolean clawClosed = false;
    protected boolean motorStopped = false;

    Grabber(Robot robot)
    {
        this.robot = robot;
        grabberSettings = new GrabberSettings();
    }
    Grabber(Robot robot, GrabberSettings grabberSettings)
    {
        this.robot = robot;
        this.grabberSettings = grabberSettings;
    }

    void initHardware()
    {

    }

    void initGrabberPos()
    {
        if(limitSwitch.getState())
        {
            int pos = 0;
            while(limitSwitch.getState() && !robot.stop())
            {
                pos -= grabberSettings.homingSpeed;
                robot.hardware.grabberLifterMotor.setTargetPosition(pos);
                robot.hardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        robot.hardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hardware.grabberLifterMotor.setTargetPosition(0);
        robot.hardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.hardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setFromControls(Gamepad gamepad)
    {
        //set encoder
        if(grabberSettings.captureButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.capturePos;
        else if(grabberSettings.horizontalButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.horizontalPos;
        else if(grabberSettings.putOverButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.putOverPos;
        else if(grabberSettings.restPosButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.restPos;

        if(Math.abs(grabberSettings.moveGrabberStick.getSliderValue(gamepad)) >= grabberSettings.stickTolerance) {
            setEncoderPos += grabberSettings.moveGrabberStick.getSliderValue(gamepad) * grabberSettings.stickToTicksMultiplier;
            if (setEncoderPos > grabberSettings.maxMotorPos) setEncoderPos = grabberSettings.maxMotorPos;
            else if (setEncoderPos < grabberSettings.minMotorPos) setEncoderPos = grabberSettings.minMotorPos;
        }

        //set servo
        if(grabberSettings.grabButton.getButtonPressed(gamepad)) { clawClosed = !clawClosed; }
        if(clawClosed)
        {
            setServoPositions[0] = grabberSettings.servoGrabPositions[0];
            setServoPositions[1] = grabberSettings.servoGrabPositions[1];
        }
        else
        {
            setServoPositions[0] = grabberSettings.servoRestPositions[0];
            setServoPositions[1] = grabberSettings.servoRestPositions[1];
        }
    }

    void runForTeleOp(Gamepad gamepad, boolean useTelemetry)
    {
        setFromControls(gamepad);
        moveAll();
        if(useTelemetry) teleOpTelemetry();
    }

    void teleOpTelemetry()
    {

    }

    void moveMotors()
    {
        robot.hardware.grabberLifterMotor.setTargetPosition(setEncoderPos);
        stopMotor();
    }
    void moveServos()
    {
        for(int i = 0; i < 2; i++)
        {
            robot.hardware.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
    }
    void moveAll()
    {
        moveMotors();
        moveServos();
    }

    void setGrabberToPos(int pos, boolean waitForMotor)
    {
        setEncoderPos = pos;
        moveMotors();
        while(robot.hardware.grabberLifterMotor.isBusy() && !robot.stop() && waitForMotor) { if(stopMotor()) break; }
    }

    boolean stopMotor()
    {
        if(!limitSwitch.getState() && setEncoderPos <= 5 && !motorStopped)
        {
            robot.hardware.grabberLifterMotor.setPower(0);
            motorStopped = true;
            return true;
        }
        else if(motorStopped)
        {
            robot.hardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
            motorStopped = false;
        }
        return false;
    }

    void setServosToPos(double[] servoPos, boolean waitForServos)
    {
        setServoPositions[0] = servoPos[0];
        setServoPositions[1] = servoPos[1];
        moveServos();
        robot.delay(grabberSettings.servoCloseTime);
    }
}

class GrabberSettings
{
    ////////////////
    //user defined//
    ////////////////
    protected int maxMotorPos = 1600;
    protected int minMotorPos = 0;
    protected double motorPower = .75;

    //preset lifter functions
    protected int capturePos = 1440;
    protected int horizontalPos = 0;
    protected int putOverPos = 1000;
    protected int restPos = 0;

    //servo pos
    protected double[] servoRestPositions = {.2, .6};
    protected double[] servoGrabPositions = {.9, .1};
    protected int servoCloseTime = 100; // time for the servos to close/open(in ms)

    //controls
    protected GamepadButtonManager moveGrabberStick = new GamepadButtonManager(GamepadButtons.combinedTRIGGERS);
    protected double stickTolerance = .03;
    protected double stickToTicksMultiplier = 20;
    protected GamepadButtonManager captureButton = new GamepadButtonManager(GamepadButtons.A);
    protected GamepadButtonManager horizontalButton = new GamepadButtonManager(GamepadButtons.B);
    protected GamepadButtonManager putOverButton = new GamepadButtonManager(GamepadButtons.X);
    protected GamepadButtonManager grabButton = new GamepadButtonManager(GamepadButtons.Y);
    protected GamepadButtonManager restPosButton = new GamepadButtonManager(GamepadButtons.B);

    //homing
    String limitSwitchName = "digital0B";
    int homingSpeed = 50;


    GrabberSettings(){}
}
