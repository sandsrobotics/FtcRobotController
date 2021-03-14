package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    protected boolean clawClosed = true;
    protected boolean motorReset = false;

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

    void initGrabberPos()
    {
        if(robot.robotHardware.grabberLimitSwitch.getState())
        {
            int pos = 0;
            while(robot.robotHardware.grabberLimitSwitch.getState() && !robot.stop())
            {
                pos -= grabberSettings.homingSpeed;
                robot.robotHardware.grabberLifterMotor.setTargetPosition(pos);
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.robotHardware.grabberLifterMotor.setTargetPosition(0);
        robot.robotHardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        robot.addTelemetry("grabber pos", robot.robotHardware.grabberLifterMotor.getCurrentPosition());
    }

    void moveMotors()
    {
        robot.robotHardware.grabberLifterMotor.setTargetPosition(setEncoderPos);
        stopMotor();
    }

    void moveServos()
    {
        for(int i = 0; i < 2; i++)
        {
            robot.robotHardware.grabberServos.get(i).setPosition(setServoPositions[i]);
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
        while(robot.robotHardware.grabberLifterMotor.isBusy() && !robot.stop() && waitForMotor) { if(stopMotor()) break; }
    }

    boolean stopMotor()
    {
        if(!robot.robotHardware.grabberLimitSwitch.getState() && setEncoderPos <= 5)
        {
            if(!motorReset)
            {
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.robotHardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorReset = true;
            }
            if(robot.robotHardware.grabberLifterMotor.getPower() != 0) robot.robotHardware.grabberLifterMotor.setPower(0);
            return true;
        }
        else {
            if (robot.robotHardware.grabberLifterMotor.getPower() != grabberSettings.motorPower) robot.robotHardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
            motorReset = false;
        }
        return false;
    }

    void setServosToPos(double[] servoPos, boolean waitForServos)
    {
        setServoPositions[0] = servoPos[0];
        setServoPositions[1] = servoPos[1];
        moveServos();
        if(waitForServos) robot.delay(grabberSettings.servoCloseTime);
    }

    ////////
    //auto//
    ////////
    void autoDrop()
    {
        if(robot.robotUsage.useDrive && robot.robotUsage.usePositionThread && robot.robotUsage.usePositionTracking) {
            setGrabberToPos(grabberSettings.putOverPos, false);
            robot.movement.moveToPosition(new double[]{robot.position.currentRobotPosition[0], -123, 90}, robot.movement.movementSettings.finalPosSettings);
            setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
            robot.movement.moveToPosition(robot.position.getPositionWithOffset(0, 10, 0), robot.movement.movementSettings.losePosSettings);
            setGrabberToPos(grabberSettings.capturePos, false);
        }
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
    protected int capturePos = 1440; //position of grabber arm when grabbing a wobble goal
    protected int horizontalPos = 0; //position of grabber arm when in storage
    protected int putOverPos = 1000; //position of grabber arm to put the wobble goal over the wall
    protected int restPos = 1550; //position of grabber arm when at rest on the side of robot

    //servo pos
    protected double[] servoRestPositions = {.2, .6};
    protected double[] servoGrabPositions = {.9, .1};
    protected int servoCloseTime = 120; // time for the servos to close/open(in ms)

    //controls
    protected GamepadButtonManager moveGrabberStick = new GamepadButtonManager(GamepadButtons.combinedTRIGGERS);//manual adjust of grabber
    protected double stickTolerance = .03;
    protected double stickToTicksMultiplier = 20;
    protected GamepadButtonManager grabButton = new GamepadButtonManager(GamepadButtons.leftBUMPER);//open and close the grabber claws
    //preset positions
    protected GamepadButtonManager captureButton = new GamepadButtonManager(GamepadButtons.dpadDOWN);
    protected GamepadButtonManager horizontalButton = new GamepadButtonManager(GamepadButtons.dpadUP);
    protected GamepadButtonManager putOverButton = new GamepadButtonManager(GamepadButtons.dpadLEFT);
    protected GamepadButtonManager restPosButton = new GamepadButtonManager(GamepadButtons.dpadRIGHT);

    //homing
    int homingSpeed = 50;


    GrabberSettings(){}
}
