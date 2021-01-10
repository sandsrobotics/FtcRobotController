package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Grabber {

    //user variables below

    /////////////
    //variables//
    /////////////
    Robot robot;
    GrabberSettings grabberSettings;
    DigitalChannel limitSwitch;

    protected int setEncoderPos;
    protected double[] setServoPositions = new double[2];
    protected boolean clawClosed = false;

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

    void init()
    {
        robot.motorConfig.grabberLifterMotor.setTargetPosition(0);
        robot.motorConfig.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.motorConfig.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(int i = 0; i < 2; i++)
        {
            setServoPositions[i] = grabberSettings.servoRestPositions[i];
            robot.motorConfig.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
        limitSwitch = robot.hardwareMap.get(DigitalChannel.class, grabberSettings.limitSwitchName);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    void initGrabberPos()
    {
        if(limitSwitch.getState())
        {
            int pos = 0;
            while(limitSwitch.getState() && !robot.stop())
            {
                pos -= grabberSettings.homingSpeed;
                robot.motorConfig.grabberLifterMotor.setTargetPosition(pos);
                robot.motorConfig.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        robot.motorConfig.grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorConfig.grabberLifterMotor.setTargetPosition(0);
        robot.motorConfig.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.motorConfig.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setFromControls(Gamepad gamepad)
    {
        //set encoder
        if(grabberSettings.captureButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.capturePos;
        else if(grabberSettings.horizontalButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.horizontalPos;
        else if(grabberSettings.putOverButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.putOverPos;

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

    void runForTeleop(Gamepad gamepad)
    {
        setFromControls(gamepad);
        moveAll();
    }

    void moveMotors()
    {
        robot.motorConfig.grabberLifterMotor.setTargetPosition(setEncoderPos);
    }
    void moveServos()
    {
        for(int i = 0; i < 2; i++)
        {
            robot.motorConfig.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
    }
    void moveAll()
    {
        moveMotors();
        moveServos();
    }

    void setGrabberToPos(int pos)
    {
        setEncoderPos = pos;
        moveMotors();
        while(robot.motorConfig.grabberLifterMotor.isBusy() && !robot.stop()) { }
    }

    void setServosToPos(double[] servoPos)
    {
        setServoPositions[0] = servoPos[0];
        setServoPositions[1] = servoPos[1];
        moveServos();
    }
}

class GrabberSettings
{
    ////////////////
    //user defined//
    ////////////////
    protected int maxMotorPos = 1660;
    protected int minMotorPos = 0;
    protected double motorPower = .75;

    //preset lifter functions
    protected int capturePos = 1500;
    protected int horizontalPos = 0;
    protected int putOverPos = 1000;

    //servo pos
    protected double[] servoRestPositions = {.2, .6};
    protected double[] servoGrabPositions = {.9, .1};

    //controls
    protected GamepadButtons moveGrabberStick = GamepadButtons.combinedTRIGGERS;
    protected double stickTolerance = .03;
    protected double stickToTicksMultiplier = 20;
    protected GamepadButtons captureButton = GamepadButtons.A;
    protected GamepadButtons horizontalButton = GamepadButtons.B;
    protected GamepadButtons putOverButton = GamepadButtons.X;
    protected GamepadButtons grabButton = GamepadButtons.Y;

    //homing
    String limitSwitchName = "digital0B";
    int homingSpeed = 50;


    GrabberSettings(){}
}
