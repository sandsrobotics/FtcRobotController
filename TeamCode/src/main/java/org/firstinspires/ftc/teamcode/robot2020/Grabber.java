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
    }

    void setFromControls(Gamepad gamepad)
    {
        //set encoder
        if(grabberSettings.captureButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.capturePos;
        else if(grabberSettings.horizontalButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.horizontalPos;
        else if(grabberSettings.putOverButton.getButtonPressed(gamepad)) setEncoderPos = grabberSettings.putOverPos;

        if(Math.abs(grabberSettings.moveGrabberStick.getSliderValue(gamepad)) >= grabberSettings.stickTolerance) setEncoderPos += grabberSettings.moveGrabberStick.getSliderValue(gamepad) * grabberSettings.stickToTicksMultiplier;
        if(setEncoderPos > grabberSettings.maxMotorPos) setEncoderPos = grabberSettings.maxMotorPos;
        else if(setEncoderPos < 0) setEncoderPos = 0;

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

        robot.motorConfig.grabberLifterMotor.setTargetPosition(setEncoderPos);

        for(int i = 0; i < 2; i++)
        {
            robot.motorConfig.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
    }
}

class GrabberSettings
{
    ////////////////
    //user defined//
    ////////////////
    protected int maxMotorPos = 1680*2;
    protected double motorPower = .5;

    //preset lifter functions
    protected int capturePos = 150;
    protected int horizontalPos = 1680;
    protected int putOverPos = 450;

    //servo pos
    protected double[] servoRestPositions = {.5, .5};
    protected double[] servoGrabPositions = {.5, .5};

    //controls
    protected GamepadButtons moveGrabberStick = GamepadButtons.combinedTRIGGERS;
    protected double stickTolerance = .03;
    protected double stickToTicksMultiplier = 5;
    protected GamepadButtons captureButton = GamepadButtons.A;
    protected GamepadButtons horizontalButton = GamepadButtons.B;
    protected GamepadButtons putOverButton = GamepadButtons.X;
    protected GamepadButtons grabButton = GamepadButtons.Y;


    GrabberSettings(){}
}
