package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Grabber {
    Robot robot;

    ////////////////
    //user defined//
    ////////////////
    protected int maxMotorPos = 1680*2;
    protected double motorPower = .5;

    //preset lifter functions
    protected int capturePos = 150;
    protected int HorizontalPos = 1680;
    protected int putOverPos = 450;

    //servo pos
    protected double[] servoRestPositions = {.9, .1};
    protected double[] servoGrabPositions = {.2, .6};


    //controls
    GamepadButtons moveGrabberStick = GamepadButtons.combinedTRIGGERS;
    double stickTolerance = .03;
    double stickToTicksMultiplier = 5;
    GamepadButtons captureButton = GamepadButtons.A;
    GamepadButtons HorizontalButton = GamepadButtons.B;
    GamepadButtons putOverButton = GamepadButtons.X;
    GamepadButtons grabButton = GamepadButtons.Y;

    /////////////
    //variables//
    /////////////
    protected int setEncoderPos;
    protected double[] setServoPositions = new double[2];

    Grabber(Robot robot){this.robot = robot;}

    void init()
    {
        robot.motorConfig.grabberLifterMotor.setTargetPosition(0);
        robot.motorConfig.grabberLifterMotor.setPower(motorPower);
        robot.motorConfig.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(int i = 0; i < 2; i++)
        {
            setServoPositions[i] = servoRestPositions[i];
            robot.motorConfig.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
    }

    void setFromControls(Gamepad gamepad)
    {
        //set encoder
        if(captureButton.getButtonPressed(gamepad)) setEncoderPos = capturePos;
        else if(HorizontalButton.getButtonPressed(gamepad)) setEncoderPos = HorizontalPos;
        else if(putOverButton.getButtonPressed(gamepad)) setEncoderPos = putOverPos;

        if(Math.abs(moveGrabberStick.getSliderValue(gamepad)) >= stickTolerance) setEncoderPos += moveGrabberStick.getSliderValue(gamepad) * stickToTicksMultiplier;
        if(setEncoderPos > maxMotorPos) setEncoderPos = maxMotorPos;
        else if(setEncoderPos < 0) setEncoderPos = 0;

        //set servo
        if(grabButton.getButtonHeld(gamepad))
        {
            setServoPositions[0] = servoGrabPositions[0];
            setServoPositions[1] = servoGrabPositions[1];
        }
        else
        {
            setServoPositions[0] = servoRestPositions[0];
            setServoPositions[1] = servoRestPositions[1];
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
