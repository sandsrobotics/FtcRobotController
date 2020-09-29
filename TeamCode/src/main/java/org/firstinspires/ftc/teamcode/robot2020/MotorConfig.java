package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;
import java.util.List;

public class MotorConfig
{
    //////////////////
    //user variables//
    //////////////////
    protected boolean[] flipMotorDir = {true, true, false, false};
    protected int leftTopMotorNum = 0;
    protected int leftBottomMotorNum = 2;
    protected int rightTopMotorNum = 1;
    protected int rightBottomMotorNum = 3;

    protected DcMotor leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    protected List<DcMotor> motors;

    //other class
    Robot robot;

    public MotorConfig(Robot robot)
    {
        this.robot = robot;
    }

    ////////
    //init//
    ////////
    public void initMotors()
    {
        leftTopMotor = robot.hardwareMap.dcMotor.get("motor" + leftTopMotorNum);
        leftBottomMotor = robot.hardwareMap.dcMotor.get("motor" + leftBottomMotorNum);
        rightTopMotor = robot.hardwareMap.dcMotor.get("motor" + rightTopMotorNum);
        rightBottomMotor = robot.hardwareMap.dcMotor.get("motor" + rightBottomMotorNum);
        motors = Arrays.asList(leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor);

        int i = 0;
        for(DcMotor motor:motors)
        {
            if(flipMotorDir[i]) motor.setDirection(DcMotor.Direction.REVERSE);
            i++;
        }

    }
    public void resetEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    ///////////////////
    //set motor modes//
    ///////////////////
    public void setMotorsToCoast()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setMotorsToBrake()
    {
        for(DcMotor motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setMotorsToRunWithoutEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setMotorsToRunWithEncoders()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setMotorsToRunToPosition()
    {
        for(DcMotor motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    ///////////////
    //motor power//
    ///////////////
    public void stopMotors()
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(0);
        }
    }

    public void setMotorsToPower(double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setPower(power);
        }
    }

    public void setMotorsToSeparatePowersArray(double[] powers)
    {
        int i = 0;
        for(DcMotor motor: motors)
        {
            motor.setPower(powers[i]);
            i++;
        }
    }

    public double[] getMotorPowers()
    {
        double[] arr = new double[4];
        int i = 0;
        for(DcMotor motor: motors)
        {
            arr[i] = motor.getPower();
            i++;
        }
        return arr;
    }

    ///////////////
    //motor ticks//
    ///////////////
    public void setMotorsToPosition(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorsForward(int ticks, double power)
    {
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorForwardSeparateAmount(int[] ticks, double power)
    {
        int i = 0;
        for(DcMotor motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }

    public int[] getMotorPositions()
    {
        int[] arr = new int[4];
        int i = 0;
        for(DcMotor motor: motors)
        {
            arr[i] = motor.getCurrentPosition();
            i++;
        }
        return arr;
    }

    /////////
    //other//
    /////////
    void waitForMotorsToFinish()
    {
        while((robot.motorConfig.leftTopMotor.isBusy() || robot.motorConfig.leftBottomMotor.isBusy() || robot.motorConfig.rightTopMotor.isBusy() || robot.motorConfig.rightBottomMotor.isBusy()) && !Robot.emergencyStop && !robot.gamepad1.back && !robot.gamepad2.back){}
    }
}
