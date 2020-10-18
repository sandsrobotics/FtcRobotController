package org.firstinspires.ftc.teamcode.robot2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class MotorConfig
{
    //////////////////
    //user variables//
    //////////////////
    //drive motors
    protected boolean[] flipDriveMotorDir = {true, true, false, false};
    protected String leftTopMotorNum = "0";
    protected String leftBottomMotorNum = "2";
    protected String rightTopMotorNum = "1";
    protected String rightBottomMotorNum = "3";
    //launcher motors
    protected boolean[] flipLauncherMotorDir = {false, false, false};
    protected String launcherWheelMotorNum = "0B";
    protected String launcherHolderMotorNum = "1B";
    protected String launcherServoNum = "4";

    /////////
    //other//
    /////////
    //drive
    protected DcMotorEx leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    protected List<DcMotorEx> driveMotors;
    //launcher
    DcMotorEx launcherWheelMotor, launcherHolderMotor;
    Servo launcherServo;
    protected List<DcMotorEx> launcherMotors;
    //other class
    Robot robot;

    public MotorConfig(Robot robot)
    {
        this.robot = robot;
    }

    ////////
    //init//
    ////////
    public void initDriveMotors()
    {
        leftTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + leftTopMotorNum);
        leftBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + leftBottomMotorNum);
        rightTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + rightTopMotorNum);
        rightBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + rightBottomMotorNum);
        driveMotors = Arrays.asList(leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor);

        int i = 0;
        for(DcMotor motor:driveMotors)
        {
            if(flipDriveMotorDir[i]) motor.setDirection(DcMotor.Direction.REVERSE);
            i++;
        }

        resetDriveEncodersList(driveMotors);
        setMotorsToBrakeList(driveMotors);
        setMotorsToRunWithEncodersList(driveMotors);
    }

    public void initLauncherMotors()
    {
        launcherWheelMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + launcherWheelMotorNum);
        launcherHolderMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + launcherHolderMotorNum);
        launcherServo = robot.hardwareMap.servo.get("servo" + launcherServoNum);
        launcherMotors = Arrays.asList(launcherWheelMotor, launcherHolderMotor);

    }

    public void resetDriveEncodersList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //public void resetMotorEncoder()

    ///////////////////
    //set motor modes//
    ///////////////////
    public void setDriveMotorsToCoastList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setMotorsToBrakeList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void setMotorsToRunWithoutEncodersList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setMotorsToRunWithEncodersList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setMotorsToRunToPositionList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    ///////////////
    //motor power//
    ///////////////
    public void stopMotorsList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setPower(0);
        }
    }

    public void setMotorsToPowerList(List<DcMotorEx> motors, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setPower(power);
        }
    }

    public void setMotorsToSeparatePowersArrayList(List<DcMotorEx> motors, double[] powers)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setPower(powers[i]);
            i++;
        }
    }

    public double[] getMotorPowersList(List<DcMotorEx> motors)
    {
        double[] arr = new double[motors.size()];
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            arr[i] = motor.getPower();
            i++;
        }
        return arr;
    }

    ///////////////
    //motor ticks//
    ///////////////
    public void setMotorsToPositionList(List<DcMotorEx> motors, int ticks, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorsForwardList(List<DcMotorEx> motors, int ticks, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorForwardSeparateAmountList(List<DcMotorEx> motors, int[] ticks, double power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }

    public int[] getMotorPositionsList(List<DcMotorEx> motors)
    {
        int[] arr = new int[motors.size()];
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            arr[i] = motor.getCurrentPosition();
            i++;
        }
        return arr;
    }

    public double[] getMotorVelocitiesList(List<DcMotorEx> motors)
    {
        double[] arr = new double[motors.size()];
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            arr[i] = motor.getVelocity();
            i++;
        }
        return arr;
    }

    /////////
    //other//
    /////////
    public void waitForMotorsToFinishList(List<DcMotorEx> motors)
    {
        int totalMotorsDone = 0;
        while(totalMotorsDone < motors.size())
        {
            totalMotorsDone = 0;
            if (robot.stop()) break;
            for (DcMotorEx motor : motors)
            {
                if(!motor.isBusy()) totalMotorsDone++;
            }
        }
    }
}
