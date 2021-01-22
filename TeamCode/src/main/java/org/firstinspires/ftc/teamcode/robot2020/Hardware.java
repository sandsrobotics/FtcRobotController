package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class Hardware
{

    /////////
    //other//
    /////////
    //drive
    protected DcMotorEx leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor;
    protected List<DcMotorEx> driveMotors;

    //launcher
    protected DcMotorEx launcherWheelMotor, launcherIntakeMotor;
    protected Servo launcherServo;

    //grabber
    protected DcMotorEx grabberLifterMotor;
    protected Servo grabberLeftServo, grabberRightServo;
    protected List<Servo> grabberServos;
    protected DigitalChannel grabberLimitSwitch;

    //other class
    Robot robot;
    MotorConfigSettings motorConfigSettings;

    public Hardware(Robot robot)
    {
        motorConfigSettings = new MotorConfigSettings();
        this.robot = robot;
    }
    public Hardware(Robot robot, MotorConfigSettings motorConfigSettings)
    {
        this.motorConfigSettings = motorConfigSettings;
        this.robot = robot;
    }

    ////////
    //init//
    ////////
    public void initDriveMotors()
    {
        leftTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + motorConfigSettings.leftTopMotorNum);
        leftBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + motorConfigSettings.leftBottomMotorNum);
        rightTopMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + motorConfigSettings.rightTopMotorNum);
        rightBottomMotor = robot.hardwareMap.get(DcMotorEx.class,"motor" + motorConfigSettings.rightBottomMotorNum);
        driveMotors = Arrays.asList(leftTopMotor, leftBottomMotor, rightTopMotor, rightBottomMotor);

        int i = 0;
        for(DcMotor motor:driveMotors)
        {
            if(motorConfigSettings.flipDriveMotorDir[i]) motor.setDirection(DcMotor.Direction.REVERSE);
            i++;
        }

        initMotorSettings(driveMotors);
    }

    public void initLauncherMotors()
    {
        launcherWheelMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + motorConfigSettings.launcherWheelMotorNum);
        launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, motorConfigSettings.launcherWheelMotorPID);
        launcherIntakeMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + motorConfigSettings.launcherIntakeMotorNum);

        launcherServo = robot.hardwareMap.servo.get("servo" + motorConfigSettings.launcherServoNum);

        if(motorConfigSettings.flipLauncherMotorDir[0]) launcherWheelMotor.setDirection(DcMotor.Direction.REVERSE);
        if(motorConfigSettings.flipLauncherMotorDir[1]) launcherIntakeMotor.setDirection(DcMotor.Direction.REVERSE);
        if(motorConfigSettings.flipLauncherMotorDir[2]) launcherServo.setDirection(Servo.Direction.REVERSE);

        initMotorSettings(launcherWheelMotor);
        initMotorSettings(launcherIntakeMotor);
    }

    public void initGrabberMotors()
    {
        grabberLifterMotor = robot.hardwareMap.get(DcMotorEx.class, "motor" + motorConfigSettings.grabberLifterMotorNum);
        grabberLeftServo = robot.hardwareMap.servo.get("servo" + motorConfigSettings.grabberLeftServoNum);
        grabberRightServo = robot.hardwareMap.servo.get("servo" + motorConfigSettings.grabberRightServoNum);
        grabberServos = Arrays.asList(grabberLeftServo, grabberRightServo);

        if(motorConfigSettings.flipGrabberMotorDir[0]) grabberLifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(motorConfigSettings.flipGrabberMotorDir[1]) grabberServos.get(0).setDirection(Servo.Direction.REVERSE);
        if(motorConfigSettings.flipGrabberMotorDir[2]) grabberServos.get(1).setDirection(Servo.Direction.REVERSE);

        grabberLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        grabberLifterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.hardware.grabberLifterMotor.setTargetPosition(0);
        robot.hardware.grabberLifterMotor.setPower(grabberSettings.motorPower);
        robot.hardware.grabberLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for(int i = 0; i < 2; i++)
        {
            setServoPositions[i] = grabberSettings.servoRestPositions[i];
            robot.hardware.grabberServos.get(i).setPosition(setServoPositions[i]);
        }
        robot.hardware.grabberLimitSwitch = robot.hardwareMap.get(DigitalChannel.class, grabberSettings.limitSwitchName);
        robot.hardware.grabberLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void initMotorSettings(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor:motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void initMotorSettings(DcMotorEx motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetMotorEncodersList(List<DcMotorEx> motors)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    ///////////////////
    //set motor modes//
    ///////////////////
    public void setMotorsToCoastList(List<DcMotorEx> motors)
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
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setMotorsToSeparatePositionsAndPowersList(List<DcMotorEx> motors, int[] ticks, double[] power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(ticks[i]);
            motor.setPower(power[i]);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public void moveMotorsForwardList(List<DcMotorEx> motors, int ticks, double power)
    {
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks);
            motor.setPower(power);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void moveMotorForwardSeparateAmountList(List<DcMotorEx> motors, int[] ticks, double power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    public void moveMotorForwardSeparateAmountsAndPowersList(List<DcMotorEx> motors, int[] ticks, double[] power)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setTargetPosition(motor.getCurrentPosition() + ticks[i]);
            motor.setPower(power[i]);
            if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    public int[] getMotorSetPositions(List<DcMotorEx> motors)
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

    ////////////////////
    //motor velocities//
    ////////////////////
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

    void setMotorVelocitiesList(List<DcMotorEx> motors, double[] velocities)
    {
        int i = 0;
        for(DcMotorEx motor: motors)
        {
            motor.setVelocity(velocities[i]);
            i++;
        }
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

    public boolean motorPositionsInToleranceList(List<DcMotorEx> motors, int tolerance)
    {
        for (DcMotorEx m:motors){if(Math.abs(m.getTargetPosition()-m.getCurrentPosition()) > tolerance) return false;}
        return true;
    }
}

class MotorConfigSettings
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
    protected boolean[] flipLauncherMotorDir = {true, true, false};
    protected String launcherWheelMotorNum = "0B";
    public static PIDFCoefficients launcherWheelMotorPID = new PIDFCoefficients(10,3,0,0);
    protected String launcherIntakeMotorNum = "1B";
    protected String launcherServoNum = "0B";
    //grabber motors
    protected boolean[] flipGrabberMotorDir = {true, false, false};
    protected String grabberLifterMotorNum = "2B";
    protected String grabberLeftServoNum = "1B";
    protected String grabberRightServoNum = "2B";

    MotorConfigSettings(){}
}
