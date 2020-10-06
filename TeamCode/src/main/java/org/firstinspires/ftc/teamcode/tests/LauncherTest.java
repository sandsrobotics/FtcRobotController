package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "test launcher v1.1")
public class LauncherTest extends LinearOpMode{
    //////////////////
    //user variables//
    //////////////////
    //pin num
    protected int launcherWheelMotorNum = 0;
    protected int launcherLifterMotorNum = 0;
    protected int launcherservoNum = 0;
    //flip
    protected boolean fliplauncherWheelMotor = false;
    protected boolean fliplauncherLifterMotor = false;
    protected boolean fliplauncherservo = false;
    //servo
    protected int servoRestAngle = 0;
    protected int servoLaunchAngle = 90;
    //wheel
    protected double gearRatio = 5;
    protected double ticksPerRev = 0;
    protected double rpmIncrements = 6;
    protected double maxRpm = 6000;
    protected boolean useEncoder = true;
    //lifter
    protected double ticksPerDegree = 1;
    protected double maxAngle = 90;
    protected double rotationIncrements;
    protected boolean resetLifterDuringStart = false;

    /////////
    //other//
    /////////
    protected double setLifterAngle = 0;
    protected double setWheelRpm = 0;
    protected Boolean runWheelOnTrigger = true;

    DcMotorEx launcherWheelMotor;
    DcMotor launcherLifterMotor;
    Servo launcherServo;

    @Override
    public void runOpMode()
    {
        initStuff();

        while(opModeIsActive())
        {
            setLauncherSevo();
            setLauncherWheelMotor();
            setLauncherLifterMotor();
            getInputs();
        }
    }

    void initStuff()
    {
        //initiate
        launcherWheelMotor = hardwareMap.get(DcMotorEx.class,"motor" + launcherWheelMotorNum);
        launcherLifterMotor = hardwareMap.dcMotor.get("motor" + launcherLifterMotorNum);
        launcherServo = hardwareMap.servo.get("servo" + launcherservoNum);

        // reverse
        if(fliplauncherWheelMotor) launcherWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(fliplauncherLifterMotor) launcherWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        if(fliplauncherservo) launcherServo.setDirection(Servo.Direction.REVERSE);

        // zero motors
        launcherWheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherLifterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherServo.setPosition(servoRestAngle);
        if(resetLifterDuringStart)resetLifter();

        //set motor modes
        if(useEncoder) launcherWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else launcherWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherLifterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void resetLifter()
    {
        // later
    }

    void setLauncherSevo()
    {
        if(gamepad1.right_bumper) launcherServo.setPosition(servoLaunchAngle);
        else launcherServo.setPosition(servoRestAngle);
    }

    void setLauncherWheelMotor()
    {
        if(runWheelOnTrigger)launcherWheelMotor.setPower(gamepad1.left_trigger);
        else launcherWheelMotor.setVelocity(setWheelRpm * ticksPerRev * 60 / gearRatio);
    }

    void setLauncherLifterMotor()
    {
        launcherLifterMotor.setTargetPosition((int)(setLifterAngle * ticksPerDegree));
        launcherLifterMotor.setPower(.3);
        launcherLifterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void getInputs()
    {
        if(gamepad1.y) runWheelOnTrigger = !runWheelOnTrigger;

        if(gamepad1.b) setWheelRpm += rpmIncrements;
        else if(gamepad1.x) setWheelRpm -= rpmIncrements;
        if(setWheelRpm > maxRpm) setWheelRpm = maxRpm;

        if(gamepad1.dpad_right) setLifterAngle += rotationIncrements;
        else if(gamepad1.dpad_left) setLifterAngle -= rotationIncrements;
        if(setLifterAngle < 0) setLifterAngle = 0;
        else if(setLifterAngle > maxAngle) setLifterAngle = maxAngle;

    }
}
