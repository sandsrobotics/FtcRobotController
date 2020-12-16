package org.firstinspires.ftc.teamcode.holo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.*;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

@Config
public class SandsRobot {
    // Robot constructor creates robot object and sets up all the actuators and sensors
    SandsRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        initTelemetry();
        initMotors();
        initImu();
        readSensors();
        //sendTelemetry();
    }

    private void initMotors() {
        motorLeftFront = hardwareMap.get(DcMotorEx.class, "motor0");
        motorRightFront = hardwareMap.get(DcMotorEx.class, "motor1");
        motorLeftRear = hardwareMap.get(DcMotorEx.class, "motor2");
        motorRightRear = hardwareMap.get(DcMotorEx.class, "motor3");
        motorLauncherWheel = hardwareMap.get(DcMotorEx.class, "motor0B");
        servo0 = hardwareMap.get(Servo.class, "servo0B");
        motorLiftArm = hardwareMap.get(DcMotorEx.class, "motor1B");
        servoClawLeft = hardwareMap.get(Servo.class, "servo1B");
        servoClawRight = hardwareMap.get(Servo.class, "servo2B");

        motors = Arrays.asList(motorLeftFront, motorLeftRear, motorRightRear, motorRightFront,motorLauncherWheel,motorLiftArm);

        motorLeftFront.setDirection(REVERSE);
        motorRightFront.setDirection(FORWARD);
        motorLeftRear.setDirection(REVERSE);
        motorRightRear.setDirection(FORWARD);
        motorLauncherWheel.setDirection(REVERSE);
        motorLiftArm.setDirection(FORWARD);

        servoClawLeft.setPosition(0.9);
        servoClawRight.setPosition(0.1);

        released = 0;

        for (DcMotorEx motor : motors) {
            motor.setMode(STOP_AND_RESET_ENCODER);
            motor.setMode(RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(BRAKE);
        }

        motorLauncherWheel.setZeroPowerBehavior(FLOAT);
    }

    // Initialize the imu within the expansion hub
    private void initImu() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParameters;
        // Create new IMU Parameters object.
        imuParameters = new BNO055IMU.Parameters();
        // Use degrees as angle unit.
        imuParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled = false;
        // Initialize IMU.
        imu.initialize(imuParameters);
    }

    protected void setPIDFCoefficients() {
        // change coefficients using methods included with DcMotorEx class.
        motorLauncherWheel.setVelocityPIDFCoefficients(P, I, D, F);
    }

    protected void getPIDFCoefficients() {
        PIDFCoefficients pidf = motorLauncherWheel.getPIDFCoefficients(RUN_USING_ENCODER);
        P = pidf.p;
        I = pidf.i;
        D = pidf.d;
        F = pidf.f;
    }

    private void initTelemetry() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    /** **********************************************************
     * Reads and sets sensor values on robot
     **********************************************************/
    void readSensors() {

    }

    // Sends messages and values to bottom of driver's screen
    void sendTelemetry() {
        Integer motorNum = 0;
        for (DcMotorEx motor : motors) {
            telemetry.addData(motorNum.toString(),motor.getPower());
            motorNum++;
        }
        telemetry.update();
    }

    public void stop() {
        //Stop the robot
        setPowerAll(0, 0, 0, 0);
    }

    public double powerToVelocity(double speed) {
        return 0.0;
    }

    // not yet finished
    protected void setVelocityAll(double rf, double rb, double lf, double lb){
        motorRightFront.setVelocity(rf);
        motorRightRear.setVelocity(rb);
        motorLeftFront.setVelocity(lf);
        motorLeftRear.setVelocity(lb);
    }

    protected void setPowerAll(double rf, double rb, double lf, double lb){
        motorRightFront.setPower(rf);
        motorRightRear.setPower(rb);
        motorLeftFront.setPower(lf);
        motorLeftRear.setPower(lb);
    }

    protected void controlDrivetrain() {
        double controlDrive = -gamepad1.left_stick_y;
        double controlStrafe = gamepad1.left_stick_x;
        double controlRotate = gamepad1.right_stick_x;
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower, driveMaxPower;

        //setting power levels based on drive controls
        leftFrontPower = controlDrive + controlStrafe + controlRotate;
        rightFrontPower = controlDrive - controlStrafe - controlRotate;
        leftRearPower = controlDrive - controlStrafe + controlRotate;
        rightRearPower = controlDrive + controlStrafe - controlRotate;

        //find max and normalize
        driveMaxPower = Math.max(Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)), Math.max(Math.abs(leftRearPower), Math.abs(rightRearPower)));
        driveMaxPower = Math.max(driveMaxPower, 1);
        leftFrontPower /= driveMaxPower;
        rightFrontPower /= driveMaxPower;
        leftRearPower /= driveMaxPower;
        rightRearPower /= driveMaxPower;

        //limit motors to max speed
        leftFrontPower *= driveSpeedLimit;
        rightFrontPower *= driveSpeedLimit;
        leftRearPower *= driveSpeedLimit;
        rightRearPower *= driveSpeedLimit;

        //set motor powers
        setPowerAll(rightFrontPower,rightRearPower,leftFrontPower,leftRearPower);
    }

    /**
     * Gets the orientation of the robot using the REV IMU
     * @return the angle of the robot
     */
    protected double getZAngle(){
        return (-imu.getAngularOrientation().firstAngle);
    }

    // Variable Definitions for Robot
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected TelemetryPacket packet;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    protected Thread positionThread;
    protected DcMotorEx motorLeftFront, motorLeftRear, motorRightRear, motorRightFront, motorLauncherWheel, motorLiftArm;
    protected Servo servo0, servoClawLeft, servoClawRight;
    //protected DcMotorEx verticalLeft, verticalRight, horizontal;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private double driveSpeedLimit = 1.0;
    public static double P;
    public static double I;
    public static double D;
    public static double F;
    double released; // Claw released

}
