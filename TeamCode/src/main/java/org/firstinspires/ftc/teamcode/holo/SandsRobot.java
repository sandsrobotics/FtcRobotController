package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class SandsRobot {
    // Robot constructor creates robot object and sets up all the actuators and sensors
    SandsRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        leftFront = hardwareMap.get(DcMotorEx.class, "motor0");
        rightFront = hardwareMap.get(DcMotorEx.class, "motor1");
        leftRear = hardwareMap.get(DcMotorEx.class, "motor2");
        rightRear = hardwareMap.get(DcMotorEx.class, "motor3");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // initialize hardware and data
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        readSensors();
        sendTelemetry();
        initImu();
    }

    // Sends messages and values to bottom of driver's screen
    void sendTelemetry() {
        telemetry.update();
    }

    /** **********************************************************
     * Reads and sets sensor values on robot
     **********************************************************/
    void readSensors() {

    }

    public void stop() {
        //Stop the robot
        setPowerAll(0, 0, 0, 0);
    }

    public double powerToVelocity(double speed) {
        return 0.0;
    }

    protected void setVelocityAll(double rf, double rb, double lf, double lb){

        rightFront.setVelocity(rf);
        rightRear.setVelocity(rb);
        leftFront.setVelocity(lf);
        leftRear.setVelocity(lb);
    }

    protected void setPowerAll(double rf, double rb, double lf, double lb){
        rightFront.setPower(rf);
        rightRear.setPower(rb);
        leftFront.setPower(lf);
        leftRear.setPower(lb);
    }

    // Initialize the imu within the expansion hub
    private void initImu() {
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
    protected Thread positionThread;
    protected DcMotorEx leftFront, leftRear, rightRear, rightFront;
    protected DcMotorEx verticalLeft, verticalRight, horizontal;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    protected double MOTORMAX = 1;
    protected double MOTORMIN = -1;
}
