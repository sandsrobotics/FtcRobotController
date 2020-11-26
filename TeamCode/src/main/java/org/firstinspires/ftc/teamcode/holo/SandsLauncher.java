package org.firstinspires.ftc.teamcode.holo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SandsLauncher {
    // Robot constructor creates robot object and sets up all the actuators and sensors
    SandsLauncher(SandsRobot robot) {
        this.gamepad1 = robot.gamepad1;
        this.gamepad2 = robot.gamepad2;
        this.robot = robot;
        this.motorLauncherWheel = robot.motorLauncherWheel;
    }

    void setLauncherWheelMotor() {
        if (runWheelOnTrigger) motorLauncherWheel.setPower(gamepad1.left_trigger);
        else motorLauncherWheel.setVelocity(setWheelRpm / spinMultiplier);
    }

    void getInputs() {
        if (gamepad1.y) {
            if (!y_pressed) {
                y_pressed = true;
                runWheelOnTrigger = !runWheelOnTrigger;
            }
        } else y_pressed = false;

        if (gamepad1.b) {
            if (!b_pressed) {
                setWheelRpm += rpmIncrements;
                b_pressed = true;
            }
        } else b_pressed = false;

        if (gamepad1.x) {
            if (!x_pressed) {
                x_pressed = true;
                setWheelRpm -= rpmIncrements;
            }
        } else x_pressed = false;

        if (setWheelRpm > maxRpm) setWheelRpm = maxRpm;
        if (setWheelRpm < 0) setWheelRpm = 0;

        if (gamepad1.dpad_right) {
            if (!dr_pressed) {
                dr_pressed = true;
                setLifterAngle += rotationIncrements;
            }
        } else dr_pressed = false;

        if (gamepad1.dpad_left) {
            if (!dl_pressed) {
                dl_pressed = true;
                setLifterAngle -= rotationIncrements;
            }
        } else dl_pressed = false;

        if (setLifterAngle < 0) setLifterAngle = 0;
        else if (setLifterAngle > maxAngle) setLifterAngle = maxAngle;
    }

    void data_out() {
        double RPM = robot.motorLauncherWheel.getVelocity() * spinMultiplier;
        robot.telemetry.addData("RPM", RPM);
        robot.telemetry.addData("Set RPM", setWheelRpm);
        // 0 encoder = ? Degrees
        //robot.telemetry.addData("Ramp Encoder", launcherLifterMotor.getCurrentPosition());
        //robot.telemetry.addData("Ramp Angle", setLifterAngle);
        robot.telemetry.update();
        //robot.packet.put("RPM: ", RPM);
        //dashboard.sendTelemetryPacket(packet);
    }

    protected SandsRobot robot;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    //servo
    protected double servoRestAngle = .34;
    protected double servoLaunchAngle = .46;
    //wheel
    protected double gearRatio = 1;
    protected double ticksPerRev = 145.6;
    protected double rpmIncrements = 100;
    protected double maxRpm = 6000;

    //lifter
    protected double ticksPerDegree = 2.0 * 1120.0 / 360.0; //Andymark 40:1  2:1 chain gear ratio
    protected double maxAngle = 90;
    protected double rotationIncrements = 10;

    //other//
    protected double setLifterAngle = 0;
    protected double setWheelRpm = 0;
    protected Boolean runWheelOnTrigger = true;
    protected FtcDashboard dashboard;
    Double spinMultiplier= 60 / ticksPerRev * gearRatio;
    Double spinVelocity;
    boolean b_pressed, x_pressed, y_pressed, dl_pressed, dr_pressed = false;
    DcMotorEx motorLauncherWheel;
    //set multiplier
}
