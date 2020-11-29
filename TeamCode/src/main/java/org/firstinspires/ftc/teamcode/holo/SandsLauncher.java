package org.firstinspires.ftc.teamcode.holo;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    void launch() {
            robot.servo0.setPosition(launchAngle);
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

        if (gamepad1.right_trigger > 0) {
            launchAngle = servoLaunchAngle;
        } else {
            launchAngle = servoRestAngle;
        }
    }

    void data_out() {
        double RPM = robot.motorLauncherWheel.getVelocity() * spinMultiplier;
        robot.telemetry.addData("RPM", RPM);
        robot.telemetry.addData("Set RPM", setWheelRpm);
        robot.telemetry.update();
        robot.packet.put("RPM: ", RPM);
        robot.packet.put("SetRPM ", setWheelRpm);
        robot.dashboard.sendTelemetryPacket(robot.packet);
        Log.i("SANDS", Double.toString(setWheelRpm) +","+Double.toString(RPM));
    }

    protected SandsRobot robot;
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    //servo
    protected double servoRestAngle = .2;
    protected double servoLaunchAngle = .45;
    protected double launchAngle = servoRestAngle;
    //wheel
    protected double gearRatio = 1;
    //protected double ticksPerRev = 145.6; // for 1150 RPM motor 5.2:1
    protected double ticksPerRev = 28; // for die 6000 RPM motor 1:1
    protected double rpmIncrements = 100;
    protected double maxRpm = 6000;

    //lifter
    //protected double ticksPerDegree = 2.0 * 1120.0 / 360.0; //Andymark 40:1  2:1 chain gear ratio
    protected double maxAngle = 90;
    protected double rotationIncrements = 10;

    //other//
    protected double setLifterAngle = 0;
    protected double setWheelRpm = 2000;
    protected Boolean runWheelOnTrigger = true;
    Double spinMultiplier = 60 / ticksPerRev * gearRatio;
    boolean b_pressed, x_pressed, y_pressed, dl_pressed, dr_pressed = false;
    DcMotorEx motorLauncherWheel;
}
