package org.firstinspires.ftc.teamcode.holo;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

public class SandsManipulators {
    // Robot constructor creates robot object and sets up all the actuators and sensors
    SandsManipulators(SandsRobot robot) {
        this.gamepad1 = robot.gamepad1;
        this.gamepad2 = robot.gamepad2;
        this.robot = robot;
        this.claw = new Claw(robot);
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

        if (gamepad1.right_bumper) {
            if (!rb_pressed) {
                rb_pressed = true;
                claw.swap();
            }
        } else rb_pressed = false;

        /*
        if (gamepad1.right_trigger == 0) {
            if (!rb_pressed) claw.open();
        } else {
            claw.close();
        }
        */
        robot.motorLiftArm.setPower(gamepad1.right_stick_y / -2);
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
    protected Claw claw;
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
    boolean b_pressed, x_pressed, y_pressed, dl_pressed, dr_pressed = false, rb_pressed = false;
    DcMotorEx motorLauncherWheel;
}

class Claw {
    SandsRobot robot;
    boolean is_closed;
    public Claw(SandsRobot robot) {
        this.robot = robot;
        open();
    }

    public void swap() {
        if (is_closed) open();
        else close();
    }
    public void close() {
        robot.servoClawLeft.setPosition(0.1);
        robot.servoClawRight.setPosition(0.9);
        is_closed = true;
    }

    public void open(){
        robot.servoClawLeft.setPosition(0.6);
        robot.servoClawRight.setPosition(0.2);
        is_closed=false;
    }



}