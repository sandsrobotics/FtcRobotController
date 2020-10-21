package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Demo Mecanum Drive")
public class DemoMecanumDrive extends LinearOpMode {
    SandsRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry);
        waitForStart();

        while(opModeIsActive()){
            double leftFront = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double rightFront = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double leftBack = gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;
            double rightBack = -gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x;

            robot.setPowerAll(rightFront,rightBack,leftFront,leftBack );
        }
        robot.stop();
    }
}

