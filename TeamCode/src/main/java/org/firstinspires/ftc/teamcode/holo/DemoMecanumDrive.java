package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Demo Mecanum Drive")
public class DemoMecanumDrive extends LinearOpMode {
    SandsRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        waitForStart();

        while(opModeIsActive()){
            robot.controlDrivetrain();
           //robot.sendTelemetry();
        }
        robot.stop();
    }
}

