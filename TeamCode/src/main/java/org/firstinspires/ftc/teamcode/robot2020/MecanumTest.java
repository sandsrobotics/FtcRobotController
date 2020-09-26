package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "test2020")
public class MecanumTest extends LinearOpMode
{
    Robot robot;

    // user variables
    public static double targetAngle = 45;
    public static double tolerance = 1;

    @Override
    public void runOpMode()
    {
       robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);

       waitForStart();

       robot.startTelemetry();
       robot.movement.moveForwardInches(7,.5);
       robot.movement.turnToAngleSimple(targetAngle, tolerance, 50, 1000);
       while(opModeIsActive())
       {

           robot.movement.moveForTeleOp(gamepad1);
           robot.updateTelemetry();
           robot.sendTelemetry();
       }
    }

}
