package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test launcher v1.1")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,true, true,false, false, true,false, false, false);

        waitForStart();

        robot.start();

        while(opModeIsActive())
        {
            /*
            robot.addTelemetry("pidf", robot.motorConfig.launcherWheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            robot.launcher.opModeRun(true);
            if(GamepadButtons.dpadUP.getButtonHeld(gamepad1, 1000)) robot.motorConfig.launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MotorConfig.launcherMotorPID);
            robot.sendTelemetry();
            */

            if(gamepad1.a) robot.launcher.goToShootingPos();
            robot.addTelemetry("point to goal ang: ", robot.launcher.getAngleToPointToPosition());
            robot.addTelemetry("x: ", robot.position.currentRobotPosition[0]);
            robot.addTelemetry("y: ", robot.position.currentRobotPosition[1]);
            robot.addTelemetry("rot: ", robot.position.currentRobotPosition[2]);
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.X);
            robot.sendTelemetry();
        }

    }
}
