package org.firstinspires.ftc.teamcode.robot2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "test debug")
public class Test2 extends LinearOpMode
{

    Robot robot;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.usePositionTracking = true;
        ru.usePositionThread = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start(false);

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1,new GamepadButtonManager(GamepadButtons.leftBUMPER), true);
            robot.addTelemetry("posX", robot.position.currentRobotPosition[0]);
            robot.addTelemetry("posY", robot.position.currentRobotPosition[1]);
            robot.addTelemetry("rot", robot.position.currentRobotPosition[2]);
            robot.sendTelemetry();
        }
    }
}
