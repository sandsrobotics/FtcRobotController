package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// test
@Config
@TeleOp(name = "test position tracking")
public class Test extends LinearOpMode {

    Robot robot;
    GamepadButtonManager brake = new GamepadButtonManager(GamepadButtons.A);
    GamepadButtonManager resetAngle = new GamepadButtonManager(GamepadButtons.dpadUP);

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.usePositionThread = true;
        ru.usePositionTracking = true;
        ru.useDistanceSensors = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true);

        while (opModeIsActive())
        {
           robot.movement.moveForTeleOp(gamepad1, brake, false);
           robot.addTelemetry("X", robot.position.currentRobotPosition[0]);
           float[] vals = robot.robotHardware.getDistancesAfterMeasure(robot.robotHardware.distSensors);
           robot.addTelemetry("dis 1", vals[0]);
           robot.addTelemetry("Y", robot.position.currentRobotPosition[1]);
           robot.addTelemetry("dis 2", vals[1]);
           robot.addTelemetry("rot" , robot.position.currentRobotPosition[2]);
           if(resetAngle.getButtonPressed(gamepad1)) robot.position.rotationOffset += robot.position.currentRotation;
           robot.sendTelemetry();
        }
    }
}
