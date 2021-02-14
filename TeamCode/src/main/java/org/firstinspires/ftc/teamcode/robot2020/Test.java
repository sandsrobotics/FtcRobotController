package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

// test
@Config
@TeleOp(name = "test dist")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.usePositionThread = true;
        ru.usePositionTracking = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(true);

        while (opModeIsActive())
        {
           robot.movement.moveForTeleOp(gamepad1, new GamepadButtonManager(GamepadButtons.A), false);
           /*
           if(robot.position.distances != null) {
               for (int i = 0; i < robot.robotHardware.distSensors.size(); i++) {
                   robot.addTelemetry("dis" + i, robot.position.distances[i]);
               }
           }

            */
           //robot.sendTelemetry();
        }
    }
}
