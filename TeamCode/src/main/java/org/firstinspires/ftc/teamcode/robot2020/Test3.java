package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test second goal pickup")
public class Test3 extends LinearOpMode {

    Robot robot;

    double[] basePos = {-20, -80, 0};

    double[][][] secondGoalPositions = {
            {{-10, - 106, 0}, {-2, -112, 0}},
            {{30, -100, 180},{22, -108.5, 180}}
    };

    int straitUpPos = 500;

    @Override
    public void runOpMode()
    {

        RobotUsage ru = new RobotUsage();
        ru.useComplexMovement = false;
        ru.useTensorFlowInTread = false;
        ru.useOpenCV = false;
        ru.useVuforiaInThread = false;

        robot = new Robot(this, ru);

        waitForStart();

        robot.start(false);

        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.robotUsage.useDistanceSensors = true;
        robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.losePosSettings);
        robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.finalPosSettings);

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.delay(2000);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, true);

        //drop off second goal
        robot.grabber.setGrabberToPos(straitUpPos, false);

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1, true);
        }
    }
}