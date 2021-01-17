package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test auto v1")
public class Autonomous extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        GamepadButtons closeButton = GamepadButtons.A;
        boolean closed = false;
        Vision.SkystoneDeterminationPipeline.RingPosition rings = Vision.SkystoneDeterminationPipeline.RingPosition.NONE;

        double[] APos = {-16,-58,-90};
        //double[] APosLose = {-20,-124,0};
        double[] BPos = {2,-30,-90};
        //double[] BPosLose = {-20,-124,0};
        double[] CPos = {-16,-4,-90};
        //double[] CPosLose = {-20,-124,0};

        double[] secondGoalPos = {-4,-117,0};

        double[] tolFinal = {1, 1, 1};
        double[] tolLose = {2, 2, 5};

        int servoMoveTime = 150;

        RobotUsage ru = new RobotUsage();
        ru.useVuforia = false;
        ru.useComplexMovement = false;

        RobotSettingsMain rsm = new RobotSettingsMain();
        robot = new Robot(this, ru, rsm);

        while(!isStarted())
        {
            if(closeButton.getButtonPressed(gamepad1))
            {
                closed = !closed;
                if(closed) robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions);
                else robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions);
            }
            rings = robot.vision.pipeline.position;
        }

        robot.start();

        while (opModeIsActive())
        {
            robot.startTelemetry();
            robot.addTelemetry("rings", rings);
            robot.addTelemetry("posX", robot.position.currentRobotPosition[0]);
            robot.addTelemetry("posY", robot.position.currentRobotPosition[1]);
            robot.sendTelemetry();
            if(gamepad1.b)
            {
               if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.NONE)
               {
                   //robot.movement.moveToPosition(APosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(APos,tolFinal,15,7000,.25);
               }
               else if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.ONE)
               {
                   //robot.movement.moveToPosition(BPosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(BPos,tolFinal,15,7000,.25);
               }
               else if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR)
               {
                   //robot.movement.moveToPosition(CPosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(CPos,tolFinal,15,7000,.25);
               }

               robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions);
               robot.delay(servoMoveTime);
               robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos), false);
               robot.movement.moveToPosition(secondGoalPos, tolLose,1,7000,.25);
               robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions);
               robot.delay(servoMoveTime);

            }
        }
    }
}

