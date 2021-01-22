package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test auto v1")
public class Autonomous extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        //////////////////
        //user variables//
        //////////////////
        //buttons
        GamepadButtons closeButton = GamepadButtons.A;

        //positions
        double[] APos = {-16,-58,-90};
        //double[] APosLose = {-20,-124,0};
        double[] BPos = {2,-30,-90};
        //double[] BPosLose = {-20,-124,0};
        double[] CPos = {-16,-4,-90};
        //double[] CPosLose = {-20,-124,0};
        double[] GoalPos = {-4,-117,0};

        //tolerances
        double[] tolFinal = {1, 1, 1};
        double[] tolLose = {2, 2, 5};


        /////////////
        //variables//
        /////////////
        int servoMoveTime = 150;
        int rings = 0;
        boolean closed = false;


        ////////
        //code//
        ////////
        RobotUsage ru = new RobotUsage();
        ru.useComplexMovement = false;

        robot = new Robot(this, ru);

        robot.vision.todActivationSequence();
        robot.vision.startDashboardCameraStream(24,false);

        while(!isStarted())
        {
            if(closeButton.getButtonPressed(gamepad1))
            {
                closed = !closed;
                if(closed) robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions);
                else robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions);
            }
            robot.vision.findAllTfodObjects();
            robot.vision.getHighestConfidence();
            rings = robot.vision.getNumOfRings(robot.vision.getHighestConfidence());
        }

        robot.vision.deactivateTfod();
        robot.vision.stopDashboardCameraStream();
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
               if(rings == 0)
               {
                   //robot.movement.moveToPosition(APosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(APos,tolFinal,15,7000,.25);
               }
               else if(rings == 1)
               {
                   //robot.movement.moveToPosition(BPosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(BPos,tolFinal,15,7000,.25);
               }
               else if(rings == 4)
               {
                   //robot.movement.moveToPosition(CPosLose, tolLose,1,7000,1);
                   robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
                   robot.movement.moveToPosition(CPos,tolFinal,15,7000,.25);
               }
               else return;

               robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions);
               robot.delay(servoMoveTime);
               robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos), false);
               robot.movement.moveToPosition(GoalPos, tolLose,1,7000,.25);
               robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions);
               robot.delay(servoMoveTime);

            }
        }
    }
}

