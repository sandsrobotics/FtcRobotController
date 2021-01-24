package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// test
@Config
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "test auto v1")
public class Autonomous extends LinearOpMode {

    //////////////////
    //user variables//
    //////////////////
    //buttons
    GamepadButtonManager closeButton = new GamepadButtonManager(GamepadButtons.A);

    //positions
    double[] APos = {-16,-58,-90};
    //double[] APosLose = {-20,-124,0};
    double[] BPos = {2,-30,-90};
    //double[] BPosLose = {-20,-124,0};
    double[] CPos = {-16,-4,-90};
    //double[] CPosLose = {-20,-124,0};

    double[] launchPos = {0, -65, 4};
    double[] parkPos = {0,-53,0};

    //tolerances
    double[] tolFinal = {1, 1, .5};
    double[] tolLose = {2, 2, 5};

    //other
    int servoMoveTime = 150;
    int timesRingRecognitionReq = 10; //how many times does tfod have to see a certain number of rings to call it good

    ///////////////////
    //other variables//
    ///////////////////
    //claw
    boolean closed = false;

    //tfod
    int timesRingsRecognized = 0;
    int lastNumOfRings = -1;
    int finalNumOfRings = -1; //what is the final say on the number of rings

    //other
    Robot robot;


    ////////
    //code//
    ////////
    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.useComplexMovement = false;
        ru.useVuforiaInThread = false;
        ru.useTensorFlowInTread = false;
        ru.useOpenCV = false;

        robot = new Robot(this, ru);

        while(!isStarted())
        {
            if(closeButton.getButtonPressed(gamepad1))
            {
                closed = !closed;
                if(closed) robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, false);
                else robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
            }
            finalNumOfRings = getNumOfRings();
            if(finalNumOfRings == -1) robot.addTelemetry("rings", " calculating...");
            else robot.addTelemetry("rings", finalNumOfRings);
        }

        robot.start();

        robot.launcher.setRPM(3700);
           if(finalNumOfRings == 0)
           {
               //robot.movement.moveToPosition(APosLose, tolLose,1,7000,1);
               robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
               robot.movement.moveToPosition(APos,tolFinal,15,7000,.5);
           }
           else if(finalNumOfRings == 1)
           {
               //robot.movement.moveToPosition(BPosLose, tolLose,1,7000,1);
               robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
               robot.movement.moveToPosition(BPos,tolFinal,15,7000,.5);
           }
           else if(finalNumOfRings == 4)
           {
               //robot.movement.moveToPosition(CPosLose, tolLose,1,7000,1);
               robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
               robot.movement.moveToPosition(CPos,tolFinal,15,7000,.5);
           }

           robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);

           robot.movement.moveToPosition(launchPos, tolFinal,15,7000,.5);
           for(int i = 0; i < 4; i++)
           {
           robot.launcher.waitForRPMInTolerance(2500);
           robot.launcher.moveLaunchServo();
           }

           robot.movement.moveToPosition(parkPos, tolFinal,1,7000,.5);
    }
    int getNumOfRings()
    {
        int currentNumOfRings = robot.vision.RunTodSequenceForRings();
        if(currentNumOfRings == lastNumOfRings)
        {
            timesRingsRecognized++;
            if(timesRingsRecognized == timesRingRecognitionReq){ return currentNumOfRings;}
        }
        else
        {
            timesRingsRecognized = 0;
            lastNumOfRings = currentNumOfRings;
        }
        return -1;
    }
}

