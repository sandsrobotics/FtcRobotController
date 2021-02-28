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
    double[] secondGoalPos = {-2, -120, 0};
    double[] parkPos = {0,-53,0};

    //tolerances
    double[] tolFinal = {1, 1, .5};
    double[] tolLose = {2, 2, 5};

    double maxSpeed = 1;

    //other
    int timesRingRecognitionReq = 10; //how many times does tfod have to see a certain number of rings to call it good

    ///////////////////
    //other variables//
    ///////////////////
    //claw
    boolean closed = false;

    //tfod
    int timesRingsRecognized = 0;
    int lastNumOfRings = -1;

    int calculatedNumOfRings;
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
        ru.useTensorFlowInTread = false;
        ru.useOpenCV = false;
        ru.useVuforiaInThread = false;

        robot = new Robot(this, ru);

        robot.vision.tofdActivationSequence();
        robot.startTelemetry();

        while(!isStarted() && !isStopRequested())
        {
            robot.addTelemetry("stop",!isStopRequested());
            if(closeButton.getButtonPressed(gamepad1))
            {
                closed = !closed;
                if(closed) robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, false);
                else robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);
            }

            calculatedNumOfRings = getNumOfRings();
            if(calculatedNumOfRings == -1) robot.addTelemetry("rings", " calculating...");
            else
            {
                finalNumOfRings = calculatedNumOfRings;
                robot.addTelemetry("rings", finalNumOfRings);
            }
            robot.sendTelemetry();
        }
        //if(true) return;
        robot.start(false);

        robot.launcher.setRPM(3500);

        goToDropZone(finalNumOfRings);

        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, false);

        robot.movement.moveToPosition(launchPos, tolFinal,15,7000,maxSpeed);

        robot.launcher.openGateServo();
        for(int i = 0; i < 4; i++)
        {
           robot.launcher.waitForRPMInTolerance(2500);
           robot.launcher.moveLaunchServo();
        }

        robot.movement.moveToPosition(secondGoalPos, tolFinal,1,7000,maxSpeed);

        goToDropZone(finalNumOfRings);

        robot.movement.moveToPosition(parkPos, tolFinal,1,7000,maxSpeed);
    }
    int getNumOfRings()
    {
        int currentNumOfRings = robot.vision.runTfodSequenceForRings();
        if(currentNumOfRings == lastNumOfRings)
        {
            timesRingsRecognized++;
            if(timesRingsRecognized >= timesRingRecognitionReq){ return currentNumOfRings;}
        }
        else
        {
            timesRingsRecognized = 0;
            lastNumOfRings = currentNumOfRings;
        }
        return -1;
    }
    void goToDropZone(int pos)
    {
        if(pos == 0)
        {
            //robot.movement.moveToPosition(APosLose, tolLose,1,7000,1);
            robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
            robot.movement.moveToPosition(APos,tolFinal,15,7000,maxSpeed);
        }
        else if(pos == 1)
        {
            //robot.movement.moveToPosition(BPosLose, tolLose,1,7000,1);
            robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
            robot.movement.moveToPosition(BPos,tolFinal,15,7000,maxSpeed);
        }
        else if(pos == 4)
        {
            //robot.movement.moveToPosition(CPosLose, tolLose,1,7000,1);
            robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
            robot.movement.moveToPosition(CPos,tolFinal,15,7000,maxSpeed);
        }
    }
}

