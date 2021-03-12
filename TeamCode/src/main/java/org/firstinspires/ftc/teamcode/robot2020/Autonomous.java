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
    double[] basePos = {-16, -80, 0};

    double[][] APositions = {
        {-25,-66,-90},
        {-18,-72,-90}
    };
    double[][] BPositions = {
        {-1.1,-33.86,-90},
        {7,-47.5,-90}
    };
    double[][] CPositions ={
        {-16,-12,-90},
        {-16,-14,-90}
    };

    double[][] secondGoalPositions = {
        {-10, - 112, 0},
        {0, -112, 0}
    };

    double[] parkPos = {0,-53,-90};

    //settings
    // tol, time to stay in tol, max loops, max speed
    MoveToPosSettings finalPosSettings = new MoveToPosSettings(new double[]{1, 1, .5}, 10, 5000, 1);
    MoveToPosSettings losePosSettings = new MoveToPosSettings(new double[]{4, 4, 7.5}, 1, 5000, 1);

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

        /////////
        //start//
        /////////
        robot = new Robot(this, ru);

        robot.vision.tofdActivationSequence();
        robot.startTelemetry();

        while(!isStarted() && !isStopRequested())
        {
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

        if(isStopRequested()) return;

        robot.start(false);

        ////////////////
        //main program//
        ////////////////

        //move to base pos
        robot.movement.moveToPosition(basePos, losePosSettings);

        //setup for launch
        robot.grabber.setGrabberToPos((robot.grabber.grabberSettings.capturePos - 75), false);
        robot.launcher.setRPM(robot.launcher.launcherSettings.autoLaunchRPM);

        //launch power shot
        robot.launcher.autoLaunchPowerShots(1);

        //drop goal one
        goToDropZone(finalNumOfRings, 1);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, true);
        robot.movement.moveToPosition(robot.position.getPositionWithOffset(0,-7, 0), losePosSettings);

        //get ready and go to second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.movement.moveToPosition(secondGoalPositions[0], losePosSettings);
        robot.movement.moveToPosition(secondGoalPositions[1], finalPosSettings);

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, true);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 75, false);

        //drop off second goal
        goToDropZone(finalNumOfRings, 2);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, true);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, true);
        robot.movement.moveToPosition(robot.position.getPositionWithOffset(0,-9, 0), losePosSettings);

        //park
        robot.movement.moveToPosition(parkPos,finalPosSettings);
    }


    ///////////
    //methods//
    ///////////
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

    void goToDropZone(int pos, int goalNum)
    {
        if(pos == 0) { robot.movement.moveToPosition(APositions[goalNum - 1], finalPosSettings); }
        else if(pos == 1) { robot.movement.moveToPosition(BPositions[goalNum - 1], finalPosSettings); }
        else if(pos == 4) { robot.movement.moveToPosition(CPositions[goalNum - 1], finalPosSettings); }
    }

}

