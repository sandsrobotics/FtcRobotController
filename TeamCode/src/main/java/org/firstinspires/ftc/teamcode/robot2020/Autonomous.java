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
    double[] basePos = {-20, -80, 0};

    double[][] APositions = {
        {-25,-62,-90},
        {-18,-68,-90}
    };
    double[][] BPositions = {
        {-1,-38,-90},
        {6,-48,-90}
    };
    double[][] CPositions ={
        {-25,-14,-90},
        {-18,-20,-90}
    };

    double[][][] secondGoalPositions = {
        {{-10, - 106, 0}, {-2, -112, 0}},
        {{30, -100, 180},{22, -107.5, 180}}
    };

    double[] parkPos = {0,-53,-90};

    //other
    int timesRingRecognitionReq = 10; //how many times does tfod have to see a certain number of rings to call it good

    ///////////////////
    //other variables//
    ///////////////////
    //grabber
    boolean closed = true;
    int straitUpPos = 500;

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
        if(finalNumOfRings > 0) robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);

        //setup for launch
        robot.grabber.setGrabberToPos(straitUpPos, false);

        //launch power shot
        robot.launcher.autoLaunchPowerShotsV2();
        robot.launcher.setRPM(0);

        //drop goal one
        robot.robotUsage.useDistanceSensors = false;
        goToDropZone(finalNumOfRings, 1);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, true);
        robot.movement.moveToPosition(robot.position.getPositionWithOffset(0,-7, 0), robot.movement.movementSettings.losePosSettings);

        //get ready and go to second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);
        robot.robotUsage.useDistanceSensors = true;
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        if(finalNumOfRings != 1){
            robot.movement.moveToPosition(secondGoalPositions[0][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[0][1], robot.movement.movementSettings.finalPosSettings);
        }
        else{
            robot.movement.moveToPosition(secondGoalPositions[1][0], robot.movement.movementSettings.losePosSettings);
            robot.movement.moveToPosition(secondGoalPositions[1][1], robot.movement.movementSettings.finalPosSettings);
        }

        //grab second goal
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos, true);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoGrabPositions, true);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 70, false);

        //drop off second goal
        if(finalNumOfRings == 4){
            robot.movement.moveToPosition(basePos, robot.movement.movementSettings.losePosSettings);
        }
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos - 70, false);
        goToDropZone(finalNumOfRings, 2);
        robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions, true);
        robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.restPos, false);

        robot.robotUsage.useDistanceSensors = false;

        robot.movement.moveToPosition(robot.position.getPositionWithOffset(3,-7, 0), robot.movement.movementSettings.losePosSettings);

        //park
        robot.movement.moveToPosition(parkPos,robot.movement.movementSettings.finalPosSettings);
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
        if(pos == 0) { robot.movement.moveToPosition(APositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 1) { robot.movement.moveToPosition(BPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
        else if(pos == 4) { robot.movement.moveToPosition(CPositions[goalNum - 1], robot.movement.movementSettings.finalPosSettings); }
    }

}

