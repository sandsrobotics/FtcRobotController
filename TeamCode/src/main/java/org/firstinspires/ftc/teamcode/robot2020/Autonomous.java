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
        double[] APos = {-20,-50,-90};
        double[] BPosLose = {0,-65,0};
        double[] BPos = {0,-26,-90};
        double[] CPos = {-20,-2,-90};

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
               if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.NONE) robot.movement.moveToPosition(APos,new double[]{1,1,.5f},10,7000,.5);
               else if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.ONE)
               {
                   robot.movement.moveToPosition(BPosLose, new double[]{5,5,5},1,7000,.5);
                   robot.movement.moveToPosition(BPos,new double[]{1,1,.5},10,7000,.5);
               }
               else if(rings == Vision.SkystoneDeterminationPipeline.RingPosition.FOUR) robot.movement.moveToPosition(CPos,new double[]{1,1,.5f},10,7000,.5);
               robot.grabber.setGrabberToPos(robot.grabber.grabberSettings.capturePos);
               robot.grabber.setServosToPos(robot.grabber.grabberSettings.servoRestPositions);
            }
        }
    }
}

