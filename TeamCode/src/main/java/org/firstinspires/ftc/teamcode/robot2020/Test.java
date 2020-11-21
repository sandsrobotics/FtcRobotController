package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test vision v1.1")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,false, false, false, false, true, false);

        waitForStart();

        robot.startTelemetry();
        robot.position.start();
        robot.vision.start();
        robot.vision.startDashboardCameraStream(24);

        while(opModeIsActive())
        {
            for(int i = 1; i < 6; i++)
            {
                if(robot.vision.currentTrackablesLocations[i-1] != null) robot.addTelemetry("target " + i + " pos: ", robot.vision.currentTrackablesLocations[i - 1].getTranslation());
                else robot.addTelemetry("target " + i + " pos: ", new double[]{0,0,0});
                robot.addTelemetry("target " + i + " rot: ", robot.vision.getTrackableAngles(robot.vision.currentTrackablesLocations[i - 1]));
            }
            if(robot.vision.currentCalculatedRobotLocation != null)robot.addTelemetry("robot pos: ", robot.vision.currentCalculatedRobotLocation.getTranslation());
            else robot.addTelemetry("robot pos: ", new double[]{0,0,0});
            robot.addTelemetry("robot rot: ", robot.vision.getTrackableAngles(robot.vision.currentCalculatedRobotLocation));

            robot.sendTelemetry();
        }
    }
}
