package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

// test
@Config
@TeleOp(name = "test vision")
public class Test2 extends LinearOpMode
{

    Robot robot;
    public static double proportional = .08;

    @Override
    public void runOpMode()
    {
        robot = new Robot(this,true, true, false, false, true, false);
        robot.motorConfig.setMotorsToCoastList(robot.motorConfig.driveMotors);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start();

        while(!robot.stop() && opModeIsActive())
        {
            robot.addTelemetry("img 1: ", robot.vision.currentTrackablesLocations[0]);
            robot.addTelemetry("img 2: ", robot.vision.currentTrackablesLocations[1]);
            robot.addTelemetry("img 3: ", robot.vision.currentTrackablesLocations[2]);
            robot.addTelemetry("img 4: ", robot.vision.currentTrackablesLocations[3]);
            robot.addTelemetry("img 5: ", robot.vision.lastTrackablesLocations[4]);
            robot.sendTelemetry();
        }
    }
}
