package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test complex movement")
public class Test2 extends LinearOpMode
{

    Robot robot;
    public static double proportional = .08;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,false, true, false, false, false);
        robot.motorConfig.setMotorsToCoastList(robot.motorConfig.driveMotors);

        robot.startTelemetry();
        robot.addTelemetryString("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        while(!robot.stop() && opModeIsActive())
        {
            robot.startTelemetry();

            if(robot.complexMovement.isRecording)
            {
                robot.addTelemetryString("Robot: ", "recording");
                robot.sendTelemetry();
                while (robot.complexMovement.isRecording && !robot.stop()) { robot.complexMovement.recorder(true); }
                robot.addTelemetryString("Robot: ", "done recording");
                robot.sendTelemetry();
                robot.complexMovement.stopRecording(true, "test");
                robot.addTelemetryString("Robot: ", "ready to move");
                robot.sendTelemetry();
            }

            if(gamepad1.a)
            {
                robot.startTelemetry();
                robot.complexMovement.loadMoveDB("test");
                robot.complexMovement.runLoadedMoveV2(1,false, proportional);
                robot.addTelemetryString("Robot: ", "done with move");
                robot.sendTelemetry();
            }
            else if(gamepad1.b && gamepad1.x)
            {
                robot.startTelemetry();
                robot.addTelemetryString("database clear activated: ", "hold B for 2 second to clear");
                robot.sendTelemetry();
                sleep(2000);
                if(gamepad1.b)
                {
                    robot.complexMovement.clearDatabase();
                    robot.startTelemetry();
                    robot.addTelemetryString("database cleared", "");
                    robot.sendTelemetry();
                    sleep(1000);
                }
                else
                {
                    robot.startTelemetry();
                    robot.addTelemetryString("database clear deactivated: ", "");
                    robot.sendTelemetry();
                }
            }
            else if(gamepad1.y) robot.complexMovement.startRecording();
        }
    }
}
