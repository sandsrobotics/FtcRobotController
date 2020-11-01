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
    public static double proportional = .02;

    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,false, true, false, false, false);
        robot.motorConfig.setMotorsToCoastList(robot.motorConfig.driveMotors);

        robot.startTelemetry();
        robot.complexMovement.startRecording();
        robot.addTelemetryString("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.startTelemetry();
        long start = System.currentTimeMillis();
        while(robot.complexMovement.isRecording && !robot.stop())
        {
            robot.complexMovement.recorder(true);
        }
        long end = System.currentTimeMillis();
        long mills =  end - start;
        robot.addTelemetryDouble("start time: ", start);
        robot.addTelemetryDouble("end time: ", end);
        robot.addTelemetryDouble("mills time: ", mills);
        robot.sendTelemetry();

        robot.startTelemetry();
        robot.complexMovement.stopRecording(false, "test");
        robot.complexMovement.loadMoveDB("test");
        robot.addTelemetryString("Robot: ", "ready to move");
        robot.sendTelemetry();
        while(!robot.stop() && opModeIsActive())
        {
            if(gamepad1.a)
            {
                robot.startTelemetry();
                robot.complexMovement.runLoadedMoveV2(1,false, proportional);
                robot.addTelemetryString("Robot: ", "done with move");
                robot.sendTelemetry();
                sleep(10000);
            }
            if(gamepad1.b && gamepad1.x)
            {
                robot.startTelemetry();
                robot.addTelemetryString("database clear activated: ", "hold B for 1 second to clear");
                robot.sendTelemetry();
                sleep(1000);
                if(gamepad1.b)
                {
                    robot.complexMovement.clearDatabase();
                    robot.startTelemetry();
                    robot.addTelemetryString("database cleared", "");
                    robot.sendTelemetry();
                }
                else
                {
                    robot.startTelemetry();
                    robot.addTelemetryString("database clear deactivated: ", "");
                    robot.sendTelemetry();
                }
            }
        }
    }
}
