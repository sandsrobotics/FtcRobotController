package org.firstinspires.ftc.teamcode.robot2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp(name = "test debug")
public class Test2 extends LinearOpMode
{
    Robot robot;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(false);
        ru.useDistanceSensors = true;
        ru.usePositionTracking = true;

        robot = new Robot(this, ru);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start(false);

        while(opModeIsActive())
        {
            float[] vals = robot.robotHardware.getDistancesList(robot.robotHardware.distSensors);
            robot.addTelemetry("dis 1:", vals[0]);
            robot.addTelemetry("dis 2:", vals[1]);
            robot.sendTelemetry();
        }
    }
}
