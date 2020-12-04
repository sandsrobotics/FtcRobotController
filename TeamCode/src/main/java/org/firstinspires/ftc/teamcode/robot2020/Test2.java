package org.firstinspires.ftc.teamcode.robot2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

@TeleOp(name = "test vision")
public class Test2 extends LinearOpMode
{

    Robot robot;

    @Override
    public void runOpMode()
    {
        robot = new Robot(this,true, true,false, false, false, true, false);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start();
        robot.vision.startDashboardCameraStream(24);
        robot.movement.setSpeedMultiplier(.75);

        while(opModeIsActive())
        {
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.X);
            robot.startTelemetry();
            if(robot.vision.anyTrackableFound)
            {
                for (int i = 0; i < 5; i++)
                {
                    if (robot.vision.currentTrackablesLocations[i] != null)
                    {
                        OpenGLMatrix m = robot.vision.currentTrackablesLocations[i];
                        robot.addTelemetry(robot.vision.trackables.get(i).getName() + " found at: ", m.getTranslation());
                        robot.addTelemetry("trackable rotation: ", robot.vision.getTrackableAngles(m));
                    }
                }
                if(robot.vision.currentCalculatedRobotLocation != null)
                {
                    robot.addTelemetry("calculated robot position: ", robot.vision.currentCalculatedRobotLocation.getTranslation());
                    robot.addTelemetry("robot rotation: ", robot.vision.getTrackableAngles(robot.vision.currentCalculatedRobotLocation));
                }
            }
            else robot.addTelemetry("no trackables found", ":(");
            robot.sendTelemetry();
        }
    }
}
