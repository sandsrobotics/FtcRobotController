package org.firstinspires.ftc.teamcode.robot2020;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "test debug")
public class Test2 extends LinearOpMode
{

    Robot robot;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.setAllToValue(true);
        ru.useVuforia = false;
        RobotSettingsMain rs = new RobotSettingsMain();
        rs.positionSettings.resetPos = false;
        robot = new Robot(this, ru);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start();

        robot.grabber.initGrabberPos();

        while(opModeIsActive())
        {
            robot.grabber.runForTeleop(gamepad1);
            robot.movement.moveForTeleOp(gamepad1,GamepadButtons.leftBUMPER);
            /*
            if(gamepad1.x)
            {
                robot.movement.moveToPosition(robot.position.getPositionWithOffset(10,0,0),new double[]{.1,.1,.1},100,7000,.3);
            }
            if(gamepad1.y)
            {
                robot.movement.moveToPosition(robot.position.getPositionWithOffset(0,10,0),new double[]{.1,.1,.1},100,7000,.3);
            }

             */
        }
    }
}
