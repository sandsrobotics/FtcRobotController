package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test launcher v1.1")
public class Test extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,false, false, false, true, false, false);
        //robot.launcher.init();

        waitForStart();

        robot.startTelemetry();

        while(opModeIsActive())
        {
            robot.addTelemetry("pidf", robot.motorConfig.launcherWheelMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            robot.launcher.opModeRun(true);
            if(GamepadButtons.dpadUP.getButtonHeld(gamepad1, 1000)) robot.motorConfig.launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MotorConfig.launcherMotorPID);
            robot.sendTelemetry();
        }
    }
}
