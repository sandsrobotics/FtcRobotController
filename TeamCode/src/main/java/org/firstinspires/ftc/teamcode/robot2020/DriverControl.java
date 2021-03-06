package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp(name = "driver control v2")
public class DriverControl extends LinearOpMode
{
    Robot robot;
    GamepadButtonManager autoLaunchButton;
    GamepadButtonManager breakButton = new GamepadButtonManager(GamepadButtons.leftJoyStickBUTTON);
    public static PIDFCoefficients PIDF = new PIDFCoefficients(10,3,0,0);
    public PIDFCoefficients lastPIDF = new PIDFCoefficients(0,0,0,0);

    short mode = 0;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.useVuforia = false;
        ru.useComplexMovement = false;
        ru.useTensorFlow = false;
        ru.useDistanceSensors = false;

        robot = new Robot(this,ru);

        waitForStart();

        autoLaunchButton = new GamepadButtonManager(gamepad1, GamepadButtons.dpadUP);
        robot.start(true);

        while (opModeIsActive())
        {
            if(mode == 0)
            {
                if(lastPIDF.p != PIDF.p || lastPIDF.i != PIDF.i || lastPIDF.d != PIDF.d || lastPIDF.f != PIDF.f)
                {
                    robot.robotHardware.launcherWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
                    robot.robotHardware.launcherWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lastPIDF = PIDF;
                }
                robot.movement.moveForTeleOp(gamepad1, breakButton, true);
                robot.grabber.runForTeleOp(gamepad1, true);
                robot.launcher.runForTeleOp(gamepad2,true);
                if(autoLaunchButton.getButtonHeld()) mode = 1;
                robot.sendTelemetry();
            }
            else if(mode == 1)
            {
                robot.launcher.autoLaunchDiskFromLine();
                mode = 0;
            }
        }
    }
}
