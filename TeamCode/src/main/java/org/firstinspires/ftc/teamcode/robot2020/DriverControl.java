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
    GamepadButtonManager fullAutoLaunchButton;
    GamepadButtonManager semiAutoLaunchButton;
    GamepadButtonManager pointToZero;
    GamepadButtonManager RPMChange;
    GamepadButtonManager resetAngle;
    GamepadButtonManager wobbleGaolDrop;
    GamepadButtonManager autoLaunchPowerShot;

    short mode = 0;

    @Override
    public void runOpMode()
    {
        RobotUsage ru = new RobotUsage();
        ru.useVuforia = false;
        ru.useComplexMovement = false;
        ru.useTensorFlow = false;

        RobotSettingsMain rm = new RobotSettingsMain();
        if(!Position.updateRotFromFile(rm.positionSettings))rm.positionSettings.startRotation = -90;


        robot = new Robot(this, ru, rm);

        waitForStart();

        fullAutoLaunchButton = new GamepadButtonManager(gamepad2, GamepadButtons.dpadUP);
        semiAutoLaunchButton = new GamepadButtonManager(gamepad2, GamepadButtons.dpadLEFT);
        pointToZero = new GamepadButtonManager(gamepad2, GamepadButtons.dpadRIGHT);
        RPMChange = new GamepadButtonManager(gamepad2, GamepadButtons.leftBUMPER);
        resetAngle = new GamepadButtonManager(gamepad1, GamepadButtons.rightBUMPER);
        wobbleGaolDrop = new GamepadButtonManager(gamepad1, GamepadButtons.A);
        autoLaunchPowerShot = new GamepadButtonManager(gamepad1, GamepadButtons.X);

        robot.start(true);

        while (opModeIsActive())
        {
            if(mode == 0)
            {
                robot.movement.moveForTeleOp(gamepad1, true);
                robot.grabber.runForTeleOp(gamepad1, true);
                robot.launcher.runForTeleOp(gamepad2,true);

                if(fullAutoLaunchButton.getButtonHeld()) mode = 1;
                else if(semiAutoLaunchButton.getButtonHeld()) mode = 2;
                else if(pointToZero.getButtonHeld()) mode = 3;
                else if(wobbleGaolDrop.getButtonHeld()) mode = 4;
                else if(autoLaunchPowerShot.getButtonHeld()) mode = 5;

                if(RPMChange.getButtonPressed())
                {
                  if(robot.launcher.targetWheelRpm == robot.launcher.launcherSettings.autoLaunchRPM){robot.launcher.targetWheelRpm = robot.launcher.launcherSettings.powerShotRPM;}
                  else{robot.launcher.targetWheelRpm = robot.launcher.launcherSettings.autoLaunchRPM;}
                }
                if(resetAngle.getButtonPressed()) robot.position.resetAngle();

                robot.sendTelemetry();
            }
            else if(mode == 1) {
                robot.launcher.autoLaunchDiskFromLine();
                mode = 0;
            }
            else if(mode == 2){
                robot.launcher.setRPM(robot.launcher.launcherSettings.autoLaunchRPM);
                robot.launcher.goToLine();
                robot.launcher.runWheelOnTrigger = false;
                mode = 0;
            }
            else if(mode == 3){
                robot.movement.turnToAngle(0 , robot.movement.movementSettings.finalPosSettings.toRotAngleSettings());
                mode = 0;
            }
            else if(mode == 4)
            {
                robot.grabber.autoDrop();
                mode = 0;
            }
            else if(mode == 5)
            {
                robot.launcher.autoLaunchPowerShotsV2();
                mode = 0;
            }
        }
    }
}
