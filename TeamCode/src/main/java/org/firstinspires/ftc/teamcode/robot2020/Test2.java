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
        ru.usePositionTracking = true;
        ru.useDrive = true;

        robot = new Robot(this, ru);

        robot.startTelemetry();
        robot.addTelemetry("Robot: ", "ready :)");
        robot.sendTelemetry();

        waitForStart();

        robot.start(false);

        int[] pos;
        PIDCoefficients pidCoefficients = new PIDCoefficients(7.5,0,0);
        double maxPower = .2;
        PID pidLoop = new PID(pidCoefficients,-maxPower, maxPower);
        double power;
        double rot = 0;
        GamepadButtonManager button = new GamepadButtonManager(GamepadButtons.A);

        while(opModeIsActive())
        {
            if(button.getButtonPressed(gamepad1)) rot++;

            //pos = robot.robotHardware.getMotorPositionsList(robot.robotHardware.odometryWheels);
            //power = pidLoop.updatePIDAndReturnValue(rot - (pos[0]/robot.position.positionSettings.ticksPerRotationX));
            //robot.movement.moveRobot(0,0, power, false, false);
            //robot.addTelemetry("power", power);
            robot.addTelemetry("rot", rot);
            robot.sendTelemetry();
        }
    }
}
