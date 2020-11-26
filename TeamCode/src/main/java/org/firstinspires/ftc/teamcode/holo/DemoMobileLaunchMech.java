package org.firstinspires.ftc.teamcode.holo;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Demo Mecanum Mobile Launch")
public class DemoMobileLaunchMech extends LinearOpMode {
    SandsRobot robot;
    SandsLauncher launcher;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        launcher = new SandsLauncher(robot);
        waitForStart();

        while(opModeIsActive()){
            robot.controlDrivetrain();
            //robot.sendTelemetry();
            launcher.getInputs();
            launcher.setLauncherWheelMotor();
            launcher.data_out();
        }
        robot.stop();
    }


}

