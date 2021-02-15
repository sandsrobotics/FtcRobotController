package org.firstinspires.ftc.teamcode.holo;
    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp(name = "Demo Mobile Launch")
public class DemoMobileLaunchMech extends LinearOpMode {
    SandsRobot robot;
    SandsManipulators manipulators;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry, gamepad1, gamepad2);
        manipulators = new SandsManipulators(robot);
        robot.getPIDFCoefficients();
        waitForStart();

        while(opModeIsActive()){
            robot.setPIDFCoefficients();
            robot.controlDrivetrain();
            manipulators.getInputs();
            manipulators.setLauncherWheelMotor();
            manipulators.launch();
            manipulators.data_out();
        }
        robot.stop();
    }


}

