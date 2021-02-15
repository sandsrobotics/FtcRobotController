package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.holo.odometry.OdometryGlobalCoordinatePosition3e;
@Disabled
@TeleOp(name = "Demo Odometry", group = "")
public class DemoOdometryPosition extends LinearOpMode {
    SandsRobot robot;
    //The amount of encoder ticks for each inch the robot moves.
    final double COUNTS_PER_INCH = 307.699557; //306.381642; // original 307.699557;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SandsRobot(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        OdometryGlobalCoordinatePosition3e globalPositionUpdate = new OdometryGlobalCoordinatePosition3e(robot, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        //globalPositionUpdate.reverseVerticalEncoder();
        //globalPositionUpdate.reverseNormalEncoder();

        while (opModeIsActive()) {
            robot.controlDrivetrain();
            //robot.sendTelemetry();
            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());
            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }
        //Stop the thread
        globalPositionUpdate.stop();
        robot.stop();
    }
}