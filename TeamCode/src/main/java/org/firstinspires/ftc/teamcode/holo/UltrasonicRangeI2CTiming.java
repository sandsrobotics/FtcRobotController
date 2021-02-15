package org.firstinspires.ftc.teamcode.holo;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@Disabled
@Config
@TeleOp(name = "Ultrasonic Range Timing", group = "Tests")
public class UltrasonicRangeI2CTiming extends LinearOpMode
{
    public static int NUM_ITERATIONS = 100;
    public static boolean READ_DIST_0 = true;
    public static boolean READ_DIST_1 = true;
    public void runOpMode()
    {
        DFR304Range distSensor0 = hardwareMap.get(DFR304Range.class, "distSensor0");
        DFR304Range distSensor1 = hardwareMap.get(DFR304Range.class, "distSensor1");

        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM500;
        parameters.measureMode = DFR304Range.MeasureMode.ACTIVE;
        distSensor0.initialize(parameters);
        distSensor1.initialize(parameters);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("x", 3.7);
        dashboardTelemetry.update();
        waitForStart();
        int distance1 = 0;
        int distance0 = 0;
        long deltaTime = 0;

        while(opModeIsActive())
        {
            final long startLoop = System.nanoTime();
            if (READ_DIST_0 | READ_DIST_1) {
                for (int i = 0; i < NUM_ITERATIONS; i++) {
                    if (READ_DIST_0) distance0 = distSensor0.getDistanceCm();
                    else distance0 = 0;
                    if (READ_DIST_1) distance1 = distSensor1.getDistanceCm();
                    else distance1 = 0;
                }
                deltaTime = (System.nanoTime() - startLoop) / 1000000;
                telemetry.addData("Distance 0", distance0);
                telemetry.addData("Distance 1", distance1);
                telemetry.addData("Number of iterations", NUM_ITERATIONS);
                //telemetry.addData("Total (ms)", deltaTime);
                telemetry.addData("Average (ms)",deltaTime/NUM_ITERATIONS);
                dashboardTelemetry.addData("Average (ms)", deltaTime/NUM_ITERATIONS);
            }
            else {
                telemetry.addData("No Read time (nano)", System.nanoTime() - startLoop);
                dashboardTelemetry.addData("No Read time (nano)", System.nanoTime() - startLoop);
                //telemetry.addData("No Read time (ms)", (System.nanoTime() - startLoop)/1000000);
            }
            dashboardTelemetry.update();
            telemetry.update();
            //idle();
        }

    }
}
