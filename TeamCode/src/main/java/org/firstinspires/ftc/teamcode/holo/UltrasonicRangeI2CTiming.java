package org.firstinspires.ftc.teamcode.holo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@Config
@TeleOp(name = "Ultrasonic Range Timing", group = "Tests")
public class UltrasonicRangeI2CTiming extends LinearOpMode
{
    public static int NUM_READS = 100;
    public static boolean READ_DIST = true;
    public void runOpMode()
    {
        DFR304Range distSensor = hardwareMap.get(DFR304Range.class, "distSensor");

        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM500;
        parameters.measureMode = DFR304Range.MeasureMode.ACTIVE;
        distSensor.initialize(parameters);

        waitForStart();
        int distance = 0;

        while(opModeIsActive())
        {
            final long startLoop = System.currentTimeMillis();
            if (READ_DIST) {
                for (int i = 0; i < NUM_READS; i++) {
                    distance = distSensor.getDistanceCm();
                }
                telemetry.addData("Distance", distance);
                telemetry.addData("Duration 1 read", (System.currentTimeMillis() - startLoop)/NUM_READS);
            }
            else {
                telemetry.addData("No Read time", System.currentTimeMillis() - startLoop);
            }
            telemetry.update();
            //idle();
        }

    }
}
