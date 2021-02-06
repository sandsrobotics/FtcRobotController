package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@TeleOp(name = "Ultrasonic Range Test", group = "Tests")
public class UltrasonicRangeTest extends LinearOpMode
{
    public void runOpMode()
    {
        DFR304Range distSensor = hardwareMap.get(DFR304Range.class, "distSensor");

        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM500;
        parameters.measureMode = DFR304Range.MeasureMode.ACTIVE;
        distSensor.initialize(parameters);

        waitForStart();

        while(opModeIsActive())
        {
            final long startLoop = System.nanoTime();
            //distSensor.measureRange();
            telemetry.addData("Distance CM", distSensor.getDistanceCm());
            telemetry.addData("Distance IN", distSensor.getDistanceIn());
            telemetry.addData("Temperature F", distSensor.getTemperatureF());
            telemetry.addData("Temperature C", distSensor.getTemperatureC());
            telemetry.addData("Config", (distSensor.readOneByte(DFR304Range.Register.CFG_INDEX) & 0xFF));
            telemetry.addData("Manufacturer", distSensor.getManufacturer());
            final long duration = System.nanoTime() - startLoop;
            telemetry.update();
            idle();
        }

    }
}
