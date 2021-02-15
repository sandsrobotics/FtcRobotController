package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@Disabled
@TeleOp(name = "Ultrasonic Range Test", group = "Tests")
public class UltrasonicRangeTest extends LinearOpMode
{
    public void runOpMode()
    {
        DFR304Range distSensor0 = hardwareMap.get(DFR304Range.class, "distSensor0");
        DFR304Range distSensor1 = hardwareMap.get(DFR304Range.class, "distSensor1");

        DFR304Range.Parameters parameters = new DFR304Range.Parameters();
        parameters.maxRange = DFR304Range.MaxRange.CM500;
        parameters.measureMode = DFR304Range.MeasureMode.ACTIVE;
        distSensor0.initialize(parameters);
        distSensor1.initialize(parameters);

        waitForStart();

        while(opModeIsActive())
        {
            //distSensor.measureRange();
            telemetry.addData("Distance 0 IN", distSensor0.getDistanceIn());
            telemetry.addData("Distance 1 IN", distSensor1.getDistanceIn());
            telemetry.addData("Distance 0 CM", distSensor0.getDistanceCm());
            telemetry.addData("Distance 1 CM", distSensor1.getDistanceCm());
            telemetry.addData("Temperature 0 F", distSensor0.getTemperatureF());
            telemetry.addData("Temperature 1 F", distSensor1.getTemperatureF());
            //telemetry.addData("Temperature0 C", distSensor0.getTemperatureC());
            //telemetry.addData("Temperature1 C", distSensor1.getTemperatureC());
            //telemetry.addData("Config0", (distSensor0.readOneByte(DFR304Range.Register.CFG_INDEX) & 0xFF));
            //telemetry.addData("Config0", (distSensor1.readOneByte(DFR304Range.Register.CFG_INDEX) & 0xFF));
            //telemetry.addData("Manufacturer0", distSensor0.getManufacturer());
            //telemetry.addData("Manufacturer0", distSensor1.getManufacturer());
            telemetry.update();
            //idle();
        }

    }
}
