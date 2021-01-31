package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@TeleOp(name = "URM09ULTRA Test", group = "Tests")
public class URM09ULTRATest extends LinearOpMode
{
    private URM09ULTRA distSensor;

    public void runOpMode()
    {
        distSensor = hardwareMap.get(URM09ULTRA.class, "distSensor");

        URM09ULTRA.Parameters parameters = new URM09ULTRA.Parameters();
        parameters.maxRange = URM09ULTRA.MaxRange.CM500;
        parameters.measureMode = URM09ULTRA.MeasureMode.PASSIVE;
        distSensor.initialize(parameters);

        waitForStart();

        while(opModeIsActive())
        {
            distSensor.measureRange();
            telemetry.addData("Distance", distSensor.getDistanceCm());
            telemetry.addData("Temperature", distSensor.getTemperatureF());
            //telemetry.addData("Config", Integer.toHexString(distSensor.readShort(URM09ULTRA.Register.CFG_INDEX)));
            telemetry.addData("Manufacturer", distSensor.getManufacturer());
            telemetry.update();
            idle();
        }
    }
}
