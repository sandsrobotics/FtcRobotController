package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * OpMode for testing DFRobotics Ultrasonic Distance sensor driver
 */
@TeleOp(name = "DFRI2cRangeSensor Test", group = "Tests")
public class DRFI2cTest extends LinearOpMode
{
    private  DFRI2cRangeSensor distSensor;

    public void runOpMode()
    {
        //distSensor = hardwareMap.get(URM09ULTRA.class, "distSensor1");
        distSensor = hardwareMap.get( DFRI2cRangeSensor.class, "distSensor");
        //URM09ULTRA.Parameters parameters = new URM09ULTRA.Parameters();
        //parameters.maxRange = URM09ULTRA.MaxRange.CM150;
        //parameters.measureMode = URM09ULTRA.MeasureMode.PASSIVE;
        distSensor.initialize();

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addData("Distance", distSensor.rawUltrasonic());
            telemetry.addData("Manufacturer", distSensor.getManufacturer());

            telemetry.update();
            idle();
        }
    }
}
