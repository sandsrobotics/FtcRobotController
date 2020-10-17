package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// test
@Config
@TeleOp(name = "test launcher read")
public class Test extends LinearOpMode
{

    Robot robot;


    @Override
    public void runOpMode()
    {
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,false, true, false, false);



        waitForStart();

        while (opModeIsActive())
        {

        }
    }
}
