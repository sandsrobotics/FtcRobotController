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
        robot = new Robot(hardwareMap,telemetry,gamepad1,gamepad2,false, true, false, false, false);

        robot.complexMovement.startRecording(true);
        robot.motorConfig.setMotorsToCoastList(robot.motorConfig.driveMotors);

        waitForStart();

        while (opModeIsActive())
        {
            robot.startTelemetry();
            while(robot.complexMovement.isRecording && !robot.stop())
            {
                robot.complexMovement.recorder();
            }
            robot.complexMovement.stopRecording(true, "move2");
            while(!gamepad1.a){if(robot.stop()) break;}
            robot.complexMovement.loadMoveDB("move2");
            robot.complexMovement.scaleLoadedMove(false);
            robot.complexMovement.runMoveV2(1);
            robot.sendTelemetry();
            break;
        }
    }
}
