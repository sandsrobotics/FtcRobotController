package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test position tracking v2.4")
public class Test extends LinearOpMode
{
    public static int calTime = 1;
    Velocity diff;
    public int i = 0;

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,true, false, false, false, false);

        waitForStart();

        robot.startTelemetry();
        robot.position.start();
        //robot.movement.turnToAngle(90,.5,50,1000);
        robot.movement.moveToPosition(new double[]{10,10,90}, new double[]{1,1,3}, 1000, Movement.movePID, Movement.turnPID, .5);
/*
            //robot.movement.headlessMoveForTeleOp(gamepad1 , 0);
            robot.addTelemetry("position x: ", robot.position.currentRobotPosition[0]);
            robot.addTelemetry("position y: ", robot.position.currentRobotPosition[1]);
            robot.addTelemetry("rotation: ", robot.position.currentRobotPosition[2]);
            robot.addTelemetry("test: ", i);

 */
        robot.sendTelemetry();

    }
}
