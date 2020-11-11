package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// test
@Config
@TeleOp(name = "test position tracking")
public class Test extends LinearOpMode
{
    public static int calTime = 1;
    Velocity diff;

    Robot robot;

    @Override
    public void runOpMode()
    {

        robot = new Robot(this,true, false, false, false, false);

        robot.startTelemetry();

        waitForStart();

        robot.position.start();

        calibrate(10);

        robot.delay(10000);

    }

    void test()
    {
        for(int i = 0; i < 10; i++)
        {

            robot.startTelemetry();
            robot.addTelemetry("test: ", i + 1);
            robot.sendTelemetry();
            robot.delay(100);
        }
    }

    Velocity calibrate(int sec)
    {
        Velocity[] changes = new Velocity[sec];

        for(int i = 0; i < sec; i++)
        {
            robot.position.resetVelocity();
            robot.delay(1000);

            changes[i] = robot.position.currentVelocity;
            robot.startTelemetry();
            robot.addTelemetry("change in second " + i + " is: ", changes[i]);
            robot.sendTelemetry();
        }

        Velocity out = new Velocity();

        for(Velocity v:changes)
        {
            out.xVeloc += v.xVeloc;
            out.yVeloc += v.yVeloc;
            out.zVeloc += v.zVeloc;
        }

        out.xVeloc /= sec;
        out.yVeloc /= sec;
        out.zVeloc /= sec;

        return out;
    }
}
