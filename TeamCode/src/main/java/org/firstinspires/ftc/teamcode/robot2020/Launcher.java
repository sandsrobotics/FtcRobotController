package org.firstinspires.ftc.teamcode.robot2020;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class Launcher {

    //////////////////
    //user variables//
    //////////////////
    protected String calibrationFileName = "launcher test.csv";

    /////////
    //other//
    /////////
    protected ArrayList<List<Double>> calibrationValues;


    //other classes
    Robot robot;

    Launcher(Robot robot) {
        this.robot = robot;
    }

    void readcsv() {
        try
        {
            BufferedReader br = new BufferedReader(new FileReader(calibrationFileName));
            String line;
            while ((line = br.readLine()) != null)
            {
                String[] employee = line.split(",");
                robot.telemetry.addData("test",employee);
            }
        }
        catch(IOException e)
        {
            e.printStackTrace();
        }
    }
}// class end