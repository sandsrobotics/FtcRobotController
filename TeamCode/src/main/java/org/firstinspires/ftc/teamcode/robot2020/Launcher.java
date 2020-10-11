package org.firstinspires.ftc.teamcode.robot2020;

import android.content.pm.PackageInfo;
import android.content.pm.PackageManager;
import android.provider.ContactsContract;

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
    protected String calibrationFileName = "lancher test.csv";

    /////////
    //other//
    /////////
    protected ArrayList<List<Double>> calibrationValues;


    //other classes
    Robot robot;

    Launcher(Robot robot) {
        this.robot = robot;
    }

    void readCsv() {
        try
        {

            BufferedReader br = new BufferedReader(new FileReader( ""+ "\\" +calibrationFileName));
            String line;
            while ((line = br.readLine()) != null)
            {
                String[] curLine = line.split(",");
                List<Double> values = new ArrayList<Double>();
                try
                {
                    for(String s:curLine) values.add(Double.parseDouble(curLine[0]));
                    calibrationValues.add(values);
                }
                catch (Exception e){}
            }
        }
        catch(IOException e)
        {
            robot.packet.put("error", e);
            e.printStackTrace();
        }
    }
}// class end