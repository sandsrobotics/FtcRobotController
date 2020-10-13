package org.firstinspires.ftc.teamcode.robot2020;

import android.os.Build;
import android.os.Environment;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.config.Config;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static android.os.Environment.DIRECTORY_DOWNLOADS;
import static android.os.Environment.getRootDirectory;

@Config
public class Launcher {

    //////////////////
    //user variables//
    //////////////////
    public static String calibrationFileName = "test.txt";
    public static String calibrationFileDir = "/system/Internal storage/Download";

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
            BufferedReader br = new BufferedReader(new FileReader(calibrationFileDir + "/" + calibrationFileName));

            String line;
            while ((line = br.readLine()) != null)
            {
                String[] curLine = line.split(",");
                List<Double> values = new ArrayList<>();
                try
                {
                    for(String s:curLine) values.add(Double.parseDouble(curLine[0]));
                    calibrationValues.add(values);
                }
                catch (Exception e) { }
            }
        }
        catch(IOException e)
        {
            robot.addTelemetryString("failed to read: ", e.toString());
            e.printStackTrace();
        }
    }
}// class end