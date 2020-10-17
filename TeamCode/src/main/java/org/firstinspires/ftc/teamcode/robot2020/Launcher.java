package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

@Config
public class Launcher {

    //////////////////
    //user variables//
    //////////////////
    public static String calibrationFileDir = "assets";
    public static String calibrationFileName =  "launcher test.csv";

    ///////////////////
    //other variables//
    ///////////////////
    protected ArrayList<List<Double>> calibrationValues;
    protected List<Double> powers;
    protected List<Double> distances;

    //other classes
    Robot robot;

    Launcher(Robot robot) { this.robot = robot; }

    void getCalibration()
    {
        try
        {
            InputStream is = getClass().getClassLoader().getResourceAsStream(calibrationFileDir + "/" + calibrationFileName);
            if(is == null) throw new Exception("file directory or name are incorrect");
            calibrationValues = readFile(is);
            powers = getColumn(calibrationValues,1);
            distances = getColumn(calibrationValues,4);
        }
        catch (Exception e) {robot.addTelemetryString("error", e.toString());}
    }

    ArrayList<List<Double>> readFile(InputStream is)
    {
        ArrayList<List<Double>> out = new ArrayList<>();
        try
        {
            BufferedReader reader = new BufferedReader(new InputStreamReader(is));
            String Line;
            while ((Line = reader.readLine()) != null)
            {
                List<Double> values = new ArrayList<>();
                boolean valid = true;
                String[] elements = Line.split(",");
                for(String element:elements)
                {
                    try { values.add(Double.parseDouble(element)); }
                    catch (Exception e){ valid = false; }
                }
                if(valid) out.add(values);
            }
        }
        catch (IOException e)
        {
            robot.addTelemetryString("error", e.toString());
        }
        return out;
    }

    List<Double> getColumn(ArrayList<List<Double>> data, int column)
    {
        List<Double> out = new ArrayList<>();
        for(List<Double> line: data) out.add(line.get(column - 1));
        return out;
    }
}// class end