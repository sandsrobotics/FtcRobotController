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
    protected String calibrationFileDir = "assets";
    protected String calibrationFileName =  "Launcher Config - Test.csv";

    protected boolean useRPM = true;
    protected int powerColumn = 0;
    protected int rpmColumn = 1;
    protected int distanceColumn = 2;

    ///////////////////
    //other variables//
    ///////////////////
    protected ArrayList<List<Double>> calibrationValues;
    protected ArrayList<List<Double>> formattedCalibrationValues;

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
            formattedCalibrationValues = removeDataWithYZeros(calibrationValues,distanceColumn);
            if(useRPM && powerColumn != 0) formattedCalibrationValues = removeColumn(formattedCalibrationValues,powerColumn);
            else if(rpmColumn != 0) formattedCalibrationValues = removeColumn(formattedCalibrationValues, rpmColumn);
        }
        catch (Exception e) {robot.addTelemetry("error", e.toString());}
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
            if(robot.debug_methods)robot.addTelemetry("error", e.toString());
        }
        return out;
    }

    List<Double> getColumn(ArrayList<List<Double>> data, int column)
    {
        List<Double> out = new ArrayList<>();
        for(List<Double> line: data) out.add(line.get(column - 1));
        return out;
    }

    ArrayList<List<Double>> removeColumn(ArrayList<List<Double>> data, int column)
    {
        for(int i = 0; i < data.size(); i++) data.get(i).remove(column);
        return data;
    }

    ArrayList<List<Double>> removeDataWithYZeros(ArrayList<List<Double>> data, int yPos)
    {
        ArrayList<List<Double>> out = new ArrayList<>();
        for(List<Double> line: data) if(line.get(yPos) != 0) out.add(line);
        return out;
    }

    List<Double> getEquation(List<Double> x, List<Double> y)//will return m,x,b
    {
        List<Double> out = new ArrayList<>();


        return out;
    }
}// class end