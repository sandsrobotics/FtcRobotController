package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

@Config
public class Launcher {

    Robot robot;

    //////////////////
    //user variables//
    //////////////////
    //controls
    Gamepad launcherGamepad = robot.gamepad1;
    GamepadButtons revIncreaseButton = GamepadButtons.X;
    GamepadButtons revDecreaseButton = GamepadButtons.X;
    GamepadButtons revPowerSlide = GamepadButtons.X;
    GamepadButtons revModeButton = GamepadButtons.X;
    GamepadButtons launchButton = GamepadButtons.A;

    //servo and motor config
    double ticksPerRev = 10;
    double gearRatio = 1;
    double maxRPM = 6000;
    double servoRestAngle = 15;
    double servoLaunchAngle = 30;

    //calibration data
    protected String calibrationFileDir = "assets";
    protected String calibrationFileName =  "Launcher Config - Test.csv";
    protected boolean useRPM = true;
    protected int powerColumn = 0;
    protected int rpmColumn = 1;
    protected int distanceColumn = 2;

    //other
    double rpmIncrements = 50;


    ///////////////////
    //other variables//
    ///////////////////
    //controls
    boolean revIncreaseButtonPressed = false;
    boolean revDecreaseButtonPressed = false;
    boolean revModeButtonPressed = false;


    //servo and motor
    double spinMultiplier = 60 / ticksPerRev * gearRatio;

    //calibration
    protected ArrayList<List<Double>> calibrationValues;
    protected ArrayList<List<Double>> formattedCalibrationValues;

    //other
    boolean runWheelOnTrigger = false;
    double targetWheelRpm = 0;

    Launcher(Robot robot) { this.robot = robot; }

    ///////////////
    //Calibration//
    ///////////////
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

    ////////////////////
    //launcher control//
    ////////////////////
    void initStuff()
    {
        // zero motors
        robot.motorConfig.launcherServo.setPosition(servoRestAngle);

        //reset variables
        boolean runWheelOnTrigger = false;
        double setWheelRpm = 0;
    }

    void telemetryDataOut()
    {
        double RPM = robot.motorConfig.launcherWheelMotor.getVelocity() * spinMultiplier;
        robot.addTelemetry("RPM", RPM);
        robot.addTelemetry("Set RPM", targetWheelRpm);
        robot.sendTelemetry();
    }

    void setLauncherServo()
    {
        if (launchButton.getButtonPressed(launcherGamepad)) robot.motorConfig.launcherServo.setPosition(servoLaunchAngle);
        else robot.motorConfig.launcherServo.setPosition(servoRestAngle);
    }

    void setLauncherWheelMotor()
    {
        //inputs
        if(revDecreaseButton.getButtonPressed(launcherGamepad))
        {
            if(!revDecreaseButtonPressed)
            {
                targetWheelRpm -= rpmIncrements;
                if(targetWheelRpm < 0) targetWheelRpm = 0;
                revDecreaseButtonPressed = true;
            }
        }
        else revDecreaseButtonPressed = false;

        if(revIncreaseButton.getButtonPressed(launcherGamepad))
        {
            if(!revIncreaseButtonPressed)
            {
                targetWheelRpm += rpmIncrements;
                if(targetWheelRpm > maxRPM) targetWheelRpm = maxRPM;
                revIncreaseButtonPressed = true;
            }
        }
        else revIncreaseButtonPressed = false;

        if(revModeButton.getButtonPressed(launcherGamepad))
        {
            if(!revModeButtonPressed)
            {
                runWheelOnTrigger =! runWheelOnTrigger;
                revModeButtonPressed = true;
            }
        }
        else revModeButtonPressed = false;

        //setting motor
        if (runWheelOnTrigger) robot.motorConfig.launcherWheelMotor.setPower(revPowerSlide.getSliderValue(launcherGamepad));
        else robot.motorConfig.launcherWheelMotor.setVelocity(targetWheelRpm / spinMultiplier);
    }

    void telemetryRun(boolean telemetry)
    {
        setLauncherServo();
        setLauncherWheelMotor();
        if(telemetry)telemetryDataOut();
    }
}// class end