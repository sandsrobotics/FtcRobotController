package org.firstinspires.ftc.teamcode.robot2020;


import android.Manifest;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.util.Log;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.acmerobotics.dashboard.config.Config;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import static android.os.Environment.DIRECTORY_DOWNLOADS;
import static android.os.Environment.getRootDirectory;
import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuMarkIdentification.TAG;

@Config
public class Launcher extends AppCompatActivity {

    //////////////////
    //user variables//
    //////////////////
    protected ArrayList<List<Double>> calibrationValues;

    //other classes
    Robot robot;

    Launcher(Robot robot) { this.robot = robot; }

    void readCSV(String Name)
    {
        try
        {
            BufferedReader reader = new BufferedReader(new InputStreamReader(getAssets().open(Name)));

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
                if(valid) calibrationValues.add(values);
            }
        }
        catch (IOException e)
        {
            robot.addTelemetryString("error", e.toString());
        }
    }
}// class end