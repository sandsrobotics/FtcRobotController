package org.firstinspires.ftc.teamcode.robot2020;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;

public class Launcher{

    //other classes
    Robot robot;

    Launcher(Robot robot) { this.robot = robot; }

    void readcsv(){
        try {
            BufferedReader csvReader = new BufferedReader(new FileReader("lancher test.csv"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
