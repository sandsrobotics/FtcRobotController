package org.firstinspires.ftc.teamcode.robot2020;

import android.database.Cursor;

import androidx.room.Query;
import androidx.room.Room;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot2020.persistence.AppDatabase;
import org.firstinspires.ftc.teamcode.robot2020.persistence.MovementEntity;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;

@Config
public class ComplexMovement {

    //////////////////
    //user variables//
    //////////////////
    //make
    protected String dataBaseName = "FIRST_INSPIRE_2020";
    public static double measureDelay = 0; //in ms
    public static double maxTime = 1350; //in ms
    public static double timePerLoop = 13.5; //in ms
    //load
    public static double load_TimePerLoop = 13;

    ///////////////////
    //other variables//
    ///////////////////
    protected AppDatabase db;
    //make
    protected List<int[]> positions;
    protected List<double[]> velocities;
    protected boolean isRecording = false;
    protected double curRecordingLength = 0;
    protected boolean startOfRecording = true;
    protected int[] motorStartOffset;
    //load
    protected List<int[]> loaded_Positions;
    protected List<double[]> loaded_Velocities;
    protected double loaded_TimePerLoop;
    protected double loaded_MeasureDelay;
    protected double loaded_TotalTime;

    //other class objects
    Robot robot;

    ComplexMovement(Robot robot)
    {
        this.robot = robot;
    }

    void recorder()
    {
        if (measureDelay > maxTime || (measureDelay == 0 && timePerLoop == 0) && robot.debug_methods) robot.addTelemetryString("error in ComplexMovement.recorder: ", "measure delay is more than max length so can not record");
        if(isRecording)
        {
            if(curRecordingLength + measureDelay + timePerLoop > maxTime)
            {
                if(robot.debug_methods)robot.addTelemetryString("ComplexMovement.recorder has stopped recording: ", "this recording has stopped at time " + curRecordingLength + " ms: stop recording to make file");
                isRecording = false;
            }
            else
            {
                if(startOfRecording)
                {
                    motorStartOffset = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
                    startOfRecording = false;
                }

                int[] pos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
                for(int i = 0; i < pos.length; i++) pos[i] -= motorStartOffset[i];
                positions.add(pos);

                velocities.add(robot.motorConfig.getMotorVelocitiesList(robot.motorConfig.driveMotors));

                curRecordingLength += measureDelay + timePerLoop;
            }
            if(measureDelay > 0) sleep((long)measureDelay);
        }
    }
    void startRecording(boolean reset)
    {
        if (reset) resetRecording();
        isRecording = true;
    }

    void pauseRecording()
    {
        isRecording = false;
    }

    void stopRecording(boolean makeFile, String moveName)
    {
        if(makeFile)
        {
            makeFile(moveName);
        }
        isRecording = false;
    }

    void resetRecording()
    {
        positions = new ArrayList<>();
        velocities = new ArrayList<>();
        curRecordingLength = 0;
        startOfRecording = true;
        motorStartOffset = null;
    }

    void makeFile(String moveName)
    {
        if (moveName == null || moveName.equals("")) moveName = "not named";
        db = Room.databaseBuilder(AppUtil.getDefContext(), AppDatabase.class, dataBaseName).build();
        MovementEntity entity = new MovementEntity(moveName, 0, (int) maxTime, measureDelay);
        db.movementEntityDAO().insertAll(entity);
        entity = new MovementEntity(moveName, 0, (int) curRecordingLength, timePerLoop);
        db.movementEntityDAO().insertAll(entity);
        for (int i = 0; i < positions.size(); i++)
        {
            for (int m = 0; m < robot.motorConfig.driveMotors.size(); m++) {
                MovementEntity entity1 = new MovementEntity(moveName, m + 1, positions.get(i)[m], velocities.get(i)[m]);
                db.movementEntityDAO().insertAll(entity1);
            }
        }
        List<MovementEntity> mlist = db.movementEntityDAO().getAll();
    }

    void loadMove(String moveName)
    {
        //for()
    }
    void clearMove()
    {
        loaded_Positions = null;
        loaded_Velocities = null;
        loaded_TimePerLoop = 0;
        loaded_MeasureDelay = 0;
        loaded_TotalTime = 0;
    }
    void runMove()
    {
        double delay = loaded_MeasureDelay + loaded_TimePerLoop - load_TimePerLoop;
        if(delay < 0)
        {
            delay = 0;
            if(robot.debug_methods) robot.addTelemetryString("error in method ComplexMovement.runMove: ", "the time it take to calculate and set motor velocities may be more than the time interval between measuring, this could cause problems");
        }
        double[] calculatedVelocity = null;
        int[] currentPos;
        int[] startPos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);

        for(int i = 0; i < loaded_Positions.size(); i++)
        {
            if(i == 0) calculatedVelocity = loaded_Velocities.get(i);
            else
            {
                currentPos = robot.motorConfig.getMotorPositionsList(robot.motorConfig.driveMotors);
                for(int c = 0; c < calculatedVelocity.length; c++)
                {
                    calculatedVelocity[c] = (loaded_Positions.get(i)[c] - (currentPos[c] - startPos[c])) / (delay/1000);
                    if(robot.debug_methods && Math.abs(calculatedVelocity[c] - loaded_Velocities.get(i)[c]) > 10) robot.addTelemetryString("warning in method ComplexMovement.runMove: ","the calculated velocity has an error more then 10 compared to loaded velocity");
                }
            }

            robot.motorConfig.setMotorVelocitiesList(robot.motorConfig.driveMotors, calculatedVelocity);
            sleep((long)delay);
        }
    }
}
