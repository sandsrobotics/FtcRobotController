package org.firstinspires.ftc.teamcode.robot2020;

import androidx.room.Room;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot2020.persistence.AppDatabase;
import org.firstinspires.ftc.teamcode.robot2020.persistence.MovementEntity;

import java.util.ArrayList;
import java.util.List;

import static android.os.SystemClock.sleep;

public class ComplexMovement {

    //////////////////
    //user variables//
    //////////////////
    protected long measureDelay = 10; //in ms
    protected long maxTime = 1000; //in ms

    ///////////////////
    //other variables//
    ///////////////////
    protected List<int[]> positions;
    protected List<double[]> velocities;
    protected boolean isRecording = false;
    protected double curRecordingLength = 0;
    protected boolean startOfRecording = true;
    protected int[] motorStartOffset;

    //other class objects
    Robot robot;

    ComplexMovement(Robot robot)
    {
        this.robot = robot;
    }

    void recorder()
    {
        if (measureDelay > maxTime) robot.addTelemetryString("error in ComplexMovement.recorder: ", "measure delay is more than max length so can not record");
        if(isRecording)
        {
            if(curRecordingLength + measureDelay > maxTime)
            {
                robot.addTelemetryString("ComplexMovement.recorder has stopped recording: ", "this recording has stopped at time " + curRecordingLength + "ms: stop recording to make file");
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

                curRecordingLength += measureDelay;
            }
            sleep(measureDelay);
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

    void stopRecording(boolean makeFile, String name)
    {
        if(makeFile)
        {
            if (name == null || name.equals("")) name = "not named";
            AppDatabase db = Room.databaseBuilder(AppUtil.getDefContext(), AppDatabase.class, "FIRST_INSPIRE_2020").build();
            MovementEntity entity = new MovementEntity(name, 0, (int) maxTime, measureDelay);
            db.movementEntityDAO().insertAll(entity);
            for (int i = 0; i < positions.size(); i++) {
                for (int m = 1; m <= robot.motorConfig.driveMotors.size(); m++) {
                    MovementEntity entity1 = new MovementEntity(name, m, positions.get(i)[m-1], velocities.get(i)[m-1]);
                    db.movementEntityDAO().insertAll(entity1);
                }
            }
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
}
