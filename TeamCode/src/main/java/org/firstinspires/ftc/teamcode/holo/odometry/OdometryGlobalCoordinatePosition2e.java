package org.firstinspires.ftc.teamcode.holo.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.holo.SandsRobot;

import java.io.File;

/**
 * Created by Sarthak on 6/1/2019.
 */
public class OdometryGlobalCoordinatePosition2e implements Runnable{
    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double verticalEncoderWheelPosition = 0, normalEncoderWheelPosition = 0,  changeInRobotOrientation = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double previousVerticalEncoderWheelPosition = 0, prevNormalEncoderWheelPosition = 0;
    private double previousOrientation = 0, currentOrientation = 0;

    //Algorithm constants
    private double horizontalEncoderTickPerDegreeOffset = 0;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    private int verticalEncoderPositionMultiplier = 1;
    private int normalEncoderPositionMultiplier = 1;

    private SandsRobot robot;

    /**
     * Constructor for GlobalCoordinatePosition Thread
     * @param robot robot that includes vertical and horizontal encoders
     * @param threadSleepDelay delay in milliseconds for the GlobalPositionUpdate thread (50-75 milliseconds is suggested)
     */
    public OdometryGlobalCoordinatePosition2e(SandsRobot robot, double COUNTS_PER_INCH, int threadSleepDelay){
        this.robot = robot;
        sleepTime = threadSleepDelay;
    }

    /**
     * Updates the global (x, y, theta) coordinate position of the robot using the odometry encoders
     */
    private void globalCoordinatePositionUpdate(){
        //Get Current Positions
        verticalEncoderWheelPosition = (robot.verticalRightEncoder.getCurrentPosition() * verticalEncoderPositionMultiplier);
        double verticalChange = verticalEncoderWheelPosition - previousVerticalEncoderWheelPosition;

        //Calculate Angle
        currentOrientation = robot.getZAngle();
        changeInRobotOrientation =  robot.getZAngle() - previousOrientation;
        robotOrientationRadians = ((robotOrientationRadians + changeInRobotOrientation));

        //Get the components of the motion
        normalEncoderWheelPosition = (robot.horizontalEncoder.getCurrentPosition()*normalEncoderPositionMultiplier);
        double rawHorizontalChange = normalEncoderWheelPosition - prevNormalEncoderWheelPosition;
        double horizontalChange = rawHorizontalChange - (changeInRobotOrientation*horizontalEncoderTickPerDegreeOffset);

        double p = verticalChange;
        double n = horizontalChange;

        //Calculate and update the position values
        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (p*Math.sin(robotOrientationRadians) + n*Math.cos(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (p*Math.cos(robotOrientationRadians) - n*Math.sin(robotOrientationRadians));

        previousVerticalEncoderWheelPosition = verticalEncoderWheelPosition;
        prevNormalEncoderWheelPosition = normalEncoderWheelPosition;
        previousOrientation = currentOrientation;
    }

    /**
     * Returns the robot's global x coordinate
     * @return global x coordinate
     */
    public double returnXCoordinate(){ return robotGlobalXCoordinatePosition; }

    /**
     * Returns the robot's global y coordinate
     * @return global y coordinate
     */
    public double returnYCoordinate(){ return robotGlobalYCoordinatePosition; }

    /**
     * Returns the robot's global orientation
     * @return global orientation, in degrees
     */
    public double returnOrientation(){ return Math.toDegrees(robotOrientationRadians) % 360; }

    /**
     * Stops the position update thread
     */
    public void stop(){ isRunning = false; }

    public void reverseVerticalEncoder(){
        if(verticalEncoderPositionMultiplier == 1){
            verticalEncoderPositionMultiplier = -1;
        }else{
            verticalEncoderPositionMultiplier = 1;
        }
    }

    public void reverseNormalEncoder(){
        if(normalEncoderPositionMultiplier == 1){
            normalEncoderPositionMultiplier = -1;
        }else{
            normalEncoderPositionMultiplier = 1;
        }
    }

    /**
     * Runs the thread
     */
    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}
