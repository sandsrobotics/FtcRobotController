package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.prefs.BackingStoreException;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.FRONT;


public class Vision
{
    //////////////////
    //user variables//
    //////////////////
    //just some stuff to get a working vuforia object
    protected final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    protected final boolean PHONE_IS_PORTRAIT = false;
    protected boolean useExtendedTracking = false;
    protected final String VUFORIA_KEY = "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";

    //set some measurements of the field(for position tracking) IN INCHES!!!
    private static final float trackablesHeight = 6;
    private static final float halfField = 72;
    private static final float quadField  = 36;

    //to know where the phone or camera is IN INCHES!!! and degrees
    float[] phonePosition = {0,0,0};
    float[] phoneRotation = {0,0,0};

    ////////////////////
    // other variables//
    ////////////////////
    //converting inch to mm
    private static final float mmPerInch = 25.4f;
    // some vuforia stuff
    protected OpenGLMatrix lastLocation = null;
    protected Orientation lastRotation = null;
    protected Position lastPosition = null;
    protected VuforiaLocalizer vuforia = null;
    protected VuforiaTrackables trackables;
    protected VuforiaLocalizer.Parameters parameters;
    //other
    boolean targetVisible = false;


    //other class
    Robot robot;

    Vision(Robot robot) { this.robot = robot; }
    void initAll()
    {
        initVuforia();
        loadAsset("UltimateGoal");
        setAllTrackablesNames();
        setAllTrackablesPosition();
        setPhoneTransform(phonePosition, phoneRotation);
    }

    void initVuforia()
    {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = useExtendedTracking;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // put camera image on dashboard
    }

    void startDashboardCameraStream(int maxFps){FtcDashboard.getInstance().startCameraStream(vuforia,maxFps);}
    void stopDashboardCameraStream(){FtcDashboard.getInstance().stopCameraStream();}

    void loadAsset(String assetName)
    {
        trackables = this.vuforia.loadTrackablesFromAsset(assetName);
    }

    void setAllTrackablesNames()
    {
        setTrackableName(0,"Blue Tower Goal Target");
        setTrackableName(1,"Red Tower Goal Target");
        setTrackableName(2,"Red Alliance Target");
        setTrackableName(3,"Blue Alliance Target");
        setTrackableName(4,"Front Wall Target");
    }

    void setAllTrackablesPosition()
    {
        setTrackableTransform(2,new float[]{0, -halfField, trackablesHeight}, new float[]{90, 0, 180});
        setTrackableTransform(3,new float[]{0, halfField, trackablesHeight}, new float[]{90, 0, 0});
        setTrackableTransform(4,new float[]{-halfField, 0, trackablesHeight}, new float[]{90, 0, 90});
        setTrackableTransform(0,new float[]{halfField, quadField, trackablesHeight}, new float[]{90, 0, -90});
        setTrackableTransform(1,new float[]{halfField, -quadField, trackablesHeight}, new float[]{90, 0, -90});
    }

    void setTrackableName(int posInTrackables, String name)
    {
        trackables.get(posInTrackables).setName(name);
    }
    
    void setTrackableTransform(int posInTrackables, float[] position, float[] angles) // position is in inches and rotation is in deg with order XYZ
    {
        for(int i = 0; i < position.length; i++) position[i] *= mmPerInch;

        trackables.get(posInTrackables).setLocation(OpenGLMatrix
                .translation(position[0], position[1], position[2])
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, angles[0], angles[1], angles[2])));
    }

    void setPhoneTransform(float[] position, float[] angles) // positions is in mm: X is mm left from center line, Y is mm above ground, Z is mm forward from center line. rotation is in order XYZ in deg
    {
        for(int i = 0; i < position.length; i++) position[i] *= mmPerInch;

        if (CAMERA_CHOICE == BACK)
        {
            angles[1] -= 90;
        }
        else
        {
            angles[1] += 90;
        }

        if (PHONE_IS_PORTRAIT)
        {
            angles[0] += 90;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(position[2], position[0], position[1])
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, angles[1], angles[2], angles[0]));

        for (VuforiaTrackable trackable : trackables)
        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    void setPhoneTorch(boolean on)
    {
        CameraDevice.getInstance().setFlashTorchMode(on);
    }

    void activate(){trackables.activate();}
    void deactivate(){trackables.deactivate();}

    boolean findTrackable(int trackableNum, boolean logPosition)
    {
        if (((VuforiaTrackableDefaultListener) trackables.get(trackableNum).getListener()).isVisible())
        {
            targetVisible = true;
            if(logPosition) {

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackables.get(trackableNum).getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null)
                {
                    lastLocation = robotLocationTransform;
                    lastRotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                    //lastPosition = lastLocation.;
                }
            }
            return true;
        }
        else
        {
            targetVisible = false;
            return false;
        }
    }

    boolean findTrackableDelay(int trackableNum, boolean logPosition, int maxTries)
    {
        for(int i = 0; i < maxTries; i++)
        {
            if(findTrackable(trackableNum,logPosition)) return true;
        }

        return false;
    }

    boolean findAnyTrackable(boolean logPosition)
    {
        for(int i = 0; i < trackables.size(); i++)
        {
            findTrackable(i,logPosition);
           if(targetVisible) return true;
        }
        return false;
    }

    void printTelemetry()
    {
        if(robot.debug_methods){
            if(targetVisible)
            {
                robot.addTelemetryString("target visible: ", "true");
                if(robot.debug_dashboard)
                {
                    robot.packet.put("position: ", lastLocation.getTranslation());
                    robot.packet.put("rotation: ", lastRotation);
                }
                if(robot.debug_telemetry)
                {

                    robot.telemetry.addData("position: ", lastLocation.getTranslation());
                    robot.telemetry.addData("rotation: ", lastLocation);
                }
            }
            else
            {
                robot.addTelemetryString("target visible: ", "false");
                if(robot.debug_dashboard)
                {
                    robot.packet.put("position: ", null);
                    robot.packet.put("rotation: ", null);
                }
                if(robot.debug_telemetry)
                {
                    robot.packet.put("position: ", null);
                    robot.packet.put("rotation: ", null);
                }
            }
        }
    }
}
