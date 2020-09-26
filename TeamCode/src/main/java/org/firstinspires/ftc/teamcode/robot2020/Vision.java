package org.firstinspires.ftc.teamcode.robot2020;

import com.vuforia.CameraDevice;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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
    protected final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    protected final boolean PHONE_IS_PORTRAIT = false;
    protected final String VUFORIA_KEY = "Ad6cSm3/////AAABmRkDMfGtWktbjulxwWmgzxl9TiuwUBtfA9n1VM546drOcSfM+JxvMxvI1WrLSLNdapOtOebE6n3BkjTjyj+sTXHoEyyJW/lPPmlX5Ar2AjeYpTW/WZM/lzG8qDPsm0tquhEj3BUisA5GRttyGXffPwfKJZNPy3WDqnPxyY/U2v+jQNfZjsWqNvUfp3a3klhVPYd25N5dliMihK3WogqNQnZM9bwJc1wRT0zcczYBJJrhpws9A5H2FpOZD6Ov7GqT+rJdKrU6bh+smoueINDFeaFuYQVMEeo7VOLgkzOeRDpfFmVOVeJrmUv+mwnxfFthAY5v90e4kgekG5OYzRQDS2ta0dbUpG6GoJMoZU2vASSa";
    protected VuforiaLocalizer vuforia = null;
    protected VuforiaTrackables trackables;
    protected VuforiaLocalizer.Parameters parameters;

    //other class
    Robot robot;

    Vision(Robot robot) { this.robot = robot; }

    void initVuforia()
    {
        int cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    void loadAsset(String assetName)
    {
        trackables = this.vuforia.loadTrackablesFromAsset(assetName);
    }

    void setTrackableName(int posInTrackables, String name)
    {
        trackables.get(posInTrackables).setName(name);
    }
    
    void setTrackableTransform(int posInTrackables, Orientation angles, Position position) // position is in mm and rotation is in deg with order XYZ
    {
        trackables.get(posInTrackables).setLocation(OpenGLMatrix
                .translation((float)position.x, (float)position.y, (float)position.z)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, angles.firstAngle, angles.secondAngle, angles.thirdAngle)));
    }

    void setPhoneTransform(Orientation angles, Position position) // positions is in mm: X is mm left from center line, Y is mm above ground, Z is mm forward from center line. rotation is in order XYZ in deg
    {
        if (CAMERA_CHOICE == BACK)
        {
            angles.secondAngle -= 90;
        }
        else
        {
            angles.secondAngle += 90;
        }

        if (PHONE_IS_PORTRAIT)
        {
            angles.firstAngle += 90;
        }

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation((float)position.z, (float)position.x, (float)position.y)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, angles.secondAngle, angles.thirdAngle, angles.firstAngle));

        for (VuforiaTrackable trackable : trackables)
        {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    void setPhoneTorch(boolean on)
    {
        CameraDevice.getInstance().setFlashTorchMode(on);
    }

    boolean findTrackable(int trackableNum)
    {
        if (((VuforiaTrackableDefaultListener) trackables.get(trackableNum).getListener()).isVisible())
        {
            if(robot.debug_methods)
            {
                if(robot.debug_dashboard) robot.packet.put("trackable found, name: ", trackables.get(trackableNum).getName());
                if(robot.debug_telemetry) robot.telemetry.addData("trackable found, name: ", trackables.get(trackableNum).getName());
            }
            return true;
        }
        return false;
    }
}
