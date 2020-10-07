package org.firstinspires.ftc.teamcode.robot2020;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.teamcode.tests.EasyOpenCVExample;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

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
    protected final VuforiaLocalizer.CameraDirection CAMERA_CHOICE_V = BACK;
    protected final OpenCvInternalCamera.CameraDirection CAMERA_CHOICE_O = OpenCvInternalCamera.CameraDirection.BACK;
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
    //some openCV stuff
    OpenCvInternalCamera phoneCam;
    Vision.SkystoneDeterminationPipeline pipeline;
    //other
    protected boolean targetVisible = false;
    protected boolean useVuforia = false;
    protected boolean useOpenCV = false;
    protected int cameraMonitorViewId;
    //other class
    Robot robot;

    Vision(Robot robot) { this.robot = robot; }

    //////////////////
    //Vision Methods//
    //////////////////
    void initAll(boolean useVuforia, boolean useOpenCV)
    {
        this.useVuforia = useVuforia;
        this.useOpenCV = useOpenCV;

        initCamera();

        if(useVuforia)
        {
            initVuforia();
            loadAsset("UltimateGoal");
            setAllTrackablesNames();
            setAllTrackablesPosition();
            setPhoneTransform(phonePosition, phoneRotation);
        }

        if(useOpenCV)
        {
            initOpenCV();
        }
    }

    void initCamera()
    {
        cameraMonitorViewId = robot.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hardwareMap.appContext.getPackageName());
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

    ///////////////////
    //Vuforia Methods//
    ///////////////////
    void initVuforia()
    {
        //make a parameters object
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        //define the parameters object
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE_V;
        parameters.useExtendedTracking = useExtendedTracking;

        //Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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

        if (CAMERA_CHOICE_V == BACK)
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
            if(robot.stop()) break;
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

    //////////////////
    //OpenCV Methods//
    //////////////////
    void initOpenCV()
    {
        //creating a camera object
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(CAMERA_CHOICE_O, cameraMonitorViewId);

        //creating a openCV pipeline
        pipeline = new Vision.SkystoneDeterminationPipeline();

        //integrate the openCV pipeline with the camera
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        //start camera
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
    }

    int getNumberOfRings()
    {
        return pipeline.numOfRings();
    }



    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181,98);

        static final int REGION_WIDTH = 35;
        static final int REGION_HEIGHT = 25;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat HSV = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

         // This function takes the RGB frame, converts to HSV,
        public void inputToHSV(Mat input)
        {
            Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE;
            }else{
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }

        public int numOfRings()
        {
            if(position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR)
            {
                return 4;
            }
            else if(position == EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE)
            {
                return 1;
            }
            else return 0;
        }
    }
}
