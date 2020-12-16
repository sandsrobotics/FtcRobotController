package org.firstinspires.ftc.teamcode.robot2020;

import androidx.room.Room;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot2020.persistence.AppDatabase;


@Config
public class Robot
{
    /////////////
    //user data//
    /////////////
    //debug
    protected boolean debug_telemetry = true;
    protected boolean debug_dashboard = true; // turn this to false during competition
    protected boolean debug_methods = true;

    //user dashboard variable
    public static boolean emergencyStop = false;

    //database
    protected String dataBaseName = "FIRST_INSPIRE_2020";

    //sensor

    ///////////////////
    //other variables//
    ///////////////////
    //other classes
    public MotorConfig motorConfig;
    public Movement movement;
    public Vision vision;
    public Launcher launcher;
    public ComplexMovement complexMovement;
    public Position position;
    public Grabber grabber;

    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected BNO055IMU imu;
    protected LinearOpMode opMode;
    protected AppDatabase db;
    protected RobotUsage robotUsage;

    //other
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    TelemetryPacket packet = new TelemetryPacket();

    Robot(LinearOpMode opMode, boolean useDrive, boolean usePositionTracking, boolean logPositionTracking, boolean useComplexMovement, boolean useLauncher, boolean useGrabber, boolean useVuforia, boolean useOpenCV)
    {
        robotUsage = new RobotUsage(useDrive,usePositionTracking,logPositionTracking,useComplexMovement,useLauncher, useGrabber, useVuforia, useOpenCV);
        init(opMode, robotUsage);
    }

    Robot(LinearOpMode opMode, RobotUsage robotUsage)
    {
        init(opMode, robotUsage);
    }

    private void init(LinearOpMode opMode, RobotUsage robotUsage)
    {
        this.robotUsage = robotUsage;
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        motorConfig = new MotorConfig(this);
        position = new Position(this);

        if(robotUsage.useDrive) movement = new Movement(this);
        if(robotUsage.useOpenCV || robotUsage.useVuforia) vision = new Vision(this);
        if(robotUsage.useLauncher) launcher = new Launcher(this);
        if(robotUsage.useComplexMovement) complexMovement = new ComplexMovement(this);
        if(robotUsage.useGrabber) grabber = new Grabber(this);

        initHardware();
        if(robotUsage.useDrive || robotUsage.usePositionTracking) motorConfig.initDriveMotors();
        //if(useLauncher) motorConfig.initLauncherMotors();
        if(robotUsage.useOpenCV || robotUsage.useVuforia) vision.initAll();
        if(robotUsage.useGrabber) motorConfig.initGrabberMotors();
    }

    void initHardware()
    {
        ///////////
        //sensors//
        ///////////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())
        {
            delay(50);
            opMode.idle();
        }

        /////////////
        //dashboard//
        /////////////
        if(debug_dashboard) dashboard = FtcDashboard.getInstance();
        startTelemetry();

        /////////////
        //data base//
        /////////////
        db = Room.databaseBuilder(AppUtil.getDefContext(), AppDatabase.class, dataBaseName).build();
    }

    //------------------My Methods------------------//

    void start()
    {
        startTelemetry();
        position.start();
        if(robotUsage.useVuforia) vision.start();
    }

    /////////////
    //telemetry//
    /////////////

    void startTelemetry()
    {
        if(debug_dashboard)
        {
            packet = new TelemetryPacket();
        }
    }

    void addTelemetry(String cap, Object val)
    {
        if(debug_dashboard) packet.put(cap, val);
        if(debug_telemetry) telemetry.addData(cap, val);
    }

    void sendTelemetry()
    {
        if(debug_dashboard) dashboard.sendTelemetryPacket(packet);
        if(debug_telemetry) telemetry.update();
    }

    ////////////////
    //calculations//
    ////////////////
    double findAngleError(double currentAngle, double targetAngle)
    {
        targetAngle = scaleAngle(targetAngle);
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError = angleError - 360;
        } else if (angleError < -180) {
            angleError = angleError + 360;
        }
        return -angleError;
    }

    double scaleAngle(double angle)// scales an angle to fit in -180 to 180
    {
        if (angle > 180) { return angle - 360; }
        if (angle < -180) { return angle + 360; }
        return angle;
    }

    double getAngleFromXY(double X, double Y)
    {
        return Math.atan2(X, Y)*(180 / Math.PI);
    }

    double[] getXYFromAngle(double angle)
    {
        // deg to rad
        angle /= (180 / Math.PI);

        //rad to X,Y
        double[] XY = new double[2];
        XY[0] = Math.sin(angle);
        XY[1] = Math.cos(angle);
        double total = Math.abs(XY[0]) + Math.abs(XY[1]);
        XY[0] /= total;
        XY[1] /= total;

        return XY;
    }

    void delay(long ms){
        long last = System.currentTimeMillis();
        while(System.currentTimeMillis() - last < ms)
        {
            if(stop())break;
        }
    }

    boolean stop() { return emergencyStop || gamepad1.back || gamepad2.back || !opMode.opModeIsActive(); }
}

enum GamepadButtons
{
    dpadUP,
    dpadDOWN,
    dpadLEFT,
    dpadRIGHT,

    A,
    B,
    X,
    Y,

    START,
    BACK,
    leftBUMPER,
    rightBUMPER,

    leftJoyStickX,
    leftJoyStickY,
    leftJoyStickBUTTON,
    leftTRIGGER,

    rightJoyStickX,
    rightJoyStickY,
    rightJoyStickBUTTON,
    rightTRIGGER;

    boolean wasButtonPressed = false;
    long lastButtonRelease = System.currentTimeMillis();

    boolean getButtonHeld(Gamepad gamepad)
    {
        if(this == GamepadButtons.A) return gamepad.a;
        if(this == GamepadButtons.B) return gamepad.b;
        if(this == GamepadButtons.X) return gamepad.x;
        if(this == GamepadButtons.Y) return gamepad.y;

        if(this == GamepadButtons.dpadUP) return gamepad.dpad_up;
        if(this == GamepadButtons.dpadDOWN) return gamepad.dpad_down;
        if(this == GamepadButtons.dpadLEFT) return gamepad.dpad_left;
        if(this == GamepadButtons.dpadRIGHT) return gamepad.dpad_right;

        if(this == GamepadButtons.leftJoyStickBUTTON) return gamepad.left_stick_button;
        if(this == GamepadButtons.rightJoyStickBUTTON) return gamepad.right_stick_button;
        if(this == GamepadButtons.leftBUMPER) return gamepad.left_bumper;
        if(this == GamepadButtons.rightBUMPER) return gamepad.right_bumper;

        if(this == GamepadButtons.START) return gamepad.start;
        if(this == GamepadButtons.BACK) return gamepad.back;

        return false;
    }

    boolean getButtonHeld(Gamepad gamepad, int time)
    {
        if(getButtonHeld(gamepad))
        {
            return System.currentTimeMillis() - lastButtonRelease > time;
        }
        else lastButtonRelease = System.currentTimeMillis();
        return false;
    }

    boolean getButtonPressed(Gamepad gamepad)
    {
        if(getButtonHeld(gamepad))
        {
            if(!wasButtonPressed)
            {
                wasButtonPressed = true;
                return true;
            }
        }
        else wasButtonPressed = false;
        return false;
    }

    boolean getButtonReleased(Gamepad gamepad)
    {
        if(wasButtonPressed && !getButtonHeld(gamepad))
        {
            wasButtonPressed = false;
            return true;
        }
        return false;
    }

    float getSliderValue(Gamepad gamepad)
    {
        if(this == GamepadButtons.leftJoyStickX) return gamepad.left_stick_x;
        if(this == GamepadButtons.leftJoyStickY) return gamepad.left_stick_y;
        if(this == GamepadButtons.rightJoyStickX) return gamepad.right_stick_x;
        if(this == GamepadButtons.rightJoyStickY) return gamepad.right_stick_y;

        if(this == GamepadButtons.leftTRIGGER) return gamepad.left_trigger;
        if(this == GamepadButtons.rightTRIGGER) return gamepad.right_trigger;

        return 0;
    }
}

class PID
{
    PIDCoefficients PIDs;
    double maxClamp;
    double minClamp;
    
    double totalError;
    double lastError;
    double currentError;
    
    PID(PIDCoefficients PIDs, double minClamp, double maxClamp)
    {
        this.PIDs = PIDs;
        this.minClamp = minClamp;
        this.maxClamp = maxClamp;
    }
    
    void updatePID(double error)
    {
        lastError = currentError;
        currentError = error;
        totalError += error;
        
        double value = (error * PIDs.p) + (totalError * PIDs.i) + ((currentError - lastError) * PIDs.d);

        if((value > maxClamp || value < minClamp) && Math.signum(error) != Math.signum(totalError))
        {
            totalError = 0;
        }
    }
    
    double updatePIDAndReturnValue(double error)
    {
        updatePID(error);
        return returnValue();
    }
    
    double returnValue()
    {
        return Math.max(Math.min((currentError * PIDs.p) + (totalError * PIDs.i) + ((currentError - lastError) * PIDs.d), maxClamp), minClamp);
    }
    
    double returnUncappedValue()
    {
        return (currentError * PIDs.p) + (totalError * PIDs.i) + ((currentError - lastError) * PIDs.d);
    }
}

class RobotUsage
{
    boolean useDrive, usePositionTracking, logPositionTracking, useComplexMovement, useLauncher, useGrabber, useVuforia, useOpenCV = true;

    RobotUsage(){}
    RobotUsage(boolean useDrive, boolean usePositionTracking, boolean logPositionTracking, boolean useComplexMovement, boolean useLauncher, boolean useGrabber, boolean useVuforia, boolean useOpenCV)
    {
        this.useDrive = useDrive;
        this.usePositionTracking = usePositionTracking;
        this.logPositionTracking = logPositionTracking;
        this.useComplexMovement = useComplexMovement;
        this.useLauncher = useLauncher;
        this.useGrabber = useGrabber;
        this.useVuforia = useVuforia;
        this.useOpenCV = useOpenCV;
    }

    void setAllToValue(boolean value)
    {
        this.useDrive = value;
        this.usePositionTracking = value;
        this.logPositionTracking = value;
        this.useComplexMovement = value;
        this.useLauncher = value;
        this.useGrabber = value;
        this.useVuforia = value;
        this.useOpenCV = value;
    }
}