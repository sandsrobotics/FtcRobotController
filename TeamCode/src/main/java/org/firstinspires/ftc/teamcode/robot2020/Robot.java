package org.firstinspires.ftc.teamcode.robot2020;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

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

    //objects
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected FtcDashboard dashboard;
    protected BNO055IMU imu;
    protected LinearOpMode opMode;

    //other
    protected Gamepad gamepad1;
    protected Gamepad gamepad2;
    TelemetryPacket packet;

    Robot(LinearOpMode opMode, boolean useDrive, boolean useComplexMovement, boolean useLauncher, boolean useVuforia, boolean useOpenCV)
    {
        motorConfig = new MotorConfig(this);
        if(useDrive)movement = new Movement(this);
        if(useOpenCV || useVuforia) vision = new Vision(this);
        if(useLauncher) launcher = new Launcher(this);
        if(useComplexMovement) complexMovement = new ComplexMovement(this);
        position = new Position(this);

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        initHardware();
        if(useDrive || useComplexMovement) motorConfig.initDriveMotors();
        if(useLauncher) motorConfig.initLauncherMotors();
        if(useOpenCV || useVuforia) vision.initAll(useVuforia, useOpenCV);
    }

    void initHardware()
    {
        ///////
        //imu//
        ///////
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!opMode.isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            opMode.idle();
        }

        imu.startAccelerationIntegration(new org.firstinspires.ftc.robotcore.external.navigation.Position(), new Velocity(), 10);

        /////////////
        //dashboard//
        /////////////
        if(debug_dashboard) dashboard = FtcDashboard.getInstance();

    }

    //------------------My Methods------------------//
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
        if (targetAngle > 180) {
            targetAngle = targetAngle - 360;
        } else if (targetAngle < -180) {
            targetAngle = targetAngle + 360;
        }
        double angleError = currentAngle - targetAngle;
        if (angleError > 180) {
            angleError = angleError - 360;
        } else if (angleError < -180) {
            angleError = angleError + 360;
        }
        return -angleError;
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

    boolean allSameValues(double[] values)
    {
        double lastValueSign = Math.signum(values[0]);
        for(double value:values){if(lastValueSign != Math.signum(value)) return false;}
        return true;
    }

    double maxAbsoluteValue(double[] values)
    {
        double max = 0;
        for(double value:values)if(Math.abs(value) > max) max = Math.abs(value);
        return max;
    }

    void addDoubleArrays(double[] main, double[] second)
    {
        if(main.length == second.length) { for(int i = 0; i < main.length; i++) main[i] += second[i]; }
    }

    boolean stop() { return emergencyStop || gamepad1.back || gamepad2.back || !opMode.opModeIsActive(); }

    void delay(long ms)
    {
        for(long i = 0; i < ms; i++)
        {
            sleep(1);
            if(stop())break;
        }
    }
}