package org.firstinspires.ftc.teamcode.holo;

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;

import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
import androidx.annotation.NonNull;

@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "DFRobot Temp/Range Sensor V2",
        description = "DFRobot Temp/Range Sensor V2",
        xmlTag = "DFRobotURM09RangeV2",
        compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class URM09ULTRA extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, URM09ULTRA.Parameters>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public short getDistanceCm()
    {
        return readShort(Register.DIST_H_INDEX);
    }

    public short getTemperatureC()
    {
        return (short)(readShort(Register.TEMP_H_INDEX)/10);
    }

    public short getDistanceIn()
    {
        return (short)(getDistanceCm() / 2.54);
    }

    public short getTemperatureF()
    {
        return (short)(getTemperatureC() * 1.8 + 32);
    }

    // in passive mode must force a temperature read
    public void measureRange()
    {
        writeShort(Register.CMD_INDEX, CMD_DISTANCE_MEASURE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        short theval = TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
        return theval;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {
        FIRST(0x00),
        PID_INDEX(0x01),
        VERSION_INDEX(0x02),
        DIST_H_INDEX(0x03),
        DIST_L_INDEX(0x04),
        TEMP_H_INDEX(0x05),
        TEMP_L_INDEX(0x06),
        CFG_INDEX(0x07),
        CMD_INDEX(0x08),
        LAST(CMD_INDEX.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum MeasureMode
    {
        PASSIVE(0x00),
        ACTIVE(0x01);

        public int bVal;

        MeasureMode(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum MaxRange
    {
        CM150(0x00),
        CM300(0x01),
        CM500(0x20); // might be 0x20

        public int bVal;

        MaxRange(int bVal)
        {
            this.bVal = bVal;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x11);
    public final static short CMD_DISTANCE_MEASURE = (0x01);

    public URM09ULTRA(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true, new Parameters());

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }


    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters params)
    {
        this.parameters = params.clone();
        deviceClient.setI2cAddress(params.i2cAddr);

        int configSettings = params.measureMode.bVal | params.maxRange.bVal;

        //the measurement mode is set to active mode, measurement range is set to 500CM.
        writeShort(Register.CFG_INDEX, (short) configSettings);

        return readShort(Register.CFG_INDEX) == configSettings;
    }

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        // All settings available
        MaxRange maxRange = MaxRange.CM500;
        MeasureMode measureMode = MeasureMode.PASSIVE;

        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "Gravity: URM09 Ultrasonic Sensor (I²C)";
    }
}
