package org.firstinspires.ftc.teamcode.robot2020;

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
@DeviceProperties(name = "DFR Range Sensor",
        description = "DFR Range Sensor",
        xmlTag = "DFRURM09RangeV2",
        compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class DFR304Range extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, DFR304Range.Parameters>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    public short getDistanceCm()
    {
        updateDistances(readShort(Register.DIST_H_INDEX));
        return getMax(distances);
        //return readShort(Register.DIST_H_INDEX);
    }

    public short getTemperatureC() { return (short)(readShort(Register.TEMP_H_INDEX)/10); }
    public float getDistanceIn()
    {
        return (getDistanceCm() / 2.54f);
    }
    public short getTemperatureF()
    {
        return (short)(getTemperatureC() * 1.8 + 32);
    }
    // in passive mode must force a temperature read
    public void measureRange()
    {
        writeByte(Register.CMD_INDEX, CMD_DISTANCE_MEASURE);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////
    protected void writeByte(final Register reg, Byte value)
    {
        byte[] txbuf =  {value};
        deviceClient.write(reg.bVal, txbuf);
    }

    protected byte readOneByte(Register reg)
    {
        byte[] raw = deviceClient.read(reg.bVal,1);
        return raw[0];
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    // CFG_INDEX
    // Bit7(control bit in ranging mode) 0: passive measurement, send ranging command once,
    // the module ranges the distance once and store the measured value into the distance register.
    // 1: automatic measurement mode, the module keeps ranging distance and updating the distance register all the time.
    // Bit6: save
    // Bit5-bit4(the maximum ranging distance bit that can be set)
    // 00:150CM(the ranging cycle is about 20MS)
    // 01:300CM(the ranging cycle is about 30MS)
    // 10:500CM(the ranging cycle is about 40MS)
    // Bits:   7654321
    // Mode  0bx000000
    // PASSIVE =  0 0x00  -- Must call measureRange() to measure range, then read it.
    // ACTIVE = 128 0x80 -- Continuously measures range, read any time.
    // ----Range----
    // 150CM 0b0000000 =  0 0x00
    // 300CM 0b0100000 = 16 0x10
    // 500CM 0b1000000 = 32 0z20
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
        ACTIVE(0x80);

        public int bVal;

        MeasureMode(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum MaxRange
    {
        CM150(0x00),
        CM300(0x10),
        CM500(0x20);

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
    public final static byte CMD_DISTANCE_MEASURE = (0x01);

    private static final int numOfLastValues = 3; //how many distances should it keep to get highest value from (a higher value will be more accurate but cause more lag, default = 3)
    private short[] distances = new short[numOfLastValues];

    public void initDistances()
    {
        for(int i = 0; i < distances.length; i++) distances[i] = 0;
    }

    private void updateDistances(short distance) {
        for(int i = distances.length - 1; i > 0; i--) {
            distances[i] = distances[i - 1];
        }
        distances[0] = distance;
    }



    private short getMax(short[] inputArray){
        short maxValue = inputArray[0];
        for(short i = 1; i < inputArray.length; i++){
            if(inputArray[i] > maxValue){
                maxValue = inputArray[i];
            }
        }
        return maxValue;
    }

    public DFR304Range(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true, new Parameters());
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
        initDistances();
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

        byte configSettings = (byte)(params.measureMode.bVal | params.maxRange.bVal);
        writeByte(Register.CFG_INDEX, configSettings);
        return readOneByte(Register.CFG_INDEX) == configSettings;
    }

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        MaxRange maxRange = MaxRange.CM150;
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