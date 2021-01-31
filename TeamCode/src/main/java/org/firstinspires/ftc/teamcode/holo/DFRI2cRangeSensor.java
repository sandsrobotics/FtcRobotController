package org.firstinspires.ftc.teamcode.holo;

/*
Copyright (c) 2016-17 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cAddrConfig;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

/**
 * {@link DFRI2cRangeSensor} implements support for the DFR ultrasonic/tempcombo
 * range sensor.
 *
 * @see <a href="https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304#target_8">MR Range Sensor</a>
 */
@SuppressWarnings("WeakerAccess")
@I2cDeviceType
@DeviceProperties(name = "DFRobot Range Sensor V2",
        description = "DFRobot Range Sensor V2",
        xmlTag = "DFRI2cRangeSensor",
        compatibleControlSystems = ControlSystem.REV_HUB, builtIn = true)
public class DFRI2cRangeSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> implements DistanceSensor, I2cAddrConfig
{
    //----------------------------------------------------------------------------------------------
    // Constants
    //----------------------------------------------------------------------------------------------

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create8bit(0x11);

    public enum Register
    {
        FIRST(0),
        PID_INDEX(0x01),
        VERSION_INDEX(0x02),
        DIST_H_INDEX(0x03),
        DIST_L_INDEX(0x04),
        TEMP_H_INDEX(0x05),
        TEMP_L_INDEX(0x06),
        CFG_INDEX(0x07),
        CMD_INDEX(0x08),
        LAST(CMD_INDEX.bVal),
        UNKNOWN(-1);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    protected static final double apiLevelMin = 0.0;
    protected static final double apiLevelMax = 1.0;
    protected static final int cmUltrasonicMax = 255;

    //----------------------------------------------------------------------------------------------
    // Construction
    //----------------------------------------------------------------------------------------------

    public DFRI2cRangeSensor(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        byte configSettings = 0x00 | 0x20;

        //the measurement mode is set to active mode, measurement range is set to 500CM.
        write8(Register.CFG_INDEX, configSettings);
        //int config = readUnsignedByte(Register.CFG_INDEX);
        //return config == configSettings;
        return true;
    }

    //----------------------------------------------------------------------------------------------
    // DistanceSensor
    //----------------------------------------------------------------------------------------------

    /**
     *
     * @param unit  the unit of distance in which the result should be returned
     * @return      the currently measured distance in the indicated units
     */
    @Override public double getDistance(DistanceUnit unit)
    {
        double cm = cmUltrasonic();
        if (cm == cmUltrasonicMax)
        {
            return DistanceSensor.distanceOutOfRange;
        }

        return unit.fromUnit(DistanceUnit.CM, cm);
    }

    public double cmUltrasonic()
    {
        return rawUltrasonic();
    }

    //----------------------------------------------------------------------------------------------
    // Raw sensor data
    //----------------------------------------------------------------------------------------------

    /**
     * Returns the raw reading on the ultrasonic sensor
     * @return the raw reading on the ultrasonic sensor
     */
    public int rawUltrasonic()
    {
        return readUnsignedByte(Register.DIST_H_INDEX);
    }

    //----------------------------------------------------------------------------------------------
    // I2cAddressConfig
    //----------------------------------------------------------------------------------------------

    @Override public void setI2cAddress(I2cAddr newAddress)
    {
        this.deviceClient.setI2cAddress(newAddress);
    }

    @Override public I2cAddr getI2cAddress()
    {
        return this.deviceClient.getI2cAddress();
    }

    //----------------------------------------------------------------------------------------------
    // HardwareDevice
    //----------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Unknown;
    }

    @Override public String getDeviceName()
    {
        return String.format(Locale.getDefault(), "DF Robotics Range Sensor V2");
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    public byte read8(Register reg)
    {
        return this.deviceClient.read8(reg.bVal);
    }

    public void write8(Register reg, byte value)
    {
        this.write8(reg, value, I2cWaitControl.NONE);
    }
    public void write8(Register reg, byte value, I2cWaitControl waitControl)
    {
        this.deviceClient.write8(reg.bVal, value, waitControl);
    }

    protected int readUnsignedByte(Register reg)
    {
        return TypeConversion.unsignedByteToInt(this.read8(reg));
    }

}
