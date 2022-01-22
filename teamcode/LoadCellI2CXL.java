package org.firstinspires.ftc.teamcode;/*
 * Copyright (c) 2021 FTC team 16537 LOGICoyote
 *Take it if you wish
 *
 */

import static java.lang.Thread.currentThread;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cSensor(name = "LoadCell I2C", description = "LoadCell I2C 500g load cell from sparkfun", xmlTag = "LoadCellI2C")
public class LoadCellI2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private final byte DEFAULT_I2C_ADDR = (byte) 0xE0;
    private final byte CHANGE_I2C_ADDR_UNLOCK_1 = (byte) 0xAA;
    private final byte CHANGE_I2C_ADDR_UNLOCK_2 = (byte) 0xA5;
    private final byte CMD_PING = 0x51;
    private final byte NUM_RANGE_BYTES = 8;
    private final int DEFAULT_FLEX_PROPAGATION_DELAY_MS = 50;
    private final int FLEX_DELAY_MS_V2 = 50;


    private int lastFlex = -1;
    private long lastPingTime;

    //------------------------------------------------------------------------------------------
    // Constructors
    //------------------------------------------------------------------------------------------

    public LoadCellI2CXL(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(DEFAULT_I2C_ADDR));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected LoadCellI2CXL(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    //------------------------------------------------------------------------------------------
    // I2cDeviceSync methods
    //------------------------------------------------------------------------------------------

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "LoadCell I2C";
    }

    //------------------------------------------------------------------------------------------
    // Internal methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping
     */
    private void ping()
    {
        deviceClient.write8(0, CMD_PING, I2cWaitControl.WRITTEN);
    }

    /***
     * Reads the range of the last commanded
     * measurement from the sensor
     *
     * @return the range of the last commanded
     *         measurement
     */
    private int getFlex()
    {
        return 0;
       // return TypeConversion.byteArrayToShort(deviceClient.read(0, NUM_RANGE_BYTES));

    }
    public byte[] get_test()
    {
        return deviceClient.read(1, NUM_RANGE_BYTES);
    }

    //------------------------------------------------------------------------------------------
    // User sync methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping,
     * sleeps for the default sonar propagation
     * delay, and then reads the result of the
     * measurement just commanded
     *
     * @return the result of the range measurement
     *         that was performed
     */
    public int getFlexSync()
    {
        return getFlexSync(DEFAULT_FLEX_PROPAGATION_DELAY_MS);
    }

    /***
     * Commands the sensor to send out a ping,
     * sleeps for the time specified by the
     * sonarPropagationDelay, and the reads the
     * result of the measurement just commanded
     *
     * @param FlexPropagationDelay controls how long to sleep after sending
     *                              out the ping before trying to read the
     *                              result of the measurement. If the value
     *                              is too short, then the sound wave might not
     *                              make it back to the sensor before the read
     *                              is performed. If it is too long, the loop
     *                              time of your program will be slowed without
     *                              purpose
     *
     * @return the result of the range measurement
     *         that was performed
     */
    public int getFlexSync(int FlexPropagationDelay)
    {
        ping();
        try
        {
            sleep(FlexPropagationDelay);
        }
        catch (InterruptedException e)
        {
            currentThread().interrupt();
            return 0;
        }
        return getFlex();
    }
    public byte[] getFlexSynctest(int FlexPropagationDelay)
    {
        ping();
        try
        {
            sleep(FlexPropagationDelay);
        }
        catch (InterruptedException e)
        {
            currentThread().interrupt();
            return new byte[0];
        }
        return get_test();
    }
    //------------------------------------------------------------------------------------------
    // User async methods
    //------------------------------------------------------------------------------------------

    /***
     * Commands the sensor to send out a ping
     * if the default delay has expired, and
     * then reads the result of the measurement
     * just commanded. If the delay has NOT
     * expired, then the last value read from
     * the sensor will be returned instead.
     *
     * @return the result of the range measurement
     *         that was performed, or, if the default
     *         delay has not yet expired, the last
     *         value that was read from the sensor
     */
    public int getFlexAsync()
    {
        return getFlexAsync(DEFAULT_FLEX_PROPAGATION_DELAY_MS);
    }

    /***
     * Commands the sensor to send out a ping
     * if the delay specified has expired, and
     * then reads the result of the measurement
     * just commanded. If the delay has NOT
     * expired, then the last value read from
     * the sensor will be returned instead.
     *
     * @param FlexPropagationDelay controls how long to sleep after sending
     *                              out the ping before trying to read the
     *                              result of the measurement. If the value
     *                              is too short, then the sound wave might not
     *                              make it back to the sensor before the read
     *                              is performed. If it is too long, the sensor
     *                              will not report new data to your program
     *                              as fast as it could otherwise with a shorter
     *                              delay
     *
     * @return the result of the range measurement
     *         that was performed, or, if the delay
     *         specified by sonarPropagationDelay
     *         has not yet expired, the last value
     *         that was read from the sensor.
     */
    public int getFlexAsync(int FlexPropagationDelay)
    {
        long curTime = System.currentTimeMillis();

        if(((curTime - lastPingTime) > FlexPropagationDelay))
        {
            lastFlex = getFlex();
            ping();
            lastPingTime = System.currentTimeMillis();
        }

        return lastFlex;
    }

    //------------------------------------------------------------------------------------------
    // User I2C address changing methods
    //------------------------------------------------------------------------------------------

    /***
     * Sets the I2C address the SDK should use when attempting
     * to communicate with the sensor. Note that this method
     * will NOT actually change the address inside the sensor,
     * for that see writeI2cAddrToSensorEEPROM(byte addr).
     *
     * If the aforementioned method is used to change the
     * address inside the sensor, then you will need to
     * call this method after getting the object from the
     * hardwareMap, and pass in the same address as you
     * wrote to the sensor's EEPROM.
     *
     * Note that you will need to call this method once at the
     * beginning of each of your opmodes after you get the object
     * from the hwMap, whereas you only need to call the method
     * that writes to the sensor's EEPROM, once, ever. As in you
     * only need to use that when you are initially wiring up the
     * sensor.
     *
     * @param i2cAddr the I2C address the SDK should use when
     *                attempting to communicate with the sensor
     */
    public void setI2cAddress(I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    /***
     * Tells the sensor that it should operate on an I2C address
     * other than the default, and then writes that address to
     * the internal EEPROM so that it is retained across a power
     * cycle.
     *
     * You only need to call this method ONE TIME, EVER. (Unless
     * you want to change the address again, that is)
     *
     * Note that the sensor will only accept even numbered address
     * values. If an odd numbered address is sent, then the address
     * will be set to the next lowest even number. Also, the following
     * addresses are invalid, and if sent, the command will be ignored:
     *
     *      0x00
     *      0x50
     *      0xA4
     *      0xAA
     *
     * @param addr the I2C address the sensor should operate on
     */
    public void writeI2cAddrToSensorEEPROM(byte addr)
    {
        byte[] bytes = new byte[] {CHANGE_I2C_ADDR_UNLOCK_1, CHANGE_I2C_ADDR_UNLOCK_2, addr};
        deviceClient.write(0, bytes);
    }
}