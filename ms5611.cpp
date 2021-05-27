/*
MIT License

Copyright (c) 2021 Ahmad Ziad Zain Aldeen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include "ms5611.h"

////    Public Functions    ////

/**
 * @brief Initilize MS5611 sensor
 * 
 * @param address I2C address of the device
 * @param ms5611Wire Wire pointer to use
 * @return uint8_t Zero on success
 */
uint8_t Ms5611::init(TwoWire *wire, uint8_t address)
{
    ms5611Wire = wire;
    _address   = address;

    ms5611Wire->begin();
    if(reset())
        return 1;

    delay(MS5611_TIME_RELOAD);

    // Datasheet P.10
    // PROM address = 1 0 1 0 x y z 0
    // where xyz can range from 0x00 to 0x07
    // but only 0x01 to 0x06 include calibration coefficients
    for(uint8_t reg = 1; reg < 7; reg++)
    {
        uint8_t _temp[2] = {0};
        if(readBytes(MS5611_CMD_READ_PROM_BASE | (reg << 1), _temp, 2))
            return 1;

        C[reg]  = _temp[0] << 8;     // combine the read bytes into one variable
        C[reg] |= _temp[1];
    }

    return 0;
}


/**
 * @brief Reset MS5611 device
 * 
 * @return uint8_t Zero on success
 */
uint8_t Ms5611::reset()
{
    return writeCommand(MS5611_CMD_RESET);
}


/**
 * @brief Update measurements and get readings from ADC
 * 
 * @param temperature pointer to store temperature
 * @param pressure pointer to store pressure
 * @return uint8_t Zero on success
 */
uint8_t Ms5611::getTempPress(double *temperature, double *pressure)
{
    // variables needed for calculation, see datasheet P.8
    uint32_t D1, D2;
    double  P, TEMP, dT, T2, OFF, SENS, OFF2, SENS2;

    // Read digital pressure value
    {
        uint8_t _temp[3] = {0};

        if(writeCommand(MS5611_CMD_CONVERT_D1 | _osr))
            return 1;
        
        delay(MS5611_TIME_ADC_CONVERT);     // wait for ADC conversion

        if(readBytes(MS5611_CMD_READ_ADC, _temp, 3))
            return 1;

        D1  = _temp[0] << 16;
        D1 |= _temp[1] << 8;
        D1 |= _temp[2];
    }

    // Read digital temperature value
    {
        uint8_t _temp[3] = {0};

        if(writeCommand(MS5611_CMD_CONVERT_D2 | _osr))
            return 1;
        
        delay(MS5611_TIME_ADC_CONVERT);     // wait for ADC conversion

        if(readBytes(MS5611_CMD_READ_ADC, _temp, 3))
            return 1;

        D2  = _temp[0] << 16;
        D2 |= _temp[1] << 8;
        D2 |= _temp[2];
    }

    // see datasheet P.8 for equations
    dT    = D2 - C[5] * 256;
    TEMP  = 2000 + dT * C[6] / 8388608;
    OFF   = C[2] * 65536 + C[4] * dT / 128;
    SENS  = C[1] * 32768 + C[3] * dT / 256;

    T2    = 0;
    OFF2  = 0;
    SENS2 = 0;

    // second order temperature compensation
    if(TEMP < 2000)
    {
        double tt;

        T2     = dT * dT / 2147483648;
        tt     = TEMP - 2000;
        OFF2   = 5 * tt * tt / 2;
        SENS2  = OFF2 / 2;

        if(TEMP < -1500)
        {
            tt      = TEMP + 1500;
            OFF2   += 7 * tt * tt;
            SENS2  += 11 * tt * tt / 2;
        }
    }

    TEMP -= T2;
    OFF  -= OFF2;
    SENS -= SENS2;

    P     = (D1 * SENS / 2097152 - OFF) / 32768;

    *temperature = (double) TEMP / 100;
    *pressure    = (double) P / 100;

    return 0;
}


/**
 * @brief Set new oversampeling rate, default = MS5611_OSR_1024
 * 
 * @param osr the desired OSR, use defines
 * @return uint8_t Zero on success
 */
uint8_t Ms5611::setOSR(uint8_t osr)
{
    switch(osr)
    {
        case MS5611_OSR_256:
        case MS5611_OSR_512:
        case MS5611_OSR_1024:
        case MS5611_OSR_2048:
        case MS5611_OSR_4096:
            _osr = osr;
            return 0;
        default:
            return 1;
    }
}


////    Private Functions    ////

/**
 * @brief Send a command to MS5611
 * 
 * @param command command byte
 * @return uint8_t Zero on success
 */
uint8_t Ms5611::writeCommand(uint8_t command)
{
    ms5611Wire->beginTransmission(_address);    // transmit to MS5611 address
    ms5611Wire->write(command);                 // write command 
    return ms5611Wire->endTransmission();       // end transmission
}


/**
 * @brief Read an array of bytes from PROM or ADC
 * 
 * @param reg register to read from
 * @param dest array to write on
 * @param len number of bytes to lead
 * @param retries number of retries before failure (default = 10)
 * @return Zero on success
 */
uint8_t Ms5611::readBytes(uint8_t reg, uint8_t *dest, uint32_t len, uint32_t retries)
{
    uint32_t time_outs = 0;

    writeCommand(reg);                          // send command to read
    ms5611Wire->requestFrom(_address, len);     // request data from MS5611

    while(ms5611Wire->available() != len)       // check if data is ready
    {
        time_outs++;

        if(time_outs > retries)
            return 1;

        delay(2);
    }

    for(unsigned int i = 0; i < len; i++)
        dest[i] = ms5611Wire->read();

    return 0;
}
