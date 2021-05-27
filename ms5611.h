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


#ifndef __MS5611_H__
#define __MS5611_H__


////    Includes    ////
#include <Arduino.h>
#include <Wire.h>


////    Defines    ////
// Device address
#define MS5611_ADDRESS              0x77

// I2C commands                             // Datasheet P.10
#define MS5611_CMD_RESET            0x1E
#define MS5611_CMD_CONVERT_D1       0x40
#define MS5611_CMD_CONVERT_D2       0x50
#define MS5611_CMD_READ_ADC         0x00
#define MS5611_CMD_READ_PROM_BASE   0xA0

// Timings ceiling to nearest 5 ms          // Datasheet P.3,10
#define MS5611_TIME_RELOAD          5
#define MS5611_TIME_ADC_CONVERT     10

// OSR (Oversampeling Ratios)
#define MS5611_OSR_256              0x00
#define MS5611_OSR_512              0x02
#define MS5611_OSR_1024             0x04
#define MS5611_OSR_2048             0x06
#define MS5611_OSR_4096             0x08


////    Ms5611 class    ////
class Ms5611
{
    public:
        Ms5611() {};
        ~Ms5611() {};

        uint8_t init                    (TwoWire *wire = &Wire, uint8_t address = MS5611_ADDRESS);
        uint8_t reset                   ();

        uint8_t getTempPress            (double *temperature, double *pressure);

        uint8_t setOSR                  (uint8_t osr);

    private:
        uint8_t    _address, _osr = MS5611_OSR_256;
        uint32_t   C[7] = {0};          // PROM calibration coefficents
        TwoWire   *ms5611Wire;

        uint8_t writeCommand            (uint8_t command);
        uint8_t readBytes               (uint8_t reg, uint8_t *dest, uint32_t len, uint32_t retries = 10);
};

#endif
