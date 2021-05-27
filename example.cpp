#include <Arduino.h>
#include <ms5611.h>


Ms5611  baro;


void setup(void)
{
    Serial.begin(9600);
    while(!Serial);
    Serial.println("serial monitor started");

    // init MS5611
    while(baro.init())
    {
        Serial.println("Can't detect an MS5611 device");
        delay(1000);
    }

    Serial.println("MS5611 init OK");

    delay(1000);
}

void loop(void)
{
    double temp, pres;
    
    // read MS5611
    if(baro.getTempPress(&temp, &pres))
    {
        Serial.println("baro read failed");
        goto loopEnd;
    }

    Serial.printf("T = %lf C\tP = %lf mbar", temp, pres);

    // loop end lable
    loopEnd:
    delay(1000);
}
