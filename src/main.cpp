#include <Arduino.h>
#include <Wire.h>

// #define HW_PCF8575
#define LIGNE0 0 // Sortie
#define LIGNE1 1 // Sortie
#define LIGNE2 2 // Sortie

#define COLONNE0 3 // Entrée
#define COLONNE1 4 // Entrée
#define COLONNE2 5 // Entrée
#define COLONNE3 6 // Entrée

#define LED 7 // Sortie

#define IRQ_PIN 2 

#ifdef HW_PCF8575
#define ADDR_PCF_I2C 0x20
#include <Adafruit_PCF8575.h>
Adafruit_PCF8575 pcf;
#else
#include <Adafruit_PCF8574.h>
#define ADDR_PCF_I2C 0x38
Adafruit_PCF8574 pcf;
#endif


#define LOG(...) {char  log[127] = {0};sprintf (log, __VA_ARGS__);Serial.print (log);}


volatile bool bInterruptDetected = false ; 

char keyboard[3][4] = {
    {'#','9','6','3'},
    {'0','8','5','2'},
    {'*','7','4','1'}
} ;


void I2C_Scan(void)
{
    unsigned char error, address;
    unsigned int nDevices;
    /*Affichages des devices i2c*/
    LOG("Scanning...\r\n");
    nDevices = 0;
    for (address = 8; address < 127; address++)
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        if (error == 0)
        {
            LOG("I2C device found at address %02x \r\n " , address);

            nDevices++;
        }
        else if (error == 4)
        {
            LOG("Unknow error at address %02x \r\n", address);
        }
    }
    if (nDevices == 0)
    {
        LOG("No I2C devices found\n");
    }
    else
    {
        LOG("done\n");
    }
}


void Interrupt_Callback(void)
{
    bInterruptDetected = true ; 
}

void setup()
{
    /* Init Serie a 115200 baud */
    while (!Serial)
    {
        delay(10);
    }
    Serial.begin(115200);

    /*Init Nano Led*/
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    /* Init I2C */
    Wire.begin();
    /*Scan I2C */
    I2C_Scan();

    /*Detection Pcf 857x**/
    if (!pcf.begin(ADDR_PCF_I2C, &Wire))
    {
        LOG("Couldn't find PCF857x, programme arreté \r\n");

        while (1)
        {
            ;
        }
    }
    else
    {
        LOG("init PCF857x\r\n");
    }

    /* Init Entrée Sortie PCF857x*/
    pcf.pinMode(LIGNE0  , OUTPUT);
    pcf.pinMode(LIGNE1  , OUTPUT);
    pcf.pinMode(LIGNE2  , OUTPUT);
    pcf.pinMode(COLONNE0, INPUT_PULLUP);
    pcf.pinMode(COLONNE1, INPUT_PULLUP);
    pcf.pinMode(COLONNE2, INPUT_PULLUP);
    pcf.pinMode(COLONNE3, INPUT_PULLUP);

    pcf.digitalWrite(LIGNE0, LOW) ; 
    pcf.digitalWrite(LIGNE1, LOW) ; 
    pcf.digitalWrite(LIGNE2, LOW) ; 

    pcf.digitalWrite(LED,HIGH); /*Eteint led PCF */


    /*Init interruption */
    pinMode(IRQ_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN), Interrupt_Callback, FALLING);
    bInterruptDetected = false ; 
}



void loop()
{
    bool KeyDetected = false ; 
    char  Key = 0; 
    if ( bInterruptDetected)
    {

        LOG("Interruption Detecté\r\n") ; 
        detachInterrupt (digitalPinToInterrupt(IRQ_PIN));
        bInterruptDetected = false ; 
        for (int i = LIGNE0 ; i <= LIGNE2 ; i ++)
        {
            pcf.digitalWrite(LIGNE0, (i == LIGNE0) ?  LOW : HIGH ); 
            pcf.digitalWrite(LIGNE1, (i == LIGNE1) ?  LOW : HIGH ); 
            pcf.digitalWrite(LIGNE2, (i == LIGNE2) ?  LOW : HIGH ); 
            for (int j  = COLONNE0 ; j <= COLONNE3 ; j ++)
            {
                bool etat = pcf.digitalRead (j) ; 
                //LOG("Test LINE %d - COLONNE %d : etat %s \r\n" , i - LIGNE0 , j - COLONNE0, (etat) ? "INACTIF" : "ACTIF" ) ; 

                if (  etat != true ) 
                {
                    KeyDetected = true ; 
                    Key = keyboard[i-LIGNE0][j-COLONNE0] ; 
                    LOG("Touche Appuyée : %c \r\n " , Key );
                    pcf.digitalWrite (LED, ! pcf.digitalRead(LED)) ; 


                }


            }
        }
        pcf.digitalWrite(LIGNE0, LOW) ; 
        pcf.digitalWrite(LIGNE1, LOW) ; 
        pcf.digitalWrite(LIGNE2, LOW) ; 

        /*Change la led du Nano a chaque IT detectée*/
        digitalWrite (LED_BUILTIN, digitalRead(LED_BUILTIN)) ; 

        attachInterrupt(digitalPinToInterrupt(IRQ_PIN), Interrupt_Callback, FALLING);
        bInterruptDetected = false ; 
    }
    
}
