#include <Arduino.h>
#include <Wire.h>

// #define HW_PCF8575
#define COLONNE0 0 // Sortie
#define COLONNE1 1 // Sortie
#define COLONNE2 2 // Sortie

#define LIGNE0 3 // Entrée
#define LIGNE1 4 // Entrée
#define LIGNE2 5 // Entrée
#define LIGNE3 6 // Entrée

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
    pcf.pinMode(COLONNE0  , OUTPUT);
    pcf.pinMode(COLONNE1  , OUTPUT);
    pcf.pinMode(COLONNE2  , OUTPUT);
    pcf.pinMode(LIGNE0, INPUT_PULLUP);
    pcf.pinMode(LIGNE1, INPUT_PULLUP);
    pcf.pinMode(LIGNE2, INPUT_PULLUP);
    pcf.pinMode(LIGNE3, INPUT_PULLUP);

    pcf.digitalWrite(COLONNE0, LOW) ; 
    pcf.digitalWrite(COLONNE1, LOW) ; 
    pcf.digitalWrite(COLONNE2, LOW) ; 

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
        for (int i = COLONNE0 ; i <= COLONNE2 ; i ++)
        {
            pcf.digitalWrite(COLONNE0, (i == COLONNE0) ?  LOW : HIGH ); 
            pcf.digitalWrite(COLONNE1, (i == COLONNE1) ?  LOW : HIGH ); 
            pcf.digitalWrite(COLONNE2, (i == COLONNE2) ?  LOW : HIGH ); 
            for (int j  = LIGNE0 ; j <= LIGNE3 ; j ++)
            {
                bool etat = pcf.digitalRead (j) ; 
                //LOG("Test COLONNE %d - LIGNE %d : etat %s \r\n" , i - COLONNE0 , j - LIGNE0, (etat) ? "INACTIF" : "ACTIF" ) ; 

                if (  etat != true ) 
                {
                    KeyDetected = true ; 
                    Key = keyboard[i-COLONNE0][j-LIGNE0] ; 
                    LOG("Touche Appuyée : %c \r\n " , Key );
                    pcf.digitalWrite (LED, ! pcf.digitalRead(LED)) ; 


                }


            }
        }
        pcf.digitalWrite(COLONNE0, LOW) ; 
        pcf.digitalWrite(COLONNE1, LOW) ; 
        pcf.digitalWrite(COLONNE2, LOW) ; 

        /*Change la led du Nano a chaque IT detectée*/
        digitalWrite (LED_BUILTIN, digitalRead(LED_BUILTIN)) ; 

        attachInterrupt(digitalPinToInterrupt(IRQ_PIN), Interrupt_Callback, FALLING);
        bInterruptDetected = false ; 
    }
    
}
