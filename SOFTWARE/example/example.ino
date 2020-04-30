/*Wiring to your Arduino
 example.ino
 this example gives differential voltage across the AN0 And AN1 in mV
 Hooking-up with the Arduino
----------------------
|ads1262 pin label| Pin Function         |Arduino Connection|
|-----------------|:--------------------:|-----------------:|
| DRDY            | Data ready Output pin|  D3              |             
| MISO            | Slave Out            |  D14             |
| MOSI            | Slave In             |  D16             |
| SCLK            | Serial Clock         |  D15             |
| CS              | Chip Select          |  D13             |
| START           | Start Conversion     |  D1              | 
| PWDN            | Power Down/Reset     |  D2              |
| DVDD            | Digital VDD          |  +5V             |
| DGND            | Digital Gnd          |  Gnd             |
| AN0-AN9         | Analog Input         |  Analog Input    |
| AINCOM          | Analog input common  |                  |
| AVDD            | Analog VDD           |  -               |
| AGND            | Analog Gnd           |  -               |
-----------------------------------------------------------------
*/
#include <SPI.h>
//#include <SoftwareSerial.h>
#include <math.h>
#include <ads1262.h>

#define PGA 1
#define VREF 3.40
#define VFSR VREF/PGA             
#define FSR (((long int)1<<23)-1)  

ads1262 PC_ADS1262;                     // class

float volt_V=0;
float volt_mV=0;
volatile int i;
volatile char SPI_RX_Buff[10];
volatile long ads1262_rx_Data[10];
volatile static int SPI_RX_Buff_Count = 0;
volatile char *SPI_RX_Buff_Ptr;
volatile int Responsebyte = false;
volatile signed long sads1262Count = 0;
volatile signed long uads1262Count=0;
double resolution;
float readADC(byte inpsel);

void setup() 
{
  pinMode(8,OUTPUT);
  Serial.begin(115200);
  digitalWrite(8,HIGH);

  // initalize the  data ready and chip select pins:
  pinMode(ADS1262_DRDY_PIN, INPUT);                  //data ready input line
  pinMode(ADS1262_CS_PIN, OUTPUT);                   //chip enable output line
  pinMode(ADS1262_START_PIN, OUTPUT);               // start
  pinMode(ADS1262_PWDN_PIN, OUTPUT);                // Power down output

  digitalWrite(8,LOW);
  PC_ADS1262.ads1262_Init();                      // initialise ads1262
}

void loop ()
{
  float pwr;
  byte channel;
  channel = 1;
  //pwr=abs(readADC(0x0a+channel*16));
  pwr=readADC(0+channel*16);
  Serial.print ("Data: ");
  Serial.print (pwr);
  Serial.println (" mV");
  delay (200);
}

float readADC(byte inpsel)
{
  volatile int i,data;

  PC_ADS1262.ads1262_Reg_Write(INPMUX, inpsel); //Ch 1 enabled, gain 6, connected to electrode in
  //delay(100); 

  while (1)
  {
    //Serial.println ("Reading");
    if((digitalRead(ADS1262_DRDY_PIN)) == LOW)               // monitor Data ready(DRDY pin)
    {  
      SPI_RX_Buff_Ptr = PC_ADS1262.ads1262_Read_Data();      // read 6 bytes conversion register
      Responsebyte = true ; 
    }

    if(Responsebyte == true)
    {
      //Serial.println ("Raw Data");
      for(i = 0; i <5; i++)
      {
        SPI_RX_Buff[SPI_RX_Buff_Count] = *(SPI_RX_Buff_Ptr + i); 
        //Serial.println ((unsigned char)SPI_RX_Buff[SPI_RX_Buff_Count]);
        SPI_RX_Buff_Count++;
      }
      Responsebyte = false;
    }

    if(SPI_RX_Buff_Count >= 5)
    {
      ads1262_rx_Data[0]= (unsigned char)SPI_RX_Buff[1];  // read 4 bytes adc count
      ads1262_rx_Data[1]= (unsigned char)SPI_RX_Buff[2];
      ads1262_rx_Data[2]= (unsigned char)SPI_RX_Buff[3];
      ads1262_rx_Data[3]= (unsigned char)SPI_RX_Buff[4];
      uads1262Count = (signed long) (((unsigned long)ads1262_rx_Data[0]<<24)|((unsigned long)ads1262_rx_Data[1]<<16)|(ads1262_rx_Data[2]<<8)|ads1262_rx_Data[3]);//get the raw 32-bit adc count out by shifting
      sads1262Count = (signed long) (uads1262Count);      // get signed value
      resolution = (double)((double)VREF/pow(2,31));       //resolution= Vref/(2^n-1) , Vref=2.5, n=no of bits
      // Serial.print(resolution,15);
      volt_V      = (resolution)*(float)sads1262Count;     // voltage = resolution * adc count
      volt_mV   =   volt_V*1000;  // voltage in mV
      SPI_RX_Buff_Count = 0;
      return volt_mV;
    }
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

