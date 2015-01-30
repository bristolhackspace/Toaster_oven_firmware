/*

  Toaster Oven SMT soldering control
  
  Adrian Bowyer
  
  2 November 2011
  
  Licence: GPL
  
*/

#include <avr/pgmspace.h>

#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>

const int heatPin =  13;     // the number of the LED pin.  This also controls the heater
int heatState = LOW;         // heatState used to set the LED and heater
long previousMillis = 0;     // will store last time LED/heater was updated
const long interval = 1000;  // interval at which to sample temperature (milliseconds)
const int tempPin = 0;       // Analogue pin for temperature reading
long time = 0;               // Time since start in seconds
bool done=false;             // Flag to indicate that the process has finished

// pin 8 - Serial clock out (SCLK)
// pin 7 - Serial data out (DIN)
// pin 6 - Data/Command select (D/C)
// pin 5 - LCD chip select (CS)
// pin 4 - LCD reset (RST)
Adafruit_PCD8544 display = Adafruit_PCD8544(4, 5, 6, 7, 8);

#define PGM_RD_W(x)  (short)pgm_read_word(&x)

#define OVERSAMPLENR 1

//100k bed thermistor

const short temptable_1[][2] PROGMEM = {
{ 23*OVERSAMPLENR , 300 },
{ 25*OVERSAMPLENR , 295 },
{ 27*OVERSAMPLENR , 290 },
{ 28*OVERSAMPLENR , 285 },
{ 31*OVERSAMPLENR , 280 },
{ 33*OVERSAMPLENR , 275 },
{ 35*OVERSAMPLENR , 270 },
{ 38*OVERSAMPLENR , 265 },
{ 41*OVERSAMPLENR , 260 },
{ 44*OVERSAMPLENR , 255 },
{ 48*OVERSAMPLENR , 250 },
{ 52*OVERSAMPLENR , 245 },
{ 56*OVERSAMPLENR , 240 },
{ 61*OVERSAMPLENR , 235 },
{ 66*OVERSAMPLENR , 230 },
{ 71*OVERSAMPLENR , 225 },
{ 78*OVERSAMPLENR , 220 },
{ 84*OVERSAMPLENR , 215 },
{ 92*OVERSAMPLENR , 210 },
{ 100*OVERSAMPLENR , 205 },
{ 109*OVERSAMPLENR , 200 },
{ 120*OVERSAMPLENR , 195 },
{ 131*OVERSAMPLENR , 190 },
{ 143*OVERSAMPLENR , 185 },
{ 156*OVERSAMPLENR , 180 },
{ 171*OVERSAMPLENR , 175 },
{ 187*OVERSAMPLENR , 170 },
{ 205*OVERSAMPLENR , 165 },
{ 224*OVERSAMPLENR , 160 },
{ 245*OVERSAMPLENR , 155 },
{ 268*OVERSAMPLENR , 150 },
{ 293*OVERSAMPLENR , 145 },
{ 320*OVERSAMPLENR , 140 },
{ 348*OVERSAMPLENR , 135 },
{ 379*OVERSAMPLENR , 130 },
{ 411*OVERSAMPLENR , 125 },
{ 445*OVERSAMPLENR , 120 },
{ 480*OVERSAMPLENR , 115 },
{ 516*OVERSAMPLENR , 110 },
{ 553*OVERSAMPLENR , 105 },
{ 591*OVERSAMPLENR , 100 },
{ 628*OVERSAMPLENR , 95 },
{ 665*OVERSAMPLENR , 90 },
{ 702*OVERSAMPLENR , 85 },
{ 737*OVERSAMPLENR , 80 },
{ 770*OVERSAMPLENR , 75 },
{ 801*OVERSAMPLENR , 70 },
{ 830*OVERSAMPLENR , 65 },
{ 857*OVERSAMPLENR , 60 },
{ 881*OVERSAMPLENR , 55 },
{ 903*OVERSAMPLENR , 50 },
{ 922*OVERSAMPLENR , 45 },
{ 939*OVERSAMPLENR , 40 },
{ 954*OVERSAMPLENR , 35 },
{ 966*OVERSAMPLENR , 30 },
{ 977*OVERSAMPLENR , 25 },
{ 985*OVERSAMPLENR , 20 },
{ 993*OVERSAMPLENR , 15 },
{ 999*OVERSAMPLENR , 10 },
{ 1004*OVERSAMPLENR , 5 },
{ 1008*OVERSAMPLENR , 0 } //safety
};


// The temperature/time profile as {secs, temp}
// This profile is linearly interpolated to get the required temperature at any time.
// PLEN is the number of entries
#define PLEN 8
long profile[PLEN][2] = { {0, 15}, {5, 50}, {90, 120}, {120, 150}, {240, 180}, {280, 225}, {320, 183}, {350, 0} };

// Linearly interpolate the profile for the current time in secs, t

int target(long t)
{
  if(t <= profile[0][0])
   return profile[0][1];
  if(t >= profile[PLEN-1][0])
  {
   done = true; // We are off the end of the time curve
   return profile[PLEN-1][1];
  }
  for(int i = 1; i < PLEN-1; i++)
  {
     if(t <= profile[i][0])
       return (int)(profile[i-1][1] + ((t - profile[i-1][0])*(profile[i][1] - profile[i-1][1]))/
         (profile[i][0] - profile[i-1][0]));
  }
  return 0;
}

// Function to read thermistor table from Marlin

float analog2temp(int raw) {
    float celsius = 0;
    byte i;
    short (*tt)[][2] = (short (*)[][2])(temptable_1);

//    raw = (1023 * OVERSAMPLENR) - raw;
    for (i=1; i<(sizeof(temptable_1)/sizeof(*temptable_1)); i++)
    {
      if (PGM_RD_W((*tt)[i][0]) > raw)
      {
        celsius = PGM_RD_W((*tt)[i-1][1]) +
          (raw - PGM_RD_W((*tt)[i-1][0])) *
          (float)(PGM_RD_W((*tt)[i][1]) - PGM_RD_W((*tt)[i-1][1])) /
          (float)(PGM_RD_W((*tt)[i][0]) - PGM_RD_W((*tt)[i-1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == (sizeof(temptable_1)/sizeof(*temptable_1))) celsius = PGM_RD_W((*tt)[i-1][1]);

    return celsius;
}



// Measure the actual temperature from the thermocouple

int rawreading()
{
  return ( analogRead(tempPin) );
}

int temperature()
{
// thermocouple
// return ( 5.0 * analogRead(tempPin) * 100.0) / 1024.0;

// thermistor raw
// return ( analogRead(tempPin) );

// termistor lookup table
return (int)analog2temp(analogRead(tempPin));
}

// Get the show on the road

void setup() {
  display.begin();
  display.setContrast(50);
  display.display(); // show splashscreen
  pinMode(2, OUTPUT);
  digitalWrite(2,HIGH); //turn on backlight
  delay(1000);
  display.clearDisplay();   // clears the screen and buffer
  
  pinMode(heatPin, OUTPUT); 
  pinMode(tempPin, INPUT);  
  Serial.begin(9600);
  Serial.println("\n\n\nTime, target, raw, temp");
  //-----LCD Display-------------
  display.setTextColor(BLACK);
  display.setTextSize(1);
  display.println("Time Target Raw Temp");
  display.display();
  done = false;
}

// Go round and round

void loop()
{
  int t;
  unsigned long currentMillis = millis();
 
  if(currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis; // set next time 
    
    // Get the actual temperature
    
    t = temperature();
    
    // One second has passed
    
    time++;   
    
    // Find the target temperature
    
    int tg = target(time);
    
    // Simple bang-bang temperature control
    
    if (t < tg)
    {
      heatState = HIGH;
    } else
    {
      heatState = LOW;
    }

    // Turn the heater on or off (and the LED)
    digitalWrite(heatPin, heatState);
    
    // Keep the user amused
    if(done)
    {
      Serial.print((char)0x07);  // Bell to wake the user up...
      Serial.print((char)0x07);
      display.clearDisplay();
      display.print("FINISHED");
      display.display();
      Serial.print("FINISHED");
    }
    //--------LCD output----------
    display.clearDisplay(); 
    display.println("Time");
    display.println(time);
    display.println("Target Temp"); 
    display.println(tg);
    display.println("Actual Temp");
    display.print(t);
    display.display(); 
    //-------Serial output---------
    Serial.print(time);
    Serial.print(",");
    Serial.print(tg);
    Serial.print(",");
    Serial.print(rawreading());
    Serial.print(",");
    Serial.println(t);    
    
  }
}
