#include <FastLED.h>

/** BASIC CONFIGURATION  **/

//The amount of LEDs in the setup
#define N_PIXELS 106
#define N_PIXELS_HALF (N_PIXELS / 2)
#define N_PIXELS_QUARTER (N_PIXELS / 4)
#define N_PIXELS_2 192
#define N_PIXELS_HALF_2 (N_PIXELS_2 / 2)
#define N_PIXELS_QUARTER_2 (N_PIXELS_2 / 4)
 
//The pin that controls the LEDs
#define LED_PIN 5
#define LED_PIN_2 7
//The pin that we read sensor values form
#define MIC_PIN 0

//Confirmed microphone low value, and max value
#define MIC_LOW 400.0
#define MIC_HIGH 700.0
/** Other macros */
//How many previous sensor values effects the operating average?
#define AVGLEN 5
//How many previous sensor values decides if we are on a peak/HIGH (e.g. in a song)
#define LONG_SECTOR 20

//Mneumonics
#define HIGH 3
#define NORMAL 2

//How long do we keep the "current average" sound, before restarting the measuring
#define MSECS 30 * 1000
#define CYCLES MSECS / DELAY

/*Sometimes readings are wrong or strange. How much is a reading allowed
to deviate from the average to not be discarded? **/
#define DEV_THRESH 0.8

//Arduino loop delay
#define DELAY 1

float fscale( float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve);
void insert(int val, int *avgs, int len);
int compute_average(int *avgs, int len);
void visualize_music();
void led_mode2();
void led_mode3();
void led_mode4();
void led_mode5();
void led_off();


// constants used here to set pin numbers:
int buttonPin1 = A2;     // the number of the pushbutton pins
int buttonPin2 = A4;     // the number of the pushbutton pins
int buttonPin3 = A6;     // the number of the pushbutton pins
int buttonPin4 = A8;     // the number of the pushbutton pins
int buttonPin5 = A10;     // the number of the pushbutton pins
int buttonPin6 = A12;     // the number of the pushbutton pins

// Variables will change:

int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState1 = 0;         // initialize state of the button pins to OFF
int buttonState2 = 0; 
int buttonState3 = 0; 
int buttonState4 = 0; 
int buttonState5 = 0; 
int buttonState6 = 0; 
//int lastButtonState = 1;

//int switchPin = A2;
int ledMode = 0;  // assume switch closed because of pull-down resistor
const unsigned long debounceTime = 1000;  // milliseconds

//How many LEDs to we display
int height = N_PIXELS;
int height_2 = N_PIXELS_2;


/*Not really used yet. Thought to be able to switch between sound reactive
mode, and general gradient pulsing/static color*/
int mode = 0;

//Showing different colors based on the mode.
int songmode = NORMAL;

//Average sound measurement the last CYCLES
unsigned long song_avg;

//The amount of iterations since the song_avg was reset
int iter = 0;

//The speed the LEDs fade to black if not relit
float fade_scale = 1.2;

//Led array
CRGB leds[N_PIXELS];
CRGB leds_2[N_PIXELS_2];


/*Short sound avg used to "normalize" the input values.
We use the short average instead of using the sensor input directly */
int avgs[AVGLEN] = {-1};

//Longer sound avg
int long_avg[LONG_SECTOR] = {-1};\

//for falling dot
byte
  peak      = 0,      // Used for falling dot
  peak_2      = 0,      // Used for falling dot 2
  dotCount  = 0,      // Frame counter for delaying dot-falling speed
  dotCount_2  = 0;      // Frame counter for delaying dot-falling speed 2

#define PEAK_FALL 2  // Rate of peak falling dot
#define PEAK_FALL_split 6  // Rate of peak falling dot
#define PEAK_FALL_2 1  // Rate of peak falling dot
#define PEAK_FALL_split_2 3  // Rate of peak falling dot

//variables pulled out to make global
int sensor_value, mapped, avg, longavg;

//Keeping track how often, and how long times we hit a certain mode
struct time_keeping {
  unsigned long times_start;
  short times;
};

//How much to increment or decrement each color every cycle
struct color {
  int r;
  int g;
  int b;
};

struct time_keeping high;
struct color Color; 

CRGBPalette16 currentPalette;
CRGBPalette16 targetPalette;
TBlendType    currentBlending;                                // NOBLEND or LINEARBLEND
uint8_t maxChanges = 48;


//Vu7 Parameters
#define NSAMPLES 64
unsigned int samplearray[NSAMPLES];
unsigned long samplesum = 0;
unsigned int sampleavg = 0;
int samplecount = 0;
unsigned int sample = 0;
unsigned long oldtime = 0;
unsigned long newtime = 0;

//vu ripple
uint8_t colour; 
uint8_t myfade = 255;                                         // Starting brightness.
#define maxsteps 16                                           // Case statement wouldn't allow a variable.
int peakspersec = 0;
int peakcount = 0;
uint8_t bgcol = 0;   
int thisdelay = 20; 
int color;
int center = 0;
int center_2 = 0;
int step = -1;
float fadeRate = 0.80;
int diff;
int width=45;


// FOR SYLON ETC
uint8_t thisbeat =  23;
uint8_t thatbeat =  28;
uint8_t thisfade =   2;                                     // How quickly does it fade? Lower = slower fade rate.
uint8_t thissat = 255;                                     // The saturation, where 255 = brilliant colours.
uint8_t thisbri = 255; 
int    myhue =   0;

//FOR JUGGLE
uint8_t numdots = 4;                                          // Number of dots in use.
uint8_t faderate = 2;                                         // How long should the trails be. Very low value = longer trails.
uint8_t hueinc = 16;                                          // Incremental change in hue between each dot.
uint8_t thishue = 0;                                          // Starting hue.
uint8_t curhue = 0; 
uint8_t thisbright = 255;                                     // How bright should the LED/display be.
uint8_t basebeat = 5; 
uint8_t max_bright = 255;

// Twinkle
float redStates[N_PIXELS];
float blueStates[N_PIXELS];
float greenStates[N_PIXELS];
float Fade = 0.96;
//unsigned int sample;

//For Fire
#define SPARKING 50
#define COOLING  55
bool gReverseDirection = false;
#define FRAMES_PER_SECOND 60


//config for balls
#define NUM_BALLS         6                  // Number of bouncing balls you want (recommend < 7, butx 20 is fun in its own way)
#define GRAVITY           -9.81              // Downward (negative) acceleration of gravity in m/s^2
#define h0                1                  // Starting height, in meters, of the ball (strip length)
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))    
float h[NUM_BALLS] ;                         // An array of heights
float vImpact0 = sqrt( -2 * GRAVITY * h0 );  // Impact velocity of the ball when it hits the ground if "dropped" from the top of the strip
float vImpact[NUM_BALLS] ;                   // As time goes on the impact velocity will change, so make an array to store those values
float tCycle[NUM_BALLS] ;                    // The time since the last time the ball struck the ground
int   pos[NUM_BALLS] ;                       // The integer position of the dot on the strip (LED index)
int   pos_2[NUM_BALLS] ; 
long  tLast[NUM_BALLS] ;                     // The clock time of the last ground strike
float COR[NUM_BALLS] ;                       // Coefficient of Restitution (bounce damping)
float COR_2[NUM_BALLS] ; 

float
  greenOffset = 30,
  blueOffset = 150;

uint8_t gHue = 0;  //variable for rotating hue
int rotate_per = 8; //variable to adjust timing between rotating visualizers

int rotate_hue = 180;
int adj_bright = 220;

void setup() {
    analogReference(EXTERNAL);                                  // Connect 3.3V to AREF pin for any microphones using 3.3V

  Serial.begin(9600);
  pinMode(buttonPin1, INPUT);  
   pinMode(buttonPin2, INPUT);  
   pinMode(buttonPin3, INPUT);  
   pinMode(buttonPin4, INPUT);  
   pinMode(buttonPin5, INPUT); 
   pinMode(buttonPin6, INPUT);
  
  //Set all lights to make sure all are working as expected
  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, N_PIXELS);
  FastLED.addLeds<NEOPIXEL, LED_PIN_2>(leds_2, N_PIXELS_2);

  for (int i = 0; i < N_PIXELS; i++) 
    leds[i] = CRGB(70, 70, 70);

  for (int i = 0; i < N_PIXELS_2; i++) 
    leds_2[i] = CRGB(70, 70, 70);
    
  FastLED.show(); 
  delay(1000);  

  //bootstrap average with some low values
  for (int i = 0; i < AVGLEN; i++) {  
    insert(250, avgs, AVGLEN);
  }

  //Initial values
  high.times = 0;
  high.times_start = millis();
  Color.r = 0;  
  Color.g = 0;
  Color.b = 1;

  currentPalette = PartyColors_p;                             // Nice bright starting colours.
  currentBlending = LINEARBLEND;

  for (int i = 0 ; i < NUM_BALLS ; i++) {    // Initialize variables
    tLast[i] = millis();
    h[i] = h0;
    pos[i] = 0;                              // Balls start on the ground
    vImpact[i] = vImpact0;                   // And "pop" up at vImpact0
    tCycle[i] = 0;
    COR[i] = 0.90 - float(i)/pow(NUM_BALLS,2);  
  
  }
  
  ledMode = 1;
}


void loop ()
  {
  // see if switch is open or closed
  buttonState1 = digitalRead (buttonPin1);
  buttonState2 = digitalRead (buttonPin2);

 // if button 1 is pressed, switch current LED mode to constant light mode, jump to this from any mode
    if (buttonState1 == 1) {
        //then the button wend from off to on:
      delay(200);
      ledMode = 1;
      rotate_hue = 180;
      adj_bright = 220;
      Serial.print("ledMode is: ");
      Serial.println(ledMode);
    }
 
 // if button 2 is pressed, rotate to the next mode
  if (buttonState2 == 1) {
      delay(200); 
      if (ledMode == 5) ledMode = 1; 
      else ledMode++;
   
      Serial.print("ledMode is: ");
      Serial.println(ledMode);
      
    }

 switch (ledMode){
          
    case 1: 
      led_on_adjustable();
      break;

    case 2:
      All_upbeat();
      break;
     
    case 3:    
      All_chill();
      break;     
       
    case 4:
      led_mode_rotate_hue();
      break;
   
    case 5:
      led_off();
      break;

    default:
      break;
 }
  
  // has it changed since last time?
//  if (buttonState1 == 1)
//    {
//      delay(300);
//    if (oldMode == 5)
//       {
//        oldMode = 0;
//        Serial.print("oldMode is ");
//      Serial.println(oldMode);
// 
//       }  // end if buttonState1 is LOW
//      else {
//        oldMode++;
//        Serial.print("oldMode is ");
//      Serial.println(oldMode);
//   
//      }
//    }  // end of state change

//  switch(oldMode) {
//
//    case 0:
//      blur_and_spark();  
//      break;
//    case 1:
//      center_burst_quad_pulse();
//      break;
//    case 2:
//      All_chill();
//      break;
//    case 3:
//      led_off();
//      break;
//    case 4:
//      All_upbeat();
//      break;
//    case 0:
//      blur_and_beat2();
//      EVERY_N_MILLISECONDS(2000) vu10();     
//      break;
//    case 1:
//      Twinkle();
//      break;
//    case 2:
//      pattern2();
//      break;
//    case 3:
//      visualize_music_4();
//      break;
//    case 4:
//      visualize_music_3();
//      break;
//    case 5:
//      visualize_music_2();
//      break;
//    case 6:
//      center_burst_FHT();
//      break;
//    case 7:
//      pattern3();
//      break;
//    case 8:
//      fireblu();
//      break;
//    case 9:
//      vu7();
//      break;
//    case 10:
//      visualize_music_5();
//      break;
//    case 11:
//      blur();
//      break;
//    case 12:
//      Balls();
//      break;
//    case 13:
//      confetti();
//      break;
//    case 14:
//      vu10();
//      break;
//    default:
//      break;
//  }
//    delay(DELAY);       // delay in between reads for stability

  delay(2); 
}


/////////////////////////////////////////////////////////////////////////////////
//Solid Color LED modes

/////////////////////////////////////////////////////////////////////////////////
//main "normal" light mode, solid color, with adjustable hue and dimming

void led_on_adjustable() {
  
  buttonState3 = digitalRead(buttonPin3);
  buttonState4 = digitalRead(buttonPin4);
  buttonState5 = digitalRead(buttonPin5);
  buttonState6 = digitalRead(buttonPin6);

  if (buttonState3 == 1) {
      if (adj_bright > 230) adj_bright = 0;
      else adj_bright = adj_bright + 20;
      Serial.print("increase brightness to: ");
      Serial.println(adj_bright);
      delay(120);
    }

    // if button 4 is pressed, switch current LED mode to mode 4
    if (buttonState4 == 1) {
      if (adj_bright < 50) adj_bright = 0;
      else adj_bright = adj_bright - 20;
      Serial.print("decrease brightness to: ");
      Serial.println(adj_bright);
      delay(120);
    }

    // if button 5 is pressed, switch current LED mode to mode 5
    if (buttonState5 == 1) {
      if (rotate_hue > 230) rotate_hue = 240;
      else rotate_hue = rotate_hue + 20;
      Serial.print("increase hue to: ");
      Serial.println(rotate_hue);
      delay(120);
    }

  // if button 6 is pressed, switch current LED mode to mode 6
    if (buttonState6 == 1) {
      if (rotate_hue < 30) rotate_hue = 0;
      else rotate_hue = rotate_hue - 20;
      Serial.print("decrease hue to: ");
      Serial.println(rotate_hue);
      delay(120);
    }
  
  for (int i =0; i < N_PIXELS; i++) {
    leds[i] = CHSV(rotate_hue, 220, adj_bright);
   }
  for (int j =0; j < N_PIXELS_2; j++) {
    leds_2[j] = CHSV(rotate_hue, 220, adj_bright);
  }
  
  FastLED.show();
  delay(2);
}

//slowly rotate hue of all LEDs
void led_mode_rotate_hue() {

//int potin = map(analogRead(POT_PIN), 0, 1023, 250, 0); 

   for (int i =0; i < N_PIXELS; i++) {
    leds[i] = CHSV(myhue, 200, 200);
   }
   for (int j =0; j < N_PIXELS_2; j++) {
    leds_2[j] = CHSV(myhue, 200, 200);
   }

  FastLED.show();
  EVERY_N_MILLISECONDS( 100 ) { myhue++; } // slowly cycle the "base color" through the rainbow
  delay(2);
} 

void led_mode2() {
  
    //  Serial.println("running led_mode2");
   for (int i =0; i < N_PIXELS; i++) 
    leds[i] = CRGB(220, 220, 220);
     for (int i =0; i < N_PIXELS_2; i++) 
    leds_2[i] = CRGB(220, 220, 220);
  

FastLED.show();
delay(5);
}

void led_mode3() {
  
   for (int i =0; i < N_PIXELS; i++) 
    leds[i] = CRGB(0, 240, 0);
  

FastLED.show();
delay(5);
}

void led_mode4() {

      Serial.println("running led_mode4");

   for (int i =0; i < N_PIXELS; i++) 
    leds[i] = CRGB(0, 0, 240);
  

FastLED.show();
delay(5);
}

void led_mode5() {
      Serial.println("running led_mode5");

   for (int i =0; i < N_PIXELS; i++) 
    leds[i] = CRGB(240, 0, 0);
  

FastLED.show();
delay(5);
}


void led_off() {
      Serial.println("turning off LEDs");

   for (int i =0; i < N_PIXELS; i++) 
    leds[i] = CRGB(0, 0, 0);
  for (int i =0; i < N_PIXELS_2; i++) 
    leds_2[i] = CRGB(0, 0, 0);
  

FastLED.show();

}

/**Funtion to check if the lamp should either enter a HIGH mode,
or revert to NORMAL if already in HIGH. If the sensors report values
that are higher than 1.1 times the average values, and this has happened
more than 30 times the last few milliseconds, it will enter HIGH mode. 
TODO: Not very well written, remove hardcoded values, and make it more
reusable and configurable.  */
void check_high(int avg) {
  if (avg > (song_avg/iter * 1.1))  {
    if (high.times != 0) {
      if (millis() - high.times_start > 200.0) {
        high.times = 0;
        songmode = NORMAL;
      } else {
        high.times_start = millis();  
        high.times++; 
      }
    } else {
      high.times++;
      high.times_start = millis();

    }
  }
  if (high.times > 30 && millis() - high.times_start < 50.0)
    songmode = HIGH;
  else if (millis() - high.times_start > 200) {
    high.times = 0;
    songmode = NORMAL;
  }
}



/////////////////////////////////////////////////
//Main function for visualizing the sounds in the lamp
void visualize_music() {

  height_and_condition();
  
  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  for (int i = 0; i < N_PIXELS; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255)
        leds[i].r = 255;
      else if (leds[i].r + Color.r < 0)
        leds[i].r = 0;
      else
        leds[i].r = leds[i].r + Color.r;
          
      if (leds[i].g + Color.g > 255)
        leds[i].g = 255;
      else if (leds[i].g + Color.g < 0)
        leds[i].g = 0;
      else 
        leds[i].g = leds[i].g + Color.g;

      if (leds[i].b + Color.b > 255)
        leds[i].b = 255;
      else if (leds[i].b + Color.b < 0)
        leds[i].b = 0;
      else 
        leds[i].b = leds[i].b + Color.b;  
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
    }

  //keep peak at top
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < N_PIXELS) leds[peak] = CHSV(200, 200, 200);  // set peak dot
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
  
}


/////////////////////////////////////////////////////////////////////////////////////
//Center out spread visualizer

void visualize_music_2() {
  
  height_and_condition();
  height = height / 2;
  height_2 = height_2 / 2;  

  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  // Update Small LED array
  for (int i = 0; i < N_PIXELS_HALF; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[N_PIXELS_HALF-i-1].r + Color.r > 255) {
        leds[N_PIXELS_HALF-i-1].r = 255;
        leds[N_PIXELS_HALF+i].r = 255;
      }
      else if (leds[N_PIXELS_HALF-i-1].r + Color.r < 0) {
        leds[N_PIXELS_HALF-i-1].r = 0;
        leds[N_PIXELS_HALF+i].r = 0;
      }
      else {
        leds[N_PIXELS_HALF-i-1].r = leds[N_PIXELS_HALF-i-1].r + Color.r;
        leds[N_PIXELS_HALF+i].r = leds[N_PIXELS_HALF+i].r + Color.r;
      }
          
      if (leds[N_PIXELS_HALF-i-1].g + Color.g > 255) {
        leds[N_PIXELS_HALF-i-1].g = 255;
        leds[N_PIXELS_HALF+i].g = 255;
      }
      else if (leds[N_PIXELS_HALF-i-1].g + Color.g < 0) {
        leds[N_PIXELS_HALF-i-1].g = 0;
        leds[N_PIXELS_HALF+i].g = 0;
      }
      else {
        leds[N_PIXELS_HALF-i-1].g = leds[N_PIXELS_HALF-i-1].g + Color.g;
        leds[N_PIXELS_HALF+i].g = leds[N_PIXELS_HALF+i].g + Color.g;
      }


      if (leds[N_PIXELS_HALF-i-1].b + Color.b > 255) {
        leds[N_PIXELS_HALF-i-1].b = 255;
        leds[N_PIXELS_HALF+i].b = 255;
      }
      else if (leds[N_PIXELS_HALF-i-1].b + Color.b < 0) {
        leds[N_PIXELS_HALF-i-1].b = 0;
        leds[N_PIXELS_HALF+i].b = 0;
      }
      else {
        leds[N_PIXELS_HALF-i-1].b = leds[N_PIXELS_HALF-i-1].b + Color.b;  
        leds[N_PIXELS_HALF+i].b = leds[N_PIXELS_HALF+i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[N_PIXELS_HALF-i-1] = CRGB(leds[N_PIXELS_HALF-i-1].r/fade_scale, leds[N_PIXELS_HALF-i-1].g/fade_scale, leds[N_PIXELS_HALF-i-1].b/fade_scale);
      leds[N_PIXELS_HALF+i] = CRGB(leds[N_PIXELS_HALF+i].r/fade_scale, leds[N_PIXELS_HALF+i].g/fade_scale, leds[N_PIXELS_HALF+i].b/fade_scale);

    }

  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < N_PIXELS_HALF) {
    leds[N_PIXELS_HALF-peak] = CHSV(200, 200, 200);  // set peak dot
    leds[N_PIXELS_HALF+peak-1] = CHSV(200, 200, 200);  // set peak dot
  }


  // Update Large LED array
  for (int i = 0; i < N_PIXELS_HALF_2; i++) 
    //The leds we want to show
    if (i < height_2) {
      if (leds_2[N_PIXELS_HALF_2-i-1].r + Color.r > 255) {
        leds_2[N_PIXELS_HALF_2-i-1].r = 255;
        leds_2[N_PIXELS_HALF_2+i].r = 255;
      }
      else if (leds_2[N_PIXELS_HALF_2-i-1].r + Color.r < 0) {
        leds_2[N_PIXELS_HALF_2-i-1].r = 0;
        leds_2[N_PIXELS_HALF_2+i].r = 0;
      }
      else {
        leds_2[N_PIXELS_HALF_2-i-1].r = leds_2[N_PIXELS_HALF_2-i-1].r + Color.r;
        leds_2[N_PIXELS_HALF_2+i].r = leds_2[N_PIXELS_HALF_2+i].r + Color.r;
      }
          
      if (leds_2[N_PIXELS_HALF_2-i-1].g + Color.g > 255) {
        leds_2[N_PIXELS_HALF_2-i-1].g = 255;
        leds_2[N_PIXELS_HALF_2+i].g = 255;
      }
      else if (leds_2[N_PIXELS_HALF_2-i-1].g + Color.g < 0) {
        leds_2[N_PIXELS_HALF_2-i-1].g = 0;
        leds_2[N_PIXELS_HALF_2+i].g = 0;
      }
      else {
        leds_2[N_PIXELS_HALF_2-i-1].g = leds_2[N_PIXELS_HALF_2-i-1].g + Color.g;
        leds_2[N_PIXELS_HALF_2+i].g = leds_2[N_PIXELS_HALF_2+i].g + Color.g;
      }


      if (leds_2[N_PIXELS_HALF_2-i-1].b + Color.b > 255) {
        leds_2[N_PIXELS_HALF_2-i-1].b = 255;
        leds_2[N_PIXELS_HALF_2+i].b = 255;
      }
      else if (leds_2[N_PIXELS_HALF_2-i-1].b + Color.b < 0) {
        leds_2[N_PIXELS_HALF_2-i-1].b = 0;
        leds_2[N_PIXELS_HALF_2+i].b = 0;
      }
      else {
        leds_2[N_PIXELS_HALF_2-i-1].b = leds_2[N_PIXELS_HALF_2-i-1].b + Color.b;  
        leds_2[N_PIXELS_HALF_2+i].b = leds_2[N_PIXELS_HALF_2+i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds_2[N_PIXELS_HALF_2-i-1] = CRGB(leds_2[N_PIXELS_HALF_2-i-1].r/fade_scale, leds_2[N_PIXELS_HALF_2-i-1].g/fade_scale, leds_2[N_PIXELS_HALF_2-i-1].b/fade_scale);
      leds_2[N_PIXELS_HALF_2+i] = CRGB(leds_2[N_PIXELS_HALF_2+i].r/fade_scale, leds_2[N_PIXELS_HALF_2+i].g/fade_scale, leds_2[N_PIXELS_HALF_2+i].b/fade_scale);

    }

  //keep peak at top
  if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
  if(peak_2 > 0 && peak_2 < N_PIXELS_HALF_2) {
    for (int j = 0; j < 2; j++) {
      leds_2[N_PIXELS_HALF_2-peak_2] = CHSV(200, 200, 200);  // set peak dot
      leds_2[N_PIXELS_HALF_2+peak_2] = CHSV(200, 200, 200);  // set peak dot
    }
  }
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  
}


/////////////////////////////////////////////////////////////////////////////////////
//quadrasect spread out VU w/ falling dot

void visualize_music_3() {
  
  height_and_condition();
  height = height / 4;
  height_2 = height_2 / 4;



  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  //Small LED Array 
  for (int i = 0; i < N_PIXELS_QUARTER; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255) {
        leds[i].r = 255;
        leds[N_PIXELS_QUARTER+i].r = 255;
        leds[N_PIXELS_HALF+i].r = 255;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].r = 255;
      }
      else if (leds[i].r + Color.r < 0) {
        leds[i].r = 0;
        leds[N_PIXELS_QUARTER+i].r = 0;
        leds[N_PIXELS_HALF+i].r = 0;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].r = 0;
      }
      else {
        leds[i].r = leds[i].r + Color.r;
        leds[N_PIXELS_QUARTER+i].r = leds[N_PIXELS_QUARTER+i].r + Color.r;
        leds[N_PIXELS_HALF+i].r = leds[N_PIXELS_HALF+i].r + Color.r;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].r = leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds[i].g = 255;
        leds[N_PIXELS_QUARTER+i].g = 255;
        leds[N_PIXELS_HALF+i].g = 255;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g = 255;
      }
      else if (leds[i].g + Color.g < 0) {
        leds[i].g = 0;
        leds[N_PIXELS_QUARTER+i].g = 0;
        leds[N_PIXELS_HALF+i].g = 0;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g = 0;
      }
      else {
        leds[i].g = leds[i].g + Color.g;
        leds[N_PIXELS_QUARTER+i].g = leds[N_PIXELS_QUARTER+i].g + Color.g;
        leds[N_PIXELS_HALF+i].g = leds[N_PIXELS_HALF+i].g + Color.g;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g = leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g + Color.g;
      }


      if (leds[i].b + Color.b > 255) {
        leds[i].b = 255;
        leds[N_PIXELS_QUARTER+i].b = 255;
        leds[N_PIXELS_HALF+i].b = 255;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].b = 255;
      }
      else if (leds[i].b + Color.b < 0) {
        leds[i].b = 0;
        leds[N_PIXELS_QUARTER+i].b = 0;
        leds[N_PIXELS_HALF+i].b = 0;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].b = 0;
      }
      else {
        leds[i].b = leds[i].b + Color.b;
        leds[N_PIXELS_QUARTER+i].b = leds[N_PIXELS_QUARTER+i].b + Color.b;
        leds[N_PIXELS_HALF+i].b = leds[N_PIXELS_HALF+i].b + Color.b;
        leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].b = leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].b + Color.b; 
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
      leds[N_PIXELS_QUARTER+i] = CRGB(leds[N_PIXELS_QUARTER+i].r/fade_scale, leds[N_PIXELS_QUARTER+i].g/fade_scale, leds[N_PIXELS_QUARTER+i].b/fade_scale);
      leds[N_PIXELS_HALF+i] = CRGB(leds[N_PIXELS_HALF+i].r/fade_scale, leds[N_PIXELS_HALF+i].g/fade_scale, leds[N_PIXELS_HALF+i].b/fade_scale);
      leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i] = CRGB(leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].r/fade_scale, leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g/fade_scale, leds[N_PIXELS_HALF+N_PIXELS_QUARTER+i].b/fade_scale);
    }

  //peak for small LED array
  //keep peak at top
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < N_PIXELS_QUARTER) {
    leds[peak] = CHSV(200, 200, 200);  // set peak dot
    leds[N_PIXELS_QUARTER+peak] = CHSV(200, 200, 200);  // set peak dot
    leds[N_PIXELS_HALF+peak] = CHSV(200, 200, 200);
    leds[N_PIXELS_HALF+N_PIXELS_QUARTER+peak] = CHSV(200, 200, 200);
  }

  //Repeat for Large LED array
   for (int i = 0; i < N_PIXELS_QUARTER_2; i++) 
    //The leds we want to show
    if (i < height_2) {
      if (leds_2[i].r + Color.r > 255) {
        leds_2[i].r = 255;
        leds_2[N_PIXELS_QUARTER_2+i].r = 255;
        leds_2[N_PIXELS_HALF_2+i].r = 255;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].r = 255;
      }
      else if (leds_2[i].r + Color.r < 0) {
        leds_2[i].r = 0;
        leds_2[N_PIXELS_QUARTER_2+i].r = 0;
        leds_2[N_PIXELS_HALF_2+i].r = 0;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].r = 0;
      }
      else {
        leds_2[i].r = leds_2[i].r + Color.r;
        leds_2[N_PIXELS_QUARTER_2+i].r = leds_2[N_PIXELS_QUARTER_2+i].r + Color.r;
        leds_2[N_PIXELS_HALF_2+i].r = leds_2[N_PIXELS_HALF_2+i].r + Color.r;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].r = leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds_2[i].g = 255;
        leds_2[N_PIXELS_QUARTER_2+i].g = 255;
        leds_2[N_PIXELS_HALF_2+i].g = 255;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].g = 255;
      }
      else if (leds_2[i].g + Color.g < 0) {
        leds_2[i].g = 0;
        leds_2[N_PIXELS_QUARTER_2+i].g = 0;
        leds_2[N_PIXELS_HALF_2+i].g = 0;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].g = 0;
      }
      else {
        leds_2[i].g = leds_2[i].g + Color.g;
        leds_2[N_PIXELS_QUARTER_2+i].g = leds_2[N_PIXELS_QUARTER_2+i].g + Color.g;
        leds_2[N_PIXELS_HALF_2+i].g = leds_2[N_PIXELS_HALF+i].g + Color.g;
        leds_2[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g = leds_2[N_PIXELS_HALF+N_PIXELS_QUARTER+i].g + Color.g;
      }


      if (leds_2[i].b + Color.b > 255) {
        leds_2[i].b = 255;
        leds_2[N_PIXELS_QUARTER_2+i].b = 255;
        leds_2[N_PIXELS_HALF_2+i].b = 255;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].b = 255;
      }
      else if (leds_2[i].b + Color.b < 0) {
        leds_2[i].b = 0;
        leds_2[N_PIXELS_QUARTER_2+i].b = 0;
        leds_2[N_PIXELS_HALF_2+i].b = 0;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].b = 0;
      }
      else {
        leds_2[i].b = leds_2[i].b + Color.b;
        leds_2[N_PIXELS_QUARTER_2+i].b = leds_2[N_PIXELS_QUARTER_2+i].b + Color.b;
        leds_2[N_PIXELS_HALF_2+i].b = leds_2[N_PIXELS_HALF_2+i].b + Color.b;
        leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].b = leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].b + Color.b; 
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds_2[i] = CRGB(leds_2[i].r/fade_scale, leds_2[i].g/fade_scale, leds_2[i].b/fade_scale);
      leds_2[N_PIXELS_QUARTER_2+i] = CRGB(leds_2[N_PIXELS_QUARTER_2+i].r/fade_scale, leds_2[N_PIXELS_QUARTER_2+i].g/fade_scale, leds_2[N_PIXELS_QUARTER_2+i].b/fade_scale);
      leds_2[N_PIXELS_HALF_2+i] = CRGB(leds_2[N_PIXELS_HALF_2+i].r/fade_scale, leds_2[N_PIXELS_HALF_2+i].g/fade_scale, leds_2[N_PIXELS_HALF_2+i].b/fade_scale);
      leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i] = CRGB(leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].r/fade_scale, leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].g/fade_scale, leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+i].b/fade_scale);
    }

  //peak for large LED array
  //keep peak at top
  if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
  if(peak_2 > 3 && peak_2 < N_PIXELS_QUARTER_2) {
    for (int j = 0; j < 2; j++) {
    leds_2[peak_2-j] = CHSV(200, 200, 200);  // set peak dot
    leds_2[N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);  // set peak dot
    leds_2[N_PIXELS_HALF_2+peak_2-j] = CHSV(200, 200, 200);
    leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);
    }
  }
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  
}


/////////////////////////////////////////////////////////////////////////////////////
//outside in spread visualizer

void visualize_music_4() {
  
  height_and_condition();
  height = height / 2;
  height_2 = height_2 / 2;

  /*Set the different leds. Control for too high and too low values.
          Fun thing to try: Dont account for overflow in one direction, 
    some interesting light effects appear! */
  // Update Small LED array
  for (int i = 0; i < N_PIXELS_HALF; i++) 
    //The leds we want to show
    if (i < height) {
      if (leds[i].r + Color.r > 255) {
        leds[i].r = 255;
        leds[N_PIXELS-1-i].r = 255;
      }
      else if (leds[i].r + Color.r < 0) {
        leds[i].r = 0;
        leds[N_PIXELS-1-i].r = 0;
      }
      else {
        leds[i].r = leds[i].r + Color.r;
        leds[N_PIXELS-1-i].r = leds[N_PIXELS-1-i].r + Color.r;
      }
          
      if (leds[i].g + Color.g > 255) {
        leds[i].g = 255;
        leds[N_PIXELS-1-i].g = 255;
      }
      else if (leds[i].g + Color.g < 0) {
        leds[i].g = 0;
        leds[N_PIXELS-1-i].g = 0;
      }
      else {
        leds[i].g = leds[i].g + Color.g;
        leds[N_PIXELS-1-i].g = leds[N_PIXELS-1-i].g + Color.g;
      }


      if (leds[i].b + Color.b > 255) {
        leds[i].b = 255;
        leds[N_PIXELS-1-i].b = 255;
      }
      else if (leds[i].b + Color.b < 0) {
        leds[i].b = 0;
        leds[N_PIXELS-1-i].b = 0;
      }
      else {
        leds[i].b = leds[i].b + Color.b;  
        leds[N_PIXELS-1-i].b = leds[N_PIXELS-1-i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
      leds[N_PIXELS-1-i] = CRGB(leds[N_PIXELS-1-i].r/fade_scale, leds[N_PIXELS-1-i].g/fade_scale, leds[N_PIXELS-1-i].b/fade_scale);

    }

  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 0 && peak < N_PIXELS_HALF) {
    leds[peak] = CHSV(200, 200, 200);  // set peak dot
    leds[N_PIXELS-1-peak] = CHSV(200, 200, 200);  // set peak dot
  }


  // Update Large LED array
  for (int i = 0; i < N_PIXELS_HALF_2; i++) 
    //The leds we want to show
    if (i < height_2) {
      if (leds_2[i].r + Color.r > 255) {
        leds_2[i].r = 255;
        leds_2[N_PIXELS_2-1-i].r = 255;
      }
      else if (leds_2[i].r + Color.r < 0) {
        leds_2[i].r = 0;
        leds_2[N_PIXELS_2-1-i].r = 0;
      }
      else {
        leds_2[i].r = leds_2[i].r + Color.r;
        leds_2[N_PIXELS_2-1-i].r = leds_2[N_PIXELS_2-1-i].r + Color.r;
      }
          
      if (leds_2[i].g + Color.g > 255) {
        leds_2[i].g = 255;
        leds_2[N_PIXELS_2-1-i].g = 255;
      }
      else if (leds_2[i].g + Color.g < 0) {
        leds_2[i].g = 0;
        leds_2[N_PIXELS_2-1-i].g = 0;
      }
      else {
        leds_2[i].g = leds_2[i].g + Color.g;
        leds_2[N_PIXELS_2-1-i].g = leds_2[N_PIXELS_2-1-i].g + Color.g;
      }


      if (leds_2[i].b + Color.b > 255) {
        leds_2[i].b = 255;
        leds_2[N_PIXELS_2-1-i].b = 255;
      }
      else if (leds_2[i].b + Color.b < 0) {
        leds_2[i].b = 0;
        leds_2[N_PIXELS_2-1-i].b = 0;
      }
      else {
        leds_2[i].b = leds_2[i].b + Color.b;  
        leds_2[N_PIXELS_2-1-i].b = leds_2[N_PIXELS_2-1-i].b + Color.b;  
      }
      
    //All the other LEDs begin their fading journey to eventual total darkness
    } else {
      leds_2[i] = CRGB(leds_2[i].r/fade_scale, leds_2[i].g/fade_scale, leds_2[i].b/fade_scale);
      leds_2[N_PIXELS_2-1-i] = CRGB(leds_2[N_PIXELS_2-1-i].r/fade_scale, leds_2[N_PIXELS_2-1-i].g/fade_scale, leds_2[N_PIXELS_2-1-i].b/fade_scale);

    }

  //keep peak at top
    if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
  if(peak_2 > 0 && peak_2 < N_PIXELS_HALF_2) {
    for (int j = 0; j < 2; j++) {
      leds_2[peak_2] = CHSV(200, 200, 200);  // set peak dot
       leds_2[N_PIXELS_2-1-peak_2] = CHSV(200, 200, 200);  // set peak dot
    }
  }
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  
}

/////////////////////////////////////////////////////////////////////////////////////
//quadrasect spread out dot-only VU

void visualize_music_5() {
  
  height_and_condition();
  height = height / 4;
  height_2 = height_2 / 4;

  for (int i = 0; i < N_PIXELS; i++)
    leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
  for (int i = 0; i < N_PIXELS_2; i++)
    leds_2[i] = CRGB(leds_2[i].r/fade_scale, leds_2[i].g/fade_scale, leds_2[i].b/fade_scale);

  //peak for small LED array
  //keep peak at top
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 2 && peak < N_PIXELS_QUARTER) {
    
    for (int j = 0; j < 3; j++) {
      leds[peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[N_PIXELS_QUARTER+peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[N_PIXELS_HALF+peak-j] = CHSV(200, 200, 200);
      leds[N_PIXELS_HALF+N_PIXELS_QUARTER+peak-j] = CHSV(200, 200, 200);
    }
  }

  //Repeat for Large LED array
  //peak for large LED array
  //keep peak at top
  if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
  if(peak_2 > 0 && peak_2 < N_PIXELS_QUARTER_2) {
    for (int j = 0; j < 5; j++) {
      leds_2[peak_2-j] = CHSV(200, 200, 200);  // set peak dot
      leds_2[N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);  // set peak dot
      leds_2[N_PIXELS_HALF_2+peak_2-j] = CHSV(200, 200, 200);
      leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);
   } 
  }
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:
 
    if(++dotCount >= PEAK_FALL_split-5) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2-6) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  
}


///////////////////////////////////////////////////////////////////////////////////////////
//calculate a height based on mic input, also condition sound get avg
//used in all of visualize_music through visualize_music5 to condition and interpret mic input 

void height_and_condition () {
  
    //Actual sensor value
  sensor_value = analogRead(MIC_PIN);
  
  //If 0, discard immediately. Probably not right and save CPU.
  if (sensor_value == 0)
    return;

  //Discard readings that deviates too much from the past avg.
  mapped = (float)fscale(MIC_LOW, MIC_HIGH, MIC_LOW, (float)MIC_HIGH, (float)sensor_value, 2.0);
  avg = compute_average(avgs, AVGLEN);

  if (((avg - mapped) > avg*DEV_THRESH)) //|| ((avg - mapped) < -avg*DEV_THRESH))
    return;
  
  //Insert new avg. values
  insert(mapped, avgs, AVGLEN); 
  insert(avg, long_avg, LONG_SECTOR); 

  //Compute the "song average" sensor value
  song_avg += avg;
  iter++;
  if (iter > CYCLES) {  
    song_avg = song_avg / iter;
    iter = 1;
  }
    
  longavg = compute_average(long_avg, LONG_SECTOR);

  //Check if we enter HIGH mode 
  check_high(longavg);  

  if (songmode == HIGH) {
    fade_scale = 3;
    Color.r = 5;
    Color.g = 3;
    Color.b = -1;
  }
  else if (songmode == NORMAL) {
    fade_scale = 2;
    Color.r = -1;
    Color.b = 2;
    Color.g = 1;
  }

  //Decides how many of the LEDs will be lit
  height = fscale(MIC_LOW, MIC_HIGH, 0.0, (float)N_PIXELS, (float)avg, -1);
  height_2 = fscale(MIC_LOW, MIC_HIGH, 0.0, (float)N_PIXELS_2, (float)avg, -1);
}



///////////////////////////////////////////////////////////////////
//Compute average of a int array, given the starting pointer and the length

int compute_average(int *avgs, int len) {
  int sum = 0;
  for (int i = 0; i < len; i++)
    sum += avgs[i];

  return (int)(sum / len);

}

//Insert a value into an array, and shift it down removing
//the first value if array already full 
void insert(int val, int *avgs, int len) {
  for (int i = 0; i < len; i++) {
    if (avgs[i] == -1) {
      avgs[i] = val;
      return;
    }  
  }

  for (int i = 1; i < len; i++) {
    avgs[i - 1] = avgs[i];
  }
  avgs[len - 1] = val;
}

//Function imported from the arduino website.
//Basically map, but with a curve on the scale (can be non-uniform).
float fscale( float originalMin, float originalMax, float newBegin, float
    newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}

/////////////////////////////////////////////////////////////////////////////
//flash ripple and background pulse

void vu7() {

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                       // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                                 // Reset the counter every second.
  }

  soundmems();

  EVERY_N_MILLISECONDS(20) {
   ripple3();
  }

  // show_at_max_brightness_for_power();
  FastLED.show();
  delay(3);
} // loop()


void soundmems() {                                                  // Rolling average counter - means we don't have to go through an array each time.
  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 90);

  samplesum = samplesum + sample - samplearray[samplecount];        // Add the new sample and remove the oldest sample in the array 
  sampleavg = samplesum / NSAMPLES;                                 // Get an average
  samplearray[samplecount] = sample;                                // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                       // Update the counter for the array

  //if (newtime > (oldtime + 200)) digitalWrite(13, LOW);             // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + 60)) && (newtime > (oldtime + 60)) ) { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.  
    //note: hardcoded 30 instead of potin in above line
    step = -1;
    peakcount++;
    //digitalWrite(13, HIGH);
    oldtime = newtime;
  }
}  // soundmems()



void ripple3() {
  for (int i = 0; i < N_PIXELS; i++) leds[i] = CHSV(bgcol, 255, sampleavg*2);  // Set the background colour.
  for (int j = 0; j < N_PIXELS_2; j++) leds_2[j] = CHSV(bgcol, 255, sampleavg*2);  // Set the background colour.


  switch (step) {

    case -1:                                                          // Initialize ripple variables.
      center = random(N_PIXELS);
      center_2 = random(N_PIXELS_2);
      colour = (peakspersec*10) % 255;                                             // More peaks/s = higher the hue colour.
      step = 0;
      bgcol = bgcol+8;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      leds_2[center_2] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                                    // At the end of the ripples.
      // step = -1;
      break;

    default:                                                             // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      leds_2[(center_2 + step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds_2[(center_2 - step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                         // Next step.
      break;  
  } // switch step

}


////////////////////////////////////////////////////////
//Center Burst  Visualizer

void center_burst_FHT() {
  
  EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(25) {
    fadeToBlackBy(leds, N_PIXELS, 10);                            // 8 bit, 1 = slow, 255 = fast    
    fadeToBlackBy(leds_2, N_PIXELS_2, 4);                            // 8 bit, 1 = slow, 255 = fast  
    sndwave();
    soundmems();
  }

  FastLED.show();
}

//basic soundmems used for original Center Burst, using mega VU version of soundmems instead
//void soundmems() {                                       // Rolling average counter - means we don't have to go through an array each time.
//  
//  int tmp = analogRead(inputPin) - 512- DC_OFFSET;
//  sample = abs(tmp);
//  
//}  // soundmems()



void sndwave() {
  
  leds[N_PIXELS/2] = ColorFromPalette(currentPalette, sample, sample*2, currentBlending); 
  leds_2[N_PIXELS_2/2] = ColorFromPalette(currentPalette, sample, sample*2, currentBlending); 
  
  for (int i = N_PIXELS - 1; i > N_PIXELS/2; i--) {       //move to the left
    leds[i] = leds[i - 1];
  }

  for (int i = 0; i < N_PIXELS/2; i++) {                  // move to the right
    leds[i] = leds[i + 1];
  }

  for (int i = N_PIXELS_2 - 1; i > N_PIXELS_2/2; i--) {       //move to the left
    leds_2[i] = leds_2[i - 1];
  }

  for (int i = 0; i < N_PIXELS_2/2; i++) {                  // move to the right
    leds_2[i] = leds_2[i + 1];
  }
} // sndwave()


///////////////////////////////////////////////////////////////////
// Pattern 3 - JUGGLE
void pattern3() {
  ChangeMe();
  juggle();
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()

void juggle() {                                               // Several colored dots, weaving in and out of sync with each other
  curhue = thishue;                                          // Reset the hue values.
  fadeToBlackBy(leds, N_PIXELS, faderate);
  fadeToBlackBy(leds_2, N_PIXELS_2, faderate);

  for( int i = 0; i < numdots; i++) {
    leds[beatsin16(basebeat+i+numdots,0,N_PIXELS)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    leds_2[beatsin16(basebeat+i+numdots,0,N_PIXELS_2)] += CHSV(curhue, thissat, thisbright);   //beat16 is a FastLED 3.1 function
    curhue += hueinc;
  }
  
} // juggle()

void ChangeMe() {                                             // A time (rather than loop) based demo sequencer. This gives us full control over the length of each sequence.
  uint8_t secondHand = (millis() / 1000) % 30;                // IMPORTANT!!! Change '30' to a different value to change duration of the loop.
  static uint8_t lastSecond = 99;                             // Static variable, means it's only defined once. This is our 'debounce' variable.
  if (lastSecond != secondHand) {                             // Debounce to make sure we're not repeating an assignment.
    lastSecond = secondHand;
    if (secondHand ==  0)  {numdots=1; faderate=2;}  // You can change values here, one at a time , or altogether.
    if (secondHand == 10)  {numdots=4; thishue=128; faderate=8;}
    if (secondHand == 20)  {hueinc=48; thishue=random8();}                               // Only gets called once, and not continuously for the next several seconds. Therefore, no rainbows.
  }
} // ChangeMe()


///////////////////////////////////////////////////////////////////
// "Blue" Flame Effect

void fireblu(){
//#define FRAMES_PER_SECOND 60
random16_add_entropy( random());

  
// Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];
  static byte heat_2[N_PIXELS_2];


  // Step 1.  Cool down every cell a little
    for( int i = 0; i < N_PIXELS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS) + 2));
    }
    for( int j = 0; j < N_PIXELS_2; j++) {
      heat_2[j] = qsub8( heat_2[j],  random8(0, ((COOLING * 14) / N_PIXELS_2) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= N_PIXELS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
     for( int l= N_PIXELS_2 - 1; l >= 2; l--) {
      heat_2[l] = (heat_2[l - 1] + heat_2[l - 2] + heat_2[l - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
      int z = random8(7);
      heat_2[y] = qadd8( heat_2[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < N_PIXELS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (N_PIXELS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;

    }

    for( int j = 0; j < N_PIXELS_2; j++) {
      byte colorindex = scale8( heat_2[j], 240);
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
      leds_2[j] = color;     
    }
    FastLED.show();
  }


///////////////////////////////////////////////////////////////////
// Two Sided  Flame Effect

void fire3(){
  #define FRAMES_PER_SECOND 40
  random16_add_entropy( random());

  
  // Array of temperature readings at each simulation cell
  static byte heat[N_PIXELS];
  static byte heat_2[N_PIXELS_2];


  // Step 1.  Cool down every cell a little
    for( int i = 0; i < N_PIXELS_HALF; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / N_PIXELS_HALF) + 2));
      heat[N_PIXELS-i-1] = heat[i];
    }
    for( int j = 0; j < N_PIXELS_HALF_2; j++) {
      heat_2[j] = qsub8( heat_2[j],  random8(0, ((COOLING * 10) / N_PIXELS_HALF_2) + 2));
      heat_2[N_PIXELS_2-j-1] = heat_2[j];
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= N_PIXELS_HALF - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
      heat[N_PIXELS-k-1] = heat[k];
    }
    for( int l= N_PIXELS_HALF_2 - 1; l >= 2; l--) {
      heat_2[l] = (heat_2[l - 1] + heat_2[l - 2] + heat_2[l - 2] ) / 3;
      heat_2[N_PIXELS_2-l-1] = heat_2[l];
    }
    
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
      heat[N_PIXELS-y-1] = heat[y];
//      int z = random8(7);
      heat_2[y] = qadd8( heat_2[y], random8(160,255) );
      heat_2[N_PIXELS_2-y-1] = heat_2[y];
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < N_PIXELS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Red, CRGB::Yellow,  CRGB::White), colorindex);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (N_PIXELS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }

    for( int j = 0; j < N_PIXELS_2; j++) {
      byte colorindex = scale8( heat_2[j], 240);
      CRGB color = ColorFromPalette( CRGBPalette16( CRGB::Black, CRGB::Blue, CRGB::Aqua,  CRGB::White), colorindex);
      leds_2[j] = color;     
    }

    FastLED.show();
  }



///////////////////////////////////////////////////////////////////
// Ripple with Black background and variable width


void vu10() {

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                  // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                            // Reset the counter every second.
  }

  soundrip();

  EVERY_N_MILLISECONDS(20) {
   rippled();
  }

   FastLED.show();
  delay(3);
} // loop()


void soundrip() {                                            // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60) + 20;

  samplesum = samplesum + sample - samplearray[samplecount];  // Add the new sample and remove the oldest sample in the array 
  sampleavg = samplesum / NSAMPLES;                           // Get an average
  samplearray[samplecount] = sample;                          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                 // Update the counter for the array

  if (newtime > (oldtime + 400)) digitalWrite(13, LOW);       // Turn the LED off 200ms after the last peak.


  
  EVERY_N_MILLISECONDS(1000) {
    uint8_t width_adj = random8(15);
    width = 30 + (width_adj * 10);
  }
  if ((sample > (sampleavg + width)) && (newtime > (oldtime + 60)) ) { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    oldtime = newtime;
  }
  
}  // soundmems()



void rippled() {

  fadeToBlackBy(leds, N_PIXELS, 44);                          // 8 bit, 1 = slow, 255 = fast
  fadeToBlackBy(leds_2, N_PIXELS_2, 34);                          // 8 bit, 1 = slow, 255 = fast

  
  switch (step) {

    case -1:                                                  // Initialize ripple variables.
      center = random(N_PIXELS);
      center_2 = random(N_PIXELS_2);
      colour = (peakspersec*10) % 255;                        // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                  // Display the first pixel of the ripple.
      leds_2[center_2] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                            // At the end of the ripples.
      // step = -1;
      break;

    default:                                                  // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      leds_2[(center_2 + step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds_2[(center_2 - step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                // Next step.
      break;  
  } // switch step
  
} // ripple()



///////////////////////////////////////////////////////////////////
// Ripple with Black background and constant width

void ripple_burst() {

  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                  // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                            // Reset the counter every second.
  }

  soundrip_2();

  EVERY_N_MILLISECONDS(30) {
   rippled_2();
  }

   FastLED.show();

} // loop()


void soundrip_2() {                                            // Rolling average counter - means we don't have to go through an array each time.

  newtime = millis();
  int tmp = analogRead(MIC_PIN) - 512;
  sample = abs(tmp);

  //int potin = map(analogRead(POT_PIN), 0, 1023, 0, 60) + 20;

  samplesum = samplesum + sample - samplearray[samplecount];  // Add the new sample and remove the oldest sample in the array 
  sampleavg = samplesum / NSAMPLES;                           // Get an average
  samplearray[samplecount] = sample;                          // Update oldest sample in the array with new sample
  samplecount = (samplecount + 1) % NSAMPLES;                 // Update the counter for the array

  if (newtime > (oldtime + 200)) digitalWrite(13, LOW);       // Turn the LED off 200ms after the last peak.

  if ((sample > (sampleavg + 60)) && (newtime > (oldtime + 60)) ) { // Check for a peak, which is 30 > the average, but wait at least 60ms for another.
    step = -1;
    peakcount++;
    digitalWrite(13, HIGH);
    oldtime = newtime;
  }
  
}  // soundmems()



void rippled_2() {

  fadeToBlackBy(leds, N_PIXELS, 44);                          // 8 bit, 1 = slow, 255 = fast
  fadeToBlackBy(leds_2, N_PIXELS_2, 34);                          // 8 bit, 1 = slow, 255 = fast

  switch (step) {

    case -1:                                                  // Initialize ripple variables.
      center = random(N_PIXELS);
      center_2 = random(N_PIXELS_2);
      colour = (peakspersec*10) % 255;                        // More peaks/s = higher the hue colour.
      step = 0;
      break;

    case 0:
      leds[center] = CHSV(colour, 255, 255);                  // Display the first pixel of the ripple.
      leds_2[center_2] = CHSV(colour, 255, 255);                          // Display the first pixel of the ripple.
      step ++;
      break;

    case maxsteps:                                            // At the end of the ripples.
      // step = -1;
      break;

    default:                                                  // Middle of the ripples.
      leds[(center + step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds[(center - step + N_PIXELS) % N_PIXELS] += CHSV(colour, 255, myfade/step*2);
      leds_2[(center_2 + step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);       // Simple wrap from Marc Miller.
      leds_2[(center_2 - step + N_PIXELS_2) % N_PIXELS_2] += CHSV(colour, 255, myfade/step*2);
      step ++;                                                // Next step.
      break;  
  } // switch step
  
} // ripple_2()


//////////////////////////////////////////////////////////////////
// Bouncing Balls

void Balls() {
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    tCycle[i] =  millis() - tLast[i] ;     // Calculate the time since the last time the ball was on the ground

    // A little kinematics equation calculates positon as a function of time, acceleration (gravity) and intial velocity
    h[i] = 0.5 * GRAVITY * pow( tCycle[i]/1000 , 2.0 ) + vImpact[i] * tCycle[i]/1000;

    if ( h[i] < 0 ) {                      
      h[i] = 0;                            // If the ball crossed the threshold of the "ground," put it back on the ground
      vImpact[i] = COR[i] * vImpact[i] ;   // and recalculate its new upward velocity as it's old velocity * COR
      tLast[i] = millis();

      if ( vImpact[i] < 0.01 ) vImpact[i] = vImpact0;  // If the ball is barely moving, "pop" it back up at vImpact0
    }
    pos[i] = round( h[i] * (N_PIXELS - 1) / h0);       // Map "h" to a "pos" integer index position on the LED strip
    pos_2[i] = round( h[i] * (N_PIXELS_2 - 1) / h0);       // Map "h" to a "pos" integer index position on the LED strip
  }

  //Choose color of LEDs, then the "pos" LED on
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CHSV( uint8_t (i * 40) , 255, 255);
    leds_2[pos_2[i]] = CHSV( uint8_t (i * 40) , 255, 255);
  }
  FastLED.show();
  //Then off for the next loop around
  for (int i = 0 ; i < NUM_BALLS ; i++) {
    leds[pos[i]] = CRGB::Black;
    leds_2[pos_2[i]] = CRGB::Black;
  }
}


/////////////////////////////////////////////////////////////////
// Sinelon variation

void pattern2() {
      
       sinelon();                                                  // Call our sequence.
  show_at_max_brightness_for_power();                         // Power managed display of LED's.
} // loop()


void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, N_PIXELS, thisfade);
  fadeToBlackBy( leds_2, N_PIXELS_2, thisfade);
  int pos1 = beatsin16(thisbeat,0,N_PIXELS);
  int pos2 = beatsin16(thatbeat,0,N_PIXELS);
  int pos1_2 = beatsin16(thisbeat,0,N_PIXELS_2);
  int pos2_2 = beatsin16(thatbeat,0,N_PIXELS_2);
  leds[(pos1+pos2)/2] += CHSV( myhue++/64, thissat, thisbri);
  leds_2[(pos1_2+pos2_2)/2] += CHSV( myhue++/64, thissat, thisbri);
}

/////////////////////////////////////////////////////////////////
// Slow blending change of hue

void blur() {

  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, N_PIXELS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  blur1d( leds_2, N_PIXELS_2, blurAmount);    
  
  uint8_t  i = beatsin8(  9, 0, N_PIXELS);
  uint8_t  j = beatsin8( 7, 0, N_PIXELS);
  uint8_t  k = beatsin8(  5, 0, N_PIXELS);
  uint8_t  i_2 = beatsin8(  9, 0, N_PIXELS_2);
  uint8_t  j_2 = beatsin8( 7, 0, N_PIXELS_2);
  uint8_t  k_2 = beatsin8(  5, 0, N_PIXELS_2);
  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);

   leds_2[(i_2+j_2)/2] = CHSV( ms / 29, 200, 255);
  leds_2[(j_2+k_2)/2] = CHSV( ms / 41, 200, 255);
  leds_2[(k_2+i_2)/2] = CHSV( ms / 73, 200, 255);
  leds_2[(k_2+i_2+j_2)/3] = CHSV( ms / 53, 200, 255);
  
  FastLED.show();
  
} // loop()


//////////////////////////////////////////////////////////////////
//FastLED confetti

void confetti() 
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, N_PIXELS, 10);
  fadeToBlackBy( leds_2, N_PIXELS_2, 10);
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  int pos = random16(N_PIXELS);
  int pos_2 = random16(N_PIXELS_2);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
  leds_2[pos_2] += CHSV( gHue + random8(64), 200, 255);
  FastLED.show();
}

//////////////////////////////////////////////////////////////////
//slow twinkling

void Twinkle () {
   if (random(15) == 1) {
      uint16_t i = random(N_PIXELS);
      uint16_t i_2 = random(N_PIXELS_2);
      
      if (redStates[i] < 1 && greenStates[i] < 1 && blueStates[i] < 1) {
        redStates[i] = random(256);
        greenStates[i] = random(256);
        blueStates[i] = random(256);
      }

       if (redStates[i_2] < 1 && greenStates[i_2] < 1 && blueStates[i_2] < 1) {
        redStates[i_2] = random(256);
        greenStates[i_2] = random(256);
        blueStates[i_2] = random(256);
      }
    }
    
    for(uint16_t l = 0; l < N_PIXELS; l++) {
      if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
        leds[l].setRGB(redStates[l], greenStates[l], blueStates[l]);
        
        if (redStates[l] > 1) {
          redStates[l] = redStates[l] * Fade;
        } else {
          redStates[l] = 0;
        }
        
        if (greenStates[l] > 1) {
          greenStates[l] = greenStates[l] * Fade;
        } else {
          greenStates[l] = 0;
        }
        
        if (blueStates[l] > 1) {
          blueStates[l] = blueStates[l] * Fade;
        } else {
          blueStates[l] = 0;
        }
        
      } else {
        leds[l].setRGB(0, 0, 0);
      }
    }

     for(uint16_t l = 0; l < N_PIXELS_2; l++) {
      if (redStates[l] > 1 || greenStates[l] > 1 || blueStates[l] > 1) {
        leds_2[l].setRGB(redStates[l], greenStates[l], blueStates[l]);
        
        if (redStates[l] > 1) {
          redStates[l] = redStates[l] * Fade;
        } else {
          redStates[l] = 0;
        }
        
        if (greenStates[l] > 1) {
          greenStates[l] = greenStates[l] * Fade;
        } else {
          greenStates[l] = 0;
        }
        
        if (blueStates[l] > 1) {
          blueStates[l] = blueStates[l] * Fade;
        } else {
          blueStates[l] = 0;
        }
        
      } else {
        leds_2[l].setRGB(0, 0, 0);
      }
     }
    FastLED.show();
     delay(5);
  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// blur with ripple spark overlay

void blur_and_spark() {

  height_and_condition();
  height = height / 4;
  height_2 = height_2 / 4;
  EVERY_N_MILLISECONDS(1000) {
    peakspersec = peakcount;                                  // Count the peaks per second. This value will become the foreground hue.
    peakcount = 0;                                            // Reset the counter every second.
  }
  
  for (int i = 0; i < N_PIXELS; i++)
    leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
  for (int i = 0; i < N_PIXELS_2; i++)
    leds_2[i] = CRGB(leds_2[i].r/fade_scale, leds_2[i].g/fade_scale, leds_2[i].b/fade_scale);

  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, N_PIXELS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  blur1d( leds_2, N_PIXELS_2, blurAmount);    
  
  uint8_t  i = beatsin8(  9, 0, N_PIXELS);
  uint8_t  j = beatsin8( 7, 0, N_PIXELS);
  uint8_t  k = beatsin8(  5, 0, N_PIXELS);
  uint8_t  i_2 = beatsin8(  9, 0, N_PIXELS_2);
  uint8_t  j_2 = beatsin8( 7, 0, N_PIXELS_2);
  uint8_t  k_2 = beatsin8(  5, 0, N_PIXELS_2);
  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);

   leds_2[(i_2+j_2)/2] = CHSV( ms / 29, 200, 255);
  leds_2[(j_2+k_2)/2] = CHSV( ms / 41, 200, 255);
  leds_2[(k_2+i_2)/2] = CHSV( ms / 73, 200, 255);
  leds_2[(k_2+i_2+j_2)/3] = CHSV( ms / 53, 200, 255);

  soundrip_2();

  EVERY_N_MILLISECONDS(30) {
   rippled_2();
  }


  FastLED.show();

} // loop()


//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Slow blending change of hue with VU meter dot over it, combine with vu10() to make work

///Need to figure out what is making, all lights go on periodically and fix

void blur_and_beat() {

  height_and_condition();
  height = height / 4;
  height_2 = height_2 / 4;
  
  for (int i = 0; i < N_PIXELS; i++)
    leds[i] = CRGB(leds[i].r/fade_scale, leds[i].g/fade_scale, leds[i].b/fade_scale);
  for (int i = 0; i < N_PIXELS_2; i++)
    leds_2[i] = CRGB(leds_2[i].r/fade_scale, leds_2[i].g/fade_scale, leds_2[i].b/fade_scale);

  uint8_t blurAmount = dim8_raw( beatsin8(3,64, 192) );       // A sinewave at 3 Hz with values ranging from 64 to 192.
  blur1d( leds, N_PIXELS, blurAmount);                        // Apply some blurring to whatever's already on the strip, which will eventually go black.
  blur1d( leds_2, N_PIXELS_2, blurAmount);    
  
  uint8_t  i = beatsin8(  9, 0, N_PIXELS);
  uint8_t  j = beatsin8( 7, 0, N_PIXELS);
  uint8_t  k = beatsin8(  5, 0, N_PIXELS);
  uint8_t  i_2 = beatsin8(  9, 0, N_PIXELS_2);
  uint8_t  j_2 = beatsin8( 7, 0, N_PIXELS_2);
  uint8_t  k_2 = beatsin8(  5, 0, N_PIXELS_2);
  
  // The color of each point shifts over time, each at a different speed.
  uint16_t ms = millis();  
  leds[(i+j)/2] = CHSV( ms / 29, 200, 255);
  leds[(j+k)/2] = CHSV( ms / 41, 200, 255);
  leds[(k+i)/2] = CHSV( ms / 73, 200, 255);
  leds[(k+i+j)/3] = CHSV( ms / 53, 200, 255);

   leds_2[(i_2+j_2)/2] = CHSV( ms / 29, 200, 255);
  leds_2[(j_2+k_2)/2] = CHSV( ms / 41, 200, 255);
  leds_2[(k_2+i_2)/2] = CHSV( ms / 73, 200, 255);
  leds_2[(k_2+i_2+j_2)/3] = CHSV( ms / 53, 200, 255);

  //peak for small LED array
  //keep peak at top
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 2 && peak < N_PIXELS_QUARTER) {
    
    for (int j = 0; j < 3; j++) {
      leds[peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[N_PIXELS_QUARTER+peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[N_PIXELS_HALF+peak-j] = CHSV(200, 200, 200);
      leds[N_PIXELS_HALF+N_PIXELS_QUARTER+peak-j] = CHSV(200, 200, 200);
    }
  }

  //Repeat for Large LED array
  //peak for large LED array
  //keep peak at top
  if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
  if(peak_2 > 0 && peak_2 < N_PIXELS_QUARTER_2) {
    for (int j = 0; j < 5; j++) {
      leds_2[peak_2-j] = CHSV(200, 200, 200);  // set peak dot
      leds_2[N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);  // set peak dot
      leds_2[N_PIXELS_HALF_2+peak_2-j] = CHSV(200, 200, 200);
      leds_2[N_PIXELS_HALF_2+N_PIXELS_QUARTER_2+peak_2-j] = CHSV(200, 200, 200);
   } 
  }
  
  FastLED.show();

  if(++dotCount >= PEAK_FALL_split-5) { //fall rate 
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2-6) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  delay(2);
} // loop()


void blur_and_beat2() {
   blur_and_beat();
   EVERY_N_MILLISECONDS(2000) vu10();     
}


////////////////////////////////////////////////////////
//Center Burst and quad pulse overlay

void center_burst_quad_pulse() {

   
  EVERY_N_MILLISECONDS(100) {                                 // AWESOME palette blending capability.
     
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxChanges);
  }


  EVERY_N_SECONDS(5) {                                        // Change the palette every 5 seconds.
    for (int i = 0; i < 16; i++) {
      targetPalette[i] = CHSV(random8(), 255, 255);
    }
  }

  EVERY_N_MILLISECONDS(25) {
    fadeToBlackBy(leds, N_PIXELS, 10);                            // 8 bit, 1 = slow, 255 = fast    
    fadeToBlackBy(leds_2, N_PIXELS_2, 4);                            // 8 bit, 1 = slow, 255 = fast  
    sndwave();
    soundmems();
  }

  height_and_condition();
  height = height / 2;
  height_2 = height_2 / 2;

  //keep peak at top for small LED array
  if(height > peak)     peak   = height; // Keep 'peak' dot at top
  if(peak > 2 && peak < N_PIXELS_HALF) {
    for (int j = 0; j < 3; j++) {
      leds[peak-j] = CHSV(200, 200, 200);  // set peak dot
      leds[N_PIXELS-1-peak+j] = CHSV(200, 200, 200);  // set peak dot
    }
  }

 //peak for small LED array
  //keep peak at top
    if(height_2 > peak_2)     peak_2   = height_2; // Keep 'peak' dot at top
    if(peak_2 > 5 && peak_2 < N_PIXELS_HALF_2) {
      for (int j = 0; j < 6; j++) {
        leds_2[peak_2-j] = CHSV(200, 200, 200);  // set peak dot
        leds_2[N_PIXELS_2-1-peak_2+j] = CHSV(200, 200, 200);  // set peak dot
    }
  }
  
  FastLED.show(); 

    // Every few frames, make the peak pixel drop by 1:

    if(++dotCount >= PEAK_FALL_split-4) { //fall rate 
      
      if(peak > 0) peak--;
      dotCount = 0;
    } 
    if(++dotCount_2 >= PEAK_FALL_split_2-2) { //fall rate 
      if(peak_2 > 0) peak_2--;
      dotCount_2 = 0;
    } 
  
  delay(2);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rotate Music Sync / Music Accompanying Patterns

// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {visualize_music_5, vu7, pattern2, visualize_music_2, fire3, center_burst_FHT, pattern3, vu10, Balls, blur, visualize_music_3, confetti, ripple_burst, visualize_music_3, center_burst_quad_pulse, visualize_music_4, vu10};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current



void All_upbeat()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  int length_adj = random(20); //randomly adjust timing between switches between 5 and 25 sec
    rotate_per = 5 + length_adj;
    
  EVERY_N_SECONDS( rotate_per ) { 
    
    nextPattern(); 
    
  } // change patterns periodically
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

// second list
// List of patterns to cycle through.  Each is defined as a separate function below.
typedef void (*SimplePatternList[])();
SimplePatternList qPatterns = {Twinkle, ripple_burst, pattern2, fire3, blur, confetti, pattern3, fireblu};
uint8_t qCurrentPatternNumber = 0; // Index number of which pattern is current

void All_chill()
{
  // Call the current pattern function once, updating the 'leds' array
  qPatterns[qCurrentPatternNumber]();
  EVERY_N_SECONDS( 10 ) { nextPattern2(); } // change patterns periodically
}

void nextPattern2()
{
  // add one to the current pattern number, and wrap around at the end
  qCurrentPatternNumber = (qCurrentPatternNumber + 1) % ARRAY_SIZE( qPatterns);
}

