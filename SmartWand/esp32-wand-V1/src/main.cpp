#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "config.h"




int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
int16_t smooth_gyro_z;
int16_t temp;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
bool oneshot_flag = false;


char tempStr[7];

char* convert(int16_t i) {
    sprintf(tempStr, "%6d", i);
    return tempStr;
}

//Screen Constants
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADR 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUMFLAKES     10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT   16
#define LOGO_WIDTH    16
static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 };

void testdrawline();      // Draw many lines
void testdrawrect();      // Draw rectangles (outlines)
void testfillrect();      // Draw rectangles (filled)
void testdrawcircle();    // Draw circles (outlines)
void testfillcircle();    // Draw circles (filled)
void testdrawroundrect(); // Draw rounded rectangles (outlines)
void testfillroundrect(); // Draw rounded rectangles (filled)
void testdrawtriangle();  // Draw triangles (outlines)
void testfilltriangle();  // Draw triangles (filled)
void testdrawchar();      // Draw characters of the default font
void testdrawstyles();    // Draw 'stylized' characters
void testscrolltext();    // Draw scrolling text
void testdrawbitmap();    // Draw a small bitmap image
void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);
void printData();
void getAccelData();
void CoordBoundsCheck(int16_t* x, int16_t* y);
void calculate_IMU_error();
float get_gyro_smoothed();
int16_t get_yaw_value();
void draw_buffer();

//Global Variables
char str[7];

//Accelerometer Funciton Prototypes
char*  convert_to_str(int16_t i) {
  sprintf(str, "%6d", i);
  return str;
}

float get_pin_voltage(int pin) {
  return ((float) analogRead(pin)/4095) * 3.3;
}


void setup() {
  Serial.begin(9600);
  pinMode(BUTTON, INPUT);
  //Accel Setup
  Wire.begin();
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(0x6B);
  Wire.write(0);
  //Accel Finish Setup


  Wire.endTransmission(true);
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADR)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  calculate_IMU_error();
  Serial.println("Set up Complete");
}

void loop() {
  

  // accAngleX = (atan(accel_y / sqrt(pow(accel_x, 2) + pow(accel_z, 2))) * 180 / PI) - AccErrorX ; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  // accAngleY = (atan(-1 * accel_x / sqrt(pow(accel_y, 2) + pow(accel_z, 2))) * 180 / PI) - AccErrorY; // AccErrorY ~(-1.58)


  // // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  // gyroAngleX = gyroAngleX + gyro_x * elapsedTime; // deg/s * s = deg
  // gyroAngleY = gyroAngleY + gyro_y * elapsedTime;
  // // Complementary filter - combine acceleromter and gyro angle values
  // roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  // pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;


    

  
  getAccelData();
  int16_t yaw_int = (int16_t) yaw;
  CoordBoundsCheck(&accel_y, &yaw_int);
  static int pixelsY[NUM_PIXELS], pixelsX[NUM_PIXELS];
  static uint8_t i = 0;
  uint8_t j = i%NUM_PIXELS;

  if (!oneshot_flag && get_pin_voltage(BUTTON)>3) {
    display.clearDisplay();
    oneshot_flag = true;
    yaw = 0;
  } else if (oneshot_flag && get_pin_voltage(BUTTON) == 0) {
    oneshot_flag = false;
  }
  Serial.println(get_pin_voltage(BUTTON));
  if(get_pin_voltage(BUTTON)>3) {
    display.display();
    pixelsY[j] = -accel_y;
    pixelsX[j] = -yaw_int;
    //Delete the first points and replace them with the new coords
    // if (i >= NUM_PIXELS) { // Only start erasing after NUM_PIXELS points
    //   uint8_t oldIndex = (i - NUM_PIXELS+1) % NUM_PIXELS; // Find the oldest point
    //   display.fillCircle(pixelsX[oldIndex] + OFFSET_X, pixelsY[oldIndex] + OFFSET_Y, POINT_RADIUS, OFF); // Draw background color
    // }
    display.fillCircle(pixelsX[j]+OFFSET_X, pixelsY[j]+OFFSET_Y, POINT_RADIUS, ON);
    i++;



    // if(i == 150) {
    //   i = 0;
    //   display.clearDisplay();
    // }
  } else {
    uint8_t *buffer = display.getBuffer();
    for (int j = 0; j < 128; j++) {
    //   for (int k=0; k < 64; k++) {
    //     Serial.printf("%b\t", buffer[j*64 + k]);
    //   }
    //   Serial.println("\n");
    }

  }

  

}


int16_t get_yaw_value() {
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime)/1000; // Divide by 1000 to get seconds

  yaw =  yaw + (smooth_gyro_z * elapsedTime);

 return yaw;
}


void getAccelData() {
  Wire.beginTransmission(ACCEL_ADR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ACCEL_ADR, (size_t)(7*2), true);

  accel_x = Wire.read() << 8 | Wire.read() / ACCEL_DIVISOR;
  accel_y = Wire.read() << 8 | Wire.read() / ACCEL_DIVISOR;
  accel_z = Wire.read() << 8 | Wire.read() / ACCEL_DIVISOR;
  temp = Wire.read() << 8 | Wire.read();
  gyro_x = (Wire.read() << 8 | Wire.read()) / GYRO_DIVISOR + GyroErrorX;
  gyro_y = (Wire.read() << 8 | Wire.read()) / GYRO_DIVISOR + GyroErrorY;
  gyro_z = (Wire.read() << 8 | Wire.read());
  if ((abs(gyro_z) > 32000) || (abs(gyro_z) < 200)){
    smooth_gyro_z = 0;
  }
  else {
    if (abs(smooth_gyro_z) == 1) {
      smooth_gyro_z = 0;
    } else {
      smooth_gyro_z = gyro_z / GYRO_DIVISOR;
    }
  }
  get_yaw_value();
}

void printData() {
    display.clearDisplay();
    display.setTextSize(1);             // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE);        // Draw white text
    display.setCursor(0,0);             // Start at top-left corner
    display.print(F("aX: ")); display.println(accel_x);
    display.print(F("aY: ")); display.println(accel_y);
    display.print(F("aZ: ")); display.println(accel_z);
    display.print(F("gX: ")); display.println(gyro_x);
    display.print(F("gY: ")); display.println(gyro_y);
    display.print(F("gZ: ")); display.println(gyro_z);

    Serial.print(("aX: ")); Serial.print(accel_x);
    Serial.print(("aY: ")); Serial.print(accel_y);
    Serial.print(("aZ: ")); Serial.print(accel_z);
    Serial.print(("gX: ")); Serial.print(gyro_x);
    Serial.print(("gY: ")); Serial.print(gyro_y);
    Serial.print(("gZ: ")); Serial.println(gyro_z);


    display.display();
}

void CoordBoundsCheck(int16_t* x, int16_t* y) {
  float x1, y1; 
  x1 = *x;
  y1 = *y;

  if (x1 > ACCEL_MAX) {
    *x = ACCEL_MAX/X_DIVISOR;
  } else {
    *x = x1/X_DIVISOR;
  }
  if (y1 > ACCEL_MAX) {
    *y = ACCEL_MAX;
  } else {
    *y = y1;
  }

}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  uint8_t c = 0;
  // Read accelerometer values 200 times
  while (c < 200) {
    getAccelData();
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((accel_y) / sqrt(pow((accel_x), 2) + pow((accel_z), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (accel_x) / sqrt(pow((accel_y), 2) + pow((accel_z), 2))) * 180 / PI));
    GyroErrorX = GyroErrorX + (gyro_x);
    GyroErrorY = GyroErrorY + (gyro_y);
    GyroErrorZ = GyroErrorZ + (gyro_z);
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}



float get_gyro_smoothed() {
  static float smooth_gyro_z = gyro_z;
  float gyro = gyro_z;
  float alpha = 0.9; // alpha-filter constant
  if ((gyro > 200) && (gyro < 30000)) {
    // alpha filter all good measurements     
    smooth_gyro_z = alpha*smooth_gyro_z +(1-alpha)*gyro;
  } 
  return(smooth_gyro_z);
}








