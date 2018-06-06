#define PWM1 3
#define PWM2 5
#define PWM3 6
#define PWM4 9

#define debug1 2
#define debug2 4

#define SCL A5
#define SDA A4

#define CE 7
#define CSN 8
#define MOSI 11
#define MISO 12

void setup() {
  //setup_gyro();
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(debug1,OUTPUT);
  pinMode(debug2,OUTPUT);
}

void loop() {
  //loop_gyro();
  digitalWrite(debug1,HIGH);
  digitalWrite(debug2,HIGH);

  analogWrite(PWM4,10);
  analogWrite(PWM3,10);
  analogWrite(PWM2,10);
  analogWrite(PWM1,10);
  delay(1000);

  digitalWrite(debug1,LOW);
  digitalWrite(debug2,LOW);

  delay(100);
}

// sensor fusion stuff
#include "sensor_fusion.h"
// DEFINE INT_STATUS IN HEADER

struct vector bias_a, bias_g, orient, orient_c;
int max_Samples = 75;
unsigned long timel = 0;
unsigned long prev_time = 0;

struct data{
  int x,y,z;
} acc, gyro;

void setup_gyro() {
  //configure device
  //set PWR_MGMT_1 register to take the IMU out of sleep mode
  uint8_t pwr_mgmt_1;
  readReg(0x6B, &pwr_mgmt_1, 1);
  pwr_mgmt_1 = 0xBF & pwr_mgmt_1; //disable SLEEP
  writeReg(0x6B, &pwr_mgmt_1, 1);
  //set GYRO_CONFIG register to the largest possible full-scale range to enable the
  //detection of high-velocity rotations
  uint8_t gyro_config;
  readReg(0x1B, &gyro_config, 1);
  gyro_config = 0x18 | gyro_config; //highest thingy
  writeReg(0x1B, &gyro_config, 1);
  //set CONFIG register to the largest possible bandwidth
  uint8_t configy;
  readReg(0x1A, &configy, 1);
  configy = 0xF8 & configy;
  writeReg(0x1A, &configy, 1);
  //enable DATA_RDY
//  uint8_t RDY;
//  readReg(0x38, &RDY, 1);
//  RDY = RDY | 0x01;
//  writeReg(0x38, &RDY, 1);

  Serial.begin(115200);
  //uint8_t *WHOAMI;
  //readReg(0x75,WHOAMI,1);
  //Serial.println(String(*WHOAMI));

  bias_a.x = bias_a.y = bias_a.z = 0;
  bias_g.x = bias_g.y = bias_g.z = 0;

  // Get samples
  for (int i = 1; i <= max_Samples; i++) {

    if (getData()) {

      bias_a.x += ((float)acc.x)/16384 - 0;
      bias_a.y += ((float)acc.y)/16384 - 0;
      bias_a.z += ((float)acc.z)/16384 - 1;

      bias_g.x += ((float)gyro.x)/16.384 - 0;
      bias_g.y += ((float)gyro.y)/16.384 - 0;
      bias_g.z += ((float)gyro.z)/16.384 - 0;

      //Serial.println("Loop" + String(i));
      //printVector(bias_a);
      //printVector(bias_g);
      //printVector(acc,"acc");
      //printVector(gyro,"gyro");
    }
    else {
      //Serial.println("No data");
      i--;
    }
  }

  bias_a.x /= max_Samples;
  bias_a.y /= max_Samples;
  bias_a.z /= max_Samples;

  bias_g.x /= max_Samples;
  bias_g.y /= max_Samples;
  bias_g.z /= max_Samples;

//  printVector(bias_a);
//  printVector(bias_g);
  //init orient to up
  orient.x = orient.y = 0;
  orient.z = 1;

  orient_c.x = orient_c.y = 0;
  orient_c.z = 1;
}



void loop_gyro() {
  if (getData()) {
    //find unit acceleration vector
    struct vector a=scaleReading(acc,"acc");
    vector unit_a;
    vector_normalize(&a, &unit_a);
    printVector(unit_a);

    //rotate quaternion to show orientation
    struct vector g = scaleReading(gyro,"gyro");
    vector unit_g;
    float len = vector_normalize(&g, &unit_g);
    struct quaternion q;
    timel = micros();
    //Serial.print(" ");
    //Serial.println(len,4);

    //deal with init case
    if(prev_time == 0) len = 0;

    float timebt = (timel-prev_time)/1000000.0;
//    Serial.println(timebt,4);
    quaternion_create(&unit_g, -len*timebt, &q);
    prev_time = timel;

    vector result;
    quaternion_rotate(&orient, &q, &result);
    orient = result;
    Serial.print(" ");
    printVector(orient);

    //complementary filter

    //apply low/high pass filter

    //add and normalize
    vector comp,sum,p1,p2,r;
    
    quaternion_rotate(&orient_c, &q, &r);
    float alpha = 0.5;
    vector_multiply(&unit_a, alpha, &p1);
    vector_multiply(&r, 1.0-alpha, &p2);
    vector_add(&p1, &p2, &sum);
    vector_normalize(&sum, &comp);

    Serial.print(" ");
    printVector(comp);
    Serial.println();

    orient_c = comp;
  }

  
  delay(10);
}

void printVector(struct vector v) {
  Serial.print(v.x);
  Serial.print(" ");
  Serial.print(v.y);
  Serial.print(" ");
  Serial.print(v.z);
  //Serial.println();
}

struct vector scaleReading(struct data a, String s){
  struct vector v;
  if(s=="acc"){
    v.x=((float)a.x)/16384-bias_a.x;
    v.y=((float)a.y)/16384-bias_a.y;
    v.z=((float)a.z)/16384-bias_a.z;
  }
  else {
    v.x=((float)a.x)/16.384-bias_g.x;
    v.y=((float)a.y)/16.384-bias_g.y;
    v.z=((float)a.z)/16.384-bias_g.z;
    v.x *= 0.017453;
    v.y *= 0.017453;
    v.z *= 0.017453;
  }
  return v;
}

//convert to g's and print
void printVector(struct data v, String s) {
  if (s=="acc"){
    Serial.print((((float)v.x)/16384),4);
    Serial.print(" ");
    Serial.print((((float)v.y)/16384),4);
    Serial.print(" ");
    Serial.print((((float)v.z)/16384),4);
    Serial.println();
  }
  else {
    Serial.print((((float)v.x)/16.4),4);
    Serial.print(" ");
    Serial.print((((float)v.y)/16.4),4);
    Serial.print(" ");
    Serial.print((((float)v.z)/16.4),4);
    Serial.println();
  }
}

/* Get data about x y z coordinates from accelerometer and gyroscope.
   Takes in two structs, acc and gyro. If data is available, acc and gyro
   will be filled with the x y z coordinates and the function returns true.
   Otherwise it returns false and structs are not touched.*/
bool getData() {
  uint8_t buf,buf2;
  readReg(0x3A, &buf, 1);
  //Serial.println(String(buf));
  if ((buf)&0x01) {
    //read acc
    readReg(0x3B, &buf, 1);
    readReg(0x3C, &buf2, 1);
    acc.x = (buf<<8) | buf2;
    readReg(0x3D, &buf, 1);
    readReg(0x3E, &buf2, 1);
    acc.y = (buf<<8) | buf2;
    readReg(0x3F, &buf, 1);
    readReg(0x40, &buf2, 1);
    acc.z = (buf<<8) | buf2;

    //read gyro
    readReg(0x43, &buf, 1);
    readReg(0x44, &buf2, 1);
    gyro.x = (buf<<8) | buf2;
    readReg(0x45, &buf, 1);
    readReg(0x46, &buf2, 1);
    gyro.y = (buf<<8) | buf2;
    readReg(0x47, &buf, 1);
    readReg(0x48, &buf2, 1);
    gyro.z = (buf<<8) | buf2;

    return true;
  }
  return false;
}
