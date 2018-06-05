#include <Arduino.h>
#include <Wire.h>
#include "sensor_fusion.h"

void readReg(uint8_t reg, uint8_t *buf, size_t len)
{
    // TODO: Implement
	//page 36
	Wire.beginTransmission(0x68); //start condition?
	Wire.write(reg); 
	Wire.endTransmission(false); 
	
	//restarts
	//Wire.endTransmission(false);
	Wire.requestFrom(0x68,len); //request the reg
	for (int i = 0; i < len; i++) {
		//while(!Wire.available()){}
		buf[i] = Wire.read();
	}
	Wire.endTransmission();
	
}

void writeReg(uint8_t reg, uint8_t *buf, size_t len)
{
    // TODO: Implement
	//page 35
	
	Wire.beginTransmission(byte(0x68)); // send AD+W = 0x68 + W bit (0)
	Wire.write(reg); // send RA
	Wire.write(buf,len); // send DATA
	Wire.endTransmission(true); //send stop signal
}

float vector_normalize(struct vector *raw, struct vector *unit)
{
  float mag = sqrt(pow(raw->x, 2) + pow(raw->y, 2) + pow(raw->z, 2));
  if(mag != 0) {
      unit->x = raw->x / mag;
      unit->y = raw->y / mag;
      unit->z = raw->z / mag;
  }
  return mag;
}

void vector_add(struct vector *v1, struct vector *v2, struct vector *result)
{
  result->x = v1->x + v2->x;
  result->y = v1->y + v2->y;
  result->z = v1->z + v2->z;
}

void vector_multiply(struct vector *v, float c, struct vector *result)
{
  result->x = v->x * c;
  result->y = v->y * c;
  result->z = v->z * c;
}

void quaternion_create(struct vector *v, float angle, struct quaternion *result)
{
  result->r = cos(angle/2);
  result->i = v->x * sin(angle/2);
  result->j = v->y * sin(angle/2);
  result->k = v->z * sin(angle/2);
}

void quaternion_rotate(struct vector *v, struct quaternion *q, struct vector *result)
{
  struct vector n;
  memcpy(&n, v, sizeof(n));
  result->x = n.x * (1 - 2*(pow(q->j, 2) + pow(q->k, 2))) + n.y * 2*(q->i * q->j - q->k * q->r) + n.z * 2*(q->i * q->k + q->j * q->r);
  result->y = n.x * 2*(q->i * q->j + q->k * q->r) + n.y * (1 - 2*(pow(q->i, 2) + pow(q->k, 2))) + n.z * 2*(q->j * q->k - q->i * q->r);
  result->z = n.x * 2*(q->i * q->k - q->j * q->r) + n.y * 2*(q->j * q->k + q->i * q->r) + n.z * (1 - 2*(pow(q->i, 2) + pow(q->j, 2)));
}

void quaternion_multiply(struct quaternion *q1, struct quaternion *q2, struct quaternion *result)
{
  result->r = q1->r * q2->r - q1->i * q2->i - q1->j * q2->j - q1->k * q2->k;
  result->i = q1->r * q2->i + q1->i * q2->r + q1->j * q2->k - q1->k * q2->j;
  result->j = q1->r * q2->j - q1->i * q2->k + q1->j * q2->r + q1->k * q2->i;
  result->k = q1->r * q2->k + q1->i * q2->j - q1->j * q2->i + q1->k * q2->r;
}

float vector_roll(struct vector *v)
{
  return atan2(v->x, sqrt(pow(v->z, 2) + pow(v->y, 2)));
}

float vector_pitch(struct vector *v)
{
  return -atan2(v->y, v->z);
}


