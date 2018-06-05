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
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(debug1,OUTPUT);
  pinMode(debug2,OUTPUT);
}

void loop() {
  digitalWrite(debug1,HIGH);
  digitalWrite(debug2,HIGH);

  for(int i = 0; i < 100; i++){
    analogWrite(PWM1,i);
    delay(10);
  }

  digitalWrite(debug1,LOW);
  digitalWrite(debug2,LOW);

  delay(100);
}
