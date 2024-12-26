#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7
// #define Enc_Z PA0

int count = 0;
int prev_count = 0;
int rotations = 0;
int rotations_enc = 0;
void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count -= 1;
  }
  else {
    count += 1;
  }

  // if (count < -3600){
  //   rotations += 1;
  //   prev_count = count;
  //   count = 0;
  // }
  // else if (count > 3600){
  //   rotations -= 1;
  //   prev_count = count;
  //   count = 0;
  // }
  

}

// void COUNT_R(){
//   if (prev_count < -3600){
//       rotations_enc += 1;
//       prev_count = 0;
//   }
//   else if (prev_count > 3600){
//       rotations_enc -= 1;
//       prev_count = 0;

//   }
  
// }

void setup() {
  pinMode(LED1, OUTPUT);  //PD2
  pinMode(LED2, OUTPUT);  //PA5

  pinMode(Enc_B, INPUT);
  pinMode(Enc_Z, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Enc_Z), COUNT_R, CHANGE);

  Serial.begin(115200);

}

void loop() {
  // Serial.print(digitalRead(Enc_A));
  // Serial.print(" ");
  // Serial.print(digitalRead(Enc_B));
  Serial.print(count);
  Serial.print(" ");
  Serial.print(rotations_enc);
  Serial.print(" ");
  Serial.println(rotations);


}
