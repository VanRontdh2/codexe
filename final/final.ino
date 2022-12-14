#define dir 2
#define pwm 9
#define en 4
#define ste 5
#define ctht 6

//xxxxxxxxxxxxxxxxxxxxxMPU6050xxxxxxxxxxxxxx
#include <Wire.h>
#include <MPU6050_tockn.h>
MPU6050_tockn mpu6050_1(Wire);

#include <mpu6050.h>
MPU6050 mpu6050_2;
void mpu6050_begin()  {

  Serial.print("MPU6050: Starting calibration; leave device flat and still ... ");
  int error= mpu6050_2.begin(); 
  Serial.println(mpu6050_2.error_str(error));
}

float mpu6050_yaw() {
  MPU6050_t data= mpu6050_2.get();
  while( data.dir.error!=0 ) { 
    // I suffer from a lot of I2C problems
    Serial.println(mpu6050_2.error_str(data.dir.error));
    // Reset I2C
    TWCR= 0; Wire.begin();
    // Reread
    data= mpu6050_2.get();
  }
  return data.dir.yaw;
}

//unsigned long gt;
int dem, dem1;
int last=0, tt=0;
int n, n1;
int bit_giua = 0;
int bit_cao = 0;
int bit_thap =0;
int sp,sp1, pv, steer;

unsigned long current_time, timer_1;
struct RX{
  volatile unsigned long gt;
  unsigned long pul;
  char pre_stt_channel;
};
RX rx;
// phat xung cho driver dong co
void pto(){
  dem1 =0;
  while (dem1<1000){
         digitalWrite(ste, HIGH);
         delayMicroseconds(30);
         digitalWrite(ste, LOW);
         delayMicroseconds(30);
         dem1++;}
}

//xxxxxxxxxxxxxxxxxxxxxPIDxxxxxxxxxxxxxxx
#define PID_K_p 5
#define PID_K_i  0
#define PID_K_d  0

float i_input;
float d_last;

void pid_begin() {
  i_input= 0;
  d_last= 0;  
  Serial.println("PID    : ok");
}

int pid(float error) {
  float p_input;
  float d_input;
    
  p_input= error;
  i_input= constrain(i_input+error,-50,+50);
  d_input= error-d_last; d_last=error;

  return p_input*PID_K_p + i_input*PID_K_i + d_input*PID_K_d;
}
void motor (int delta){
  if (delta > 1)
  {digitalWrite(en, LOW);
    digitalWrite(dir, HIGH);
    pto();
}
  if (delta<1 && delta>-1){
    digitalWrite(en, HIGH);
  }
  if (delta < -1){
    digitalWrite(en, LOW);
    digitalWrite(dir, LOW);
    pto();
  }}

void setup() {
  // put your setup code here, to run once:
pinMode(ste, OUTPUT);
pinMode(dir, OUTPUT);
pinMode(en, OUTPUT);
pinMode(pwm, INPUT);
pinMode(ctht, INPUT);
// attachInterrupt(1, xung, FALLING); 

Serial.begin(9600);

 Wire.begin();
  mpu6050_1.begin();
  mpu6050_1.calcGyroOffsets(true);


 mpu6050_begin();
    pid_begin();
  //timer = millis();
   PCICR = 0x00;          // Set bit PCIE0 = 1 trong thanh ghi PCICR ????? k??ch ho???t ng???t thay ?????i tr???ng th??i ??? c??c ch??n B0,1,2,3,4,5,6,7 tr??n atmega328P
  PCICR = 0x01;

  PCMSK0 = 0x00;         // Set bit PCINT0,1,2,3,4,5 = 1 trong thanh ghi PCMSK0 ????? k??ch ho???t ng???t thay ?????i tr???ng th??i tr??n ch??n B0,1,2,3,4,5 c???a atmega328 (ch??n 8,9,10,11,12,13 tr??n arduino) 
  PCMSK0 = B00111111;

//???????????????????????
while(rx.gt<1210 || rx.gt>1360 && ctht != 1 ){
  delayMicroseconds(1000);
}


}

void loop() {
 
   Serial.print("\tangleZ : ");
  Serial.println(mpu6050.getAngleZ());
  
  rx.pul = rx.gt;
  Serial.println(rx.pul);
  //tao gioi han gia tri cho xung
  if (rx.pul >1660) rx.pul =1660;
  if (rx.pul<920) rx.pul =920;
  //xxxxxxxxxxxxxxxxxx giua xxxxxxxxxxxxxxxxxxxx
  if (rx.pul>1220 && rx.pul<1360){
   Serial.println("GIUA GIUA GIUA GIUA GIUA GIUA");
  mpu6050_1.update();
  sp = 0;
  pv = mpu6050_yaw()-mpu6050_1.getAngleZ();
  steer = pid (sp -pv);
  motor(steer);
  }
   //xxxxxxxxxxxxxxxxxx thap xxxxxxxxxxxxxxxxxx
  if (rx.pul <1220){
    Serial.println("THAP THAP THAP THAP THAP THAP");
    tt = (rx.pul-1220)/10;
    sp = 2*tt;
    mpu6050_1.update();
    pv = mpu6050_yaw()-mpu6050_1.getAngleZ();
    // gioi han banh re la 60 do
     if (pv<(sp-60)){
        digitalWrite(en, HIGH);
  }
else{
  steer = pid (sp -pv);
  motor(steer);
}
         
  }
  //xxxxxxxxxxxxxxxxxxxxxx cao xxxxxxxxxxxxxxxxxxxxxx
  if (rx.pul>1360){
    Serial.println("CAO CAO CAO CAO CAO CAO CAO");
    tt = (rx.pul-1360)/10;
    sp = 2*tt;
    mpu6050_1.update();
    pv = mpu6050_yaw()-mpu6050_1.getAngleZ();
    // gioi han banh re la 60 do

     if (pv>(sp+60)){
        digitalWrite(en, HIGH);
  }
else {
  steer = pid (sp -pv);
  motor(steer);
}
     
  }


}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx chuong trinh ngat xxxxxxxxxxxxxxxxxxxxxxxxxxx

   ISR(PCINT0_vect){                                                       // H??m ng???t thay ?????i tr???ng th??i c???a ch??n9, tr??n arduino ????? ?????c xung t??? TX ?????n RX
  current_time = micros();                                              // Khi c?? ng???t thay ?????i tr???ng th??i ??? b???t k?? ch??n n??o ch??n 9  th?? s??? v??o ng???t
                                                                        // L??u l???i m???c th???i gian khi x???y ra ng???t
  //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX chan 9 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  if(PINB & B00000010){                                                 // Ki???m tra ch??n 9 n???u ??? m???c 1
    if(rx.pre_stt_channel == 0){                                     // N???u tr?????c ???? ch??n 9 ??? m???c 0 (x??t xung c???nh l??n)
      rx.pre_stt_channel = 1;                                        // Cho bi???n tr???ng th??i ch??n 9 = 1
      timer_1 = current_time;                                           // L??u th???i ??i???m b???t ?????u l??n m???c 1 v??o bi???n timer_1
    }
  }
  else if(rx.pre_stt_channel == 1){                                  // Ki???m tra ch??n 9 n???u ??? m???c 0 v?? tr?????c ???? l?? m???c 1
    rx.pre_stt_channel = 0;                                          // Cho bi???n tr???ng th??i ch??n 9 = 0
    rx.gt = current_time - timer_1;                  // T??nh ra ????? r???ng xung k??nh roll = th???i ??i???m b???t ?????u xu???ng m???c 0 tr??? cho th???i ??i???m b???t ?????u l??n m???c 1
  }
   }
