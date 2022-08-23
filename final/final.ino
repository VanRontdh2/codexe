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
   PCICR = 0x00;          // Set bit PCIE0 = 1 trong thanh ghi PCICR để kích hoạt ngắt thay đổi trạng thái ở các chân B0,1,2,3,4,5,6,7 trên atmega328P
  PCICR = 0x01;

  PCMSK0 = 0x00;         // Set bit PCINT0,1,2,3,4,5 = 1 trong thanh ghi PCMSK0 để kích hoạt ngắt thay đổi trạng thái trên chân B0,1,2,3,4,5 của atmega328 (chân 8,9,10,11,12,13 trên arduino) 
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

   ISR(PCINT0_vect){                                                       // Hàm ngắt thay đổi trạng thái của chân9, trên arduino để đọc xung từ TX đến RX
  current_time = micros();                                              // Khi có ngắt thay đổi trạng thái ở bất kì chân nào chân 9  thì sẽ vào ngắt
                                                                        // Lưu lại mốc thời gian khi xảy ra ngắt
  //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX chan 9 XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
  if(PINB & B00000010){                                                 // Kiểm tra chân 9 nếu ở mức 1
    if(rx.pre_stt_channel == 0){                                     // Nếu trước đó chân 9 ở mức 0 (xét xung cạnh lên)
      rx.pre_stt_channel = 1;                                        // Cho biến trạng thái chân 9 = 1
      timer_1 = current_time;                                           // Lưu thời điểm bắt đầu lên mức 1 vào biến timer_1
    }
  }
  else if(rx.pre_stt_channel == 1){                                  // Kiểm tra chân 9 nếu ở mức 0 và trước đó là mức 1
    rx.pre_stt_channel = 0;                                          // Cho biến trạng thái chân 9 = 0
    rx.gt = current_time - timer_1;                  // Tính ra độ rộng xung kênh roll = thời điểm bắt đầu xuống mức 0 trừ cho thời điểm bắt đầu lên mức 1
  }
   }
