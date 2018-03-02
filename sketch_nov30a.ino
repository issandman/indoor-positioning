#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
MPU6050 accelgyro;

unsigned long now , lastTime = 0;
float dt;  //微分时间

float aax,aay,aaz;//加速度计求出来的角姿态
float a_pitch[10]={0},a_roll[10]={0},a_yaw[10]={0}; //pitch，roll，yaw方差计算数组
float S_pitch , S_roll , S_yaw;  //pitch,roll,yaw观测值均值
float R_pitch , R_roll , R_yaw;  //pitch,roll,yaw观测值方差
float P_pitch = 1 , P_roll = 1 , P_yaw = 1; //状态量初始方差
float K_pitch , K_roll , K_yaw;  //卡尔曼增益
float Q_pitch = 0.0025 , Q_roll = 0.0025 , Q_yaw = 0.0025; //过程测量噪声

//滑动均值加权滤波
uint8_t n_sample = 8;          //采样个数
long aax_sum,aay_sum,aaz_sum;  //采样和
float aaxs[8]={0},aays[8]={0},aazs[8]={0}; //加速度角采样


//未处理的数据
int16_t ax,ay,az;
int16_t gx,gy,gz;
//处理过后的数据
float accx,accy,accz;
float gyrox,gyroy,gyroz;
//加速计和陀螺仪零飘量
float axo=0,ayo=0,azo=0;
float gxo=0,gyo=0,gzo=0;
float ainitial = 0;     //零飘均值
//精度系数
float AcceRatio = 16384.0f;
float GyroRatio = 131.0f;
float Pi = 3.1415926f;
float Rad = 180.0/Pi;

//PID系数,比例系数和积分系数
#define Kp 0.1f;
#define Ki 0.02f;

float q0=1,q1=0,q2=0,q3=0;      //初始四元数
float exInt=0,eyInt=0,ezInt=0;
float Angle_roll,Angle_pitch,Angle_yaw;

//四元数微分方程，结果转换为欧拉角
void AngleUpdate(float ax,float ay,float az,float gx,float gy,float gz,float dt){
  float norm;
  float vx,vy,vz;
  float ex,ey,ez;

  //if(ax*ay*az==0){return;}

  norm = sqrt(ax*ax+ay*ay+az*az);  //加速度向量单位化
  ax = ax/norm;
  ay = ay/norm;
  az = az/norm;

  vx = 2*(q1*q3 - q0*q2);  //t-1时刻的加速度向量
  vy = 2*(q0*q1 + q3*q2);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  //t-1时刻与t时刻加速度向量积
  ex = ay*vz-az*vy;
  ey = az*vx-ax*vz;
  ez = ax*vy-ay*vx;

  //误差积分
  exInt = exInt+ex*Ki;
  eyInt = eyInt+ey*Ki;
  ezInt = ezInt+ez*Ki;

  //经过PID补偿后的陀螺仪数据
  //如果没有这一块PID补偿效果会这么样
  gx = gx+ex*Kp+exInt;
  gy = gy+ey*Kp+eyInt;
  gz = gz+ez*Kp+ezInt;
  
  //四元数微分方程
  float q0_last = q0;
  float q1_last = q1;
  float q2_last = q2;
  float q3_last = q3;

  dt /= 2;
  q0 = q0_last + ( -q1_last*gx - q2_last*gy - q3_last*gz)*dt;
  q1 = q1_last + (q0_last*gx + q2_last*gz - q3_last*gy)*dt;
  q2 = q2_last + (q0_last*gy - q1_last*gz + q3_last*gx)*dt;
  q3 = q3_last + (q0_last*gz + q1_last*gy - q2_last*gx)*dt;
  
  //四元数标准化
  norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0 = q0/norm;
  q1 = q1/norm;
  q2 = q2/norm;
  q3 = q3/norm; 

  //四元数转化为欧拉角,弧度值转为角度值
  Angle_roll = atan2(2*q2*q3+2*q0*q1,-2*q1*q1-2*q2*q2+1) * Rad;
  Angle_pitch = asin(-2*q1*q3+2*q0*q2) * Rad;
  Angle_yaw = atan2(2*q1*q2+2*q0*q3,q0*q0+q1*q1-q2*q2-q3*q3) * Rad;

  //由加速度计计算得到的姿态角,注意方向是否于上吻合，正负
  aax = -(atan(ax / az) * Rad);  //pitch
  aay = atan(ay / az) * Rad;     //roll
  aaz = atan(az / ay) * Rad;     //yaw

  //开始滑动均值滤波
  aax_sum = 0;
  aay_sum = 0;
  aaz_sum = 0;
  for(int i=0;i<n_sample;i++){
    aaxs[i-1] = aaxs[i];
    aax_sum += aaxs[i] * i;
    aays[i-1] = aays[i];
    aay_sum += aays[i] * i;
    aazs[i-1] = aazs[i];
    aaz_sum += aazs[i] * i;
  }
  aaxs[n_sample-1] = aax;
  aax_sum += aax * n_sample;
  aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0;
  aays[n_sample-1] = aay;
  aay_sum += aay * n_sample;
  aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
  aazs[n_sample-1] = aaz;
  aaz_sum += aaz * n_sample;
  aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;
  
  //计算加速度计姿态角(观测值的)方差
  S_pitch = 0; R_pitch = 0;
  S_roll = 0;  R_roll = 0;
  S_yaw = 0;   R_yaw = 0;
  for(int i=1;i<10;i++){
    a_pitch[i-1] = a_pitch[i];
    S_pitch += a_pitch[i];
    a_roll[i-1] = a_roll[i];
    S_roll += a_roll[i];
    a_yaw[i-1] = a_yaw[i];
    S_yaw += a_yaw[i];
  }
  a_pitch[9] = aax;
  S_pitch += aax;
  S_pitch /= 10;  //pitch观测值的均值
  a_roll[9] = aay;
  S_roll += aay;
  S_roll /= 10;  //roll观测值的均值
  a_yaw[9] = aaz;
  S_yaw += aaz;
  S_yaw /= 10;  //yaw观测值的均值
  for(int i=0;i<10;i++){
    R_pitch += sq(a_pitch[i] - S_pitch);
    R_roll += sq(a_roll[i] - S_roll);
    R_yaw += sq(a_yaw[i] - S_yaw);
  }
  R_pitch = R_pitch / 9;  //得到方差
  R_roll = R_roll / 9;
  R_yaw = R_yaw / 9;

  //pitch角卡尔曼数据融合
  P_pitch = P_pitch + Q_pitch;
  K_pitch = P_pitch / (P_pitch+R_pitch);
  Angle_pitch = Angle_pitch + K_pitch*(aax - Angle_pitch);
  P_pitch = (1 - K_pitch) * P_pitch;
  //roll角卡尔曼数据融合
  P_roll = P_roll + Q_roll;
  K_roll = P_roll / (P_roll+R_roll);
  Angle_roll = Angle_roll + K_roll*(aay - Angle_roll);
  P_roll = (1 - K_roll) * P_roll;
  //yaw角卡尔曼融合
  P_yaw = P_yaw + Q_yaw;
  K_yaw = P_yaw / (P_yaw+R_yaw);
  Angle_yaw = Angle_yaw + K_yaw*(aaz - Angle_yaw);
  P_yaw = (1 - K_yaw) * P_yaw;
  
  Angle_roll /= Rad; Angle_pitch /= Rad; Angle_yaw /= Rad;

  //欧拉角到四元数
  q0 = cos(Angle_roll/2)*cos(Angle_pitch/2)*cos(Angle_yaw/2)+sin(Angle_pitch/2)*sin(Angle_roll/2)*sin(Angle_yaw/2);
  q1 = sin(Angle_roll/2)*cos(Angle_pitch/2)*cos(Angle_yaw/2)-cos(Angle_pitch/2)*sin(Angle_roll/2)*sin(Angle_yaw/2);
  q2 = cos(Angle_roll/2)*sin(Angle_pitch/2)*cos(Angle_yaw/2)+sin(Angle_pitch/2)*cos(Angle_roll/2)*sin(Angle_yaw/2);
  q3 = cos(Angle_roll/2)*cos(Angle_pitch/2)*sin(Angle_yaw/2)-sin(Angle_pitch/2)*sin(Angle_roll/2)*cos(Angle_yaw/2);

  //四元数标准化
  norm=sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
  q0 = q0/norm;
  q1 = q1/norm;
  q2 = q2/norm;
  q3 = q3/norm; 
}

void setup(){
  Wire.begin();
  Serial.begin(9600);
  accelgyro.initialize();
  
  //去除零飘
  unsigned short times = 400;
  for(int i=0;i<times;i++){
    accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
    axo += ax;ayo += ay;azo += az;
    gxo += gx;gyo += gy;gzo += gz;
  }
  axo/=times;ayo/=times;azo/=times;  //加速度计数据偏移量
  gxo/=times;gyo/=times;gzo/=times;  //陀螺仪数据偏移量
  ainitial = (axo + ayo)/2;
}

void loop(){
  now = millis();  //当前时间
  dt = (now - lastTime) /1000.0; //微分时间
  lastTime = now;
  
  accelgyro.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);
  accx = (ax-ainitial)/AcceRatio;  //补偿偏移量的数据
  accy = (ay-ainitial)/AcceRatio;
  accz = (az-ainitial)/AcceRatio;
  gyrox = (gx-gxo)/GyroRatio;
  gyroy = (gy-gyo)/GyroRatio;
  gyroz = (gz-gzo)/GyroRatio;
  AngleUpdate(accx,accy,accz,gyrox,gyroy,gyroz,dt);

  Serial.print(Angle_roll*Rad);Serial.print(",");
  Serial.println(Angle_pitch*Rad);Serial.print(",");
  Serial.println(Angle_yaw*Rad);
}
