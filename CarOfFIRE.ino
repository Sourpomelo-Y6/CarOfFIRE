/********************************************************
 * CarOfFIRE
 * Custamized From M5Bala balance car Basic Example
 * Reading encoder and writting the motor via I2C
 ********************************************************/

#include <M5Stack.h>

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "ydlidarTask.h"
#include "esp_log.h"

#include <Wire.h>
#include <WiFi.h>
//#include <WiFiClient.h>
//#include <WiFiAP.h>

#define MPU6050_ADDR 0x68
#define MPU6050_AX  0x3B
#define MPU6050_AY  0x3D
#define MPU6050_AZ  0x3F
#define MPU6050_TP  0x41    //  data not used
#define MPU6050_GX  0x43
#define MPU6050_GY  0x45
#define MPU6050_GZ  0x47

#define MOTOR_RPM           150
#define MAX_PWM             255
#define DEAD_ZONE           20

#define M5GO_WHEEL_ADDR     0x56
#define MOTOR_CTRL_ADDR     0x00
#define ENCODER_ADDR        0x04

int16_t AccX, AccY, AccZ;
int16_t Temp;
int16_t GyroX, GyroY, GyroZ;

int16_t speed_input0, speed_input1;
int16_t pwm_out0, pwm_out1;

// Set these to your desired credentials.
//const char * ssid = "your-ssid";
//const char * password = "your-password";
const char * ssid = "M5StackFIRE";
const char * password = "lets5ev9";

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
//const char * udpAddress = "192.168.4.255";
//const int udpPort = 80;
//const char * tcpAddress = "192.168.4.2";
//const int tcpPort = 80;

//WiFiUDP UDP;M5Stack使ってみた　その６「加速度＆ジャイロセンサ」（MPU6050）※10月30日追記

//#define BUFLEN 64
//char WiFibuf[BUFLEN];

//class WiFiHardware {
//  public:
//    WiFiClient client;
//    
//    WiFiHardware() {};
//    void init() {
//      this->client.connect(tcpAddress, tcpPort);    
//    }
//    
//    int read() {
//      if (this->client.connected()) {
//        return this->client.read();
//      } else {
//        this->client.connect(tcpAddress, tcpPort);
//      }
//      return -1;    
//    }
//    
//    void write(uint8_t* data, int length) {
//      for (int i = 0; i < length; i++){
//        Serial.print(data[i]);
//        this->client.write(data[i]);
//      }
//    }
//    unsigned long time() {return millis();}
//};

//typedef ros::NodeHandle_<WiFiHardware, 15, 15, 4096, 4096> NodeHandle;
ros::NodeHandle nh;


// Listener (For Debug)
void listenerCb(const std_msgs::String& str) {
  M5.Lcd.setCursor(0, 60);
  M5.Lcd.println(str.data);
}
ros::Subscriber<std_msgs::String> listener_sub("listener", &listenerCb);

// Command Velocity
void messageCb(const geometry_msgs::Twist& twist) {
  float linear_x = twist.linear.x;
  float angle_z = twist.angular.z;

  setMotor( (int16_t)(linear_x + angle_z),
            (int16_t)(linear_x - angle_z));
}
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", &messageCb);

// TF (For Odom)
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

// Chatter (For Debug)
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

// LaserScan
sensor_msgs::LaserScan lidar_msg;
ros::Publisher lidar_pub("laser_scan", &lidar_msg);

float dist[360];
uint8_t flag=0;

void onRecv(void *dummy){
    flag=1;
}

void setup() {
  // Power ON Stabilizing...
  delay(500);
  M5.begin();
  M5.setPowerBoostKeepOn(false);

  // Init I2C
  Wire.begin();
  Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
  delay(500);

  // Motor Init
  setMotor(0, 0);

  // IMU Init
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  // Clear Screen
  M5.Lcd.fillScreen(TFT_BLACK);
  
  //Serial.begin(115200);//
  Serial.println();
  Serial.println("Configuring access point...");

  M5.Lcd.println();
  M5.Lcd.println("Configuring access point...");

  // delete old config
  WiFi.disconnect(true);
  delay(500);
  
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  M5.Lcd.print("AP IP address: ");
  M5.Lcd.println(myIP);

  nh.initNode();
  nh.subscribe(listener_sub);
  nh.advertise(chatter);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(lidar_pub);  

  broadcaster.init(nh);

  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";

  // Set LaserScan Definition
  lidar_msg.header.frame_id = "lidar";
  lidar_msg.angle_min = 0.0;               // start angle of the scan [rad]
  lidar_msg.angle_max = 3.14*2;            // end angle of the scan [rad]
  lidar_msg.angle_increment = 3.14*2/360;  // angular distance between measurements [rad]
  lidar_msg.range_min = 0.3;               // minimum range value [m]
  lidar_msg.range_max = 50.0;                // maximum range value [m]

  lidar_msg.ranges_length = 360;
  lidar_msg.intensities_length = 0;

  ydlidar_begin(UART_NUM_2,17,16,onRecv,NULL);

  M5.Lcd.println("Start command is sent");
  while(!ydlidar_is_ready())vTaskDelay(1 / portTICK_PERIOD_MS);
  M5.Lcd.println("First packet comes! start!");

  //WiFi.config(myIP, WiFi.gatewayIP(), WiFi.subnetMask());
  //UDP.begin(80);

  //Serial.println("UDP started");
  //M5.Lcd.println("UDP started");

}

unsigned int old_millis = 0;

void loop() {
  // LCD display
  static uint32_t print_interval = millis() + 30;
  if (millis() > print_interval) {
    print_interval = millis() + 100;
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("Input  Encoer0: %+4d  Encoer1: %+4d    \r\n", 
                speed_input0, speed_input1);
    M5.Lcd.printf("Output PWM0: %+4d     PWM1: %+4d    \r\n", 
                pwm_out0, pwm_out1);
  }
//
//  if (UDP.parsePacket() > 0) {
//    UDP.read(WiFibuf, BUFLEN);
//    M5.Lcd.print(WiFibuf);
//    //UDP.flush();
//  }
//
//  //for debug
//  {
//    //Send a packet
//    UDP.beginPacket(udpAddress,udpPort);
//    //udp.printf("Seconds since boot: %u", millis()/1000);
//
//    UDP.write(AccX);//UDP.print(",");
//    UDP.write(AccY);//UDP.print(",");
//    UDP.write(AccZ);//UDP.print(",");
//    UDP.write(GyroX);//UDP.print(",");
//    UDP.write(GyroY);//UDP.print(",");
//    UDP.write(GyroZ);//UDP.print(",");
//    UDP.write(Temp);//UDP.print(",");
//    UDP.write(speed_input0);//UDP.print(",");
//    UDP.write(speed_input1);
//    
//    UDP.endPacket();
//  }

  static YDPacket data;
    
  ////////////////////////
  //Get packet data exsample
  ///////////////////////
  for(int l=0;l<30;l++){
      if(flag){
          flag=0;
          ydlidar_get(&data);
          for(int i=0;i<data.lsn;i++){
              int pos=(int)(ydpacket_angle(&data,i));
              pos=(pos+360)%360;
              dist[pos]=(ydpacket_len(&data,i))/1000.0;
          }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }

  if(millis() - old_millis > 1000){
    lidar_msg.ranges = dist;
    lidar_msg.header.stamp = nh.now();
    lidar_pub.publish(&lidar_msg);
    old_millis = millis(); 
  }

  readIMU();
  readEncoder();

  // tf odom->base_link
  
  t.transform.translation.x = speed_input0;
  t.transform.translation.y = speed_input1;
  
  t.transform.rotation = tf::createQuaternionFromYaw(GyroZ);
  t.header.stamp = nh.now();
  
  broadcaster.sendTransform(t);

  str_msg.data = "hello";
  chatter.publish( &str_msg );
  // 
  nh.spinOnce();
  // M5 Loop
  M5.update();
}

void setMotor(int16_t pwm0, int16_t pwm1) {
  // Value range
  int16_t m0 = constrain(pwm0, -255, 255);
  int16_t m1 = constrain(pwm1, -255, 255);

  // Dead zone
  if (((m0 > 0) && (m0 < DEAD_ZONE)) || ((m0 < 0) && (m0 > -DEAD_ZONE))) m0 = 0;
  if (((m1 > 0) && (m1 < DEAD_ZONE)) || ((m1 < 0) && (m1 > -DEAD_ZONE))) m1 = 0;

  // Same value
  static int16_t pre_m0, pre_m1;
  if ((m0 == pre_m0) && (m1 == pre_m1))
    return;
  pre_m0 = m0;
  pre_m1 = m1;

  // Send I2C
  Wire.beginTransmission(M5GO_WHEEL_ADDR);
  Wire.write(MOTOR_CTRL_ADDR); // Motor ctrl reg addr
  Wire.write(((uint8_t*)&m0)[0]);
  Wire.write(((uint8_t*)&m0)[1]);
  Wire.write(((uint8_t*)&m1)[0]);
  Wire.write(((uint8_t*)&m1)[1]);
  Wire.endTransmission();
}

void readEncoder() {
  static float _speed_input0 = 0, _speed_input1 = 0;
  int16_t rx_buf[2];

  //Get Data from Module.
  Wire.beginTransmission(M5GO_WHEEL_ADDR);
  Wire.write(ENCODER_ADDR); // encoder reg addr
  Wire.endTransmission();
  Wire.beginTransmission(M5GO_WHEEL_ADDR);
  Wire.requestFrom(M5GO_WHEEL_ADDR, 4);

  if (Wire.available()) {
    ((uint8_t*)rx_buf)[0] = Wire.read();
    ((uint8_t*)rx_buf)[1] = Wire.read();
    ((uint8_t*)rx_buf)[2] = Wire.read();
    ((uint8_t*)rx_buf)[3] = Wire.read();
    
    // filter
    _speed_input0 *= 0.9;
    _speed_input0 += 0.1 * rx_buf[0];
    _speed_input1 *= 0.9;
    _speed_input1 += 0.1 * rx_buf[1];
    
    speed_input0 = constrain((int16_t)(-_speed_input0), -255, 255);
    speed_input1 = constrain((int16_t)(_speed_input1), -255, 255);
  }
}

void readIMU() {
  //  send start address
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_AX);
  Wire.endTransmission();  
  //  request 14bytes (int16 x 7)
  Wire.requestFrom(MPU6050_ADDR, 14);
  //  get 14bytes
  AccX = Wire.read() << 8;  AccX |= Wire.read();
  AccY = Wire.read() << 8;  AccY |= Wire.read();
  AccZ = Wire.read() << 8;  AccZ |= Wire.read();
  Temp = Wire.read() << 8;  Temp |= Wire.read();  //  (Temp-12421)/340.0 [degC]
  GyroX = Wire.read() << 8; GyroX |= Wire.read();
  GyroY = Wire.read() << 8; GyroY |= Wire.read();
  GyroZ = Wire.read() << 8; GyroZ |= Wire.read();
}
