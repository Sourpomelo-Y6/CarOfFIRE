# CarOfFIREForROS

    Customized M5BALA with M5StackFIRE

## Necessary ROS package and change

* rosserial
* rosserial_arduino

### rosserial

file:(catkin_workspece)/rosserial/rosserial_server/include/rosserial_server/session.h

```C++
-enum { buffer_max = 1023 };
+enum { buffer_max = 2047 };
```

Changed after catkin_make command.

### rosserial_arduino

file:(Arduino_libraries)/ros_lib/ros.h

```C++
-  typedef NodeHandle_<ArduinoHardware> NodeHandle; // default 25, 25, 512, 512

+  class WiFiHardware {
+      char * tcpAddress = "192.168.4.2";
+      int tcpPort = 80;    
+    public:
+      WiFiClient client;
+      
+      WiFiHardware() {};
+      void init() {
+        this->client.connect(tcpAddress, tcpPort);    
+      }
+      
+      int read() {
+        if (this->client.connected()) {
+          return this->client.read();
+        } else {
+          this->client.connect(tcpAddress, tcpPort);
+        }
+        return -1;    
+      }
+      
+      void write(uint8_t* data, int length) {
+        for (int i = 0; i < length; i++){
+          Serial.print(data[i]);
+          this->client.write(data[i]);
+        }
+      }
+      unsigned long time() {return millis();}
+  };
+
+  typedef NodeHandle_<WiFiHardware, 15, 15, 4096, 4096> NodeHandle;
```  


## Reference

Arduino IDE Setup</br>
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

M5Bala</br>
https://github.com/m5stack/M5Bala

M5Stack使ってみた　その６「加速度＆ジャイロセンサ」（MPU6050）※10月30日追記</br>
https://ameblo.jp/ruru12245/entry-12404525191.html


YDLidar X4 をESP32で使ってみた</br>
https://blog.akirayou.net/2018/06/ydlidar-x4-esp32.html
bitbucket</br>
https://bitbucket.org/akira_you/esp-ydlidarx4/src/master/
