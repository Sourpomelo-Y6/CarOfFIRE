#ifndef YDLIDAR_TASK_H
#define YDLIDAR_TASK_H
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"

#define YLIDAR_TASK_PRIO 3
#define YLIDAR_TASK_CORE 0 
#define YDLIDAR_USE_PACKET_DATA 1
#define YDLIDAR_USE_SUMMARY_DATA 1

#ifdef __cplusplus
extern "C"{
#endif
///////////////////
//C and C++ section
//////////////////
#define MaxLSN 40
typedef struct  {
    uint8_t lsn;//Length of Sensing Data
    uint16_t fsa;//start angle
    uint16_t lsa;//last angle
    uint16_t s[MaxLSN];
    ////// set by prepare();
    float startAng,endAng,stepAng;
}YDPacket;
#define YDLIDAR_SUMMARY_LEN 32
typedef uint16_t ydlidar_summary_t[YDLIDAR_SUMMARY_LEN];
///////////////////////////////////////////
///Simple C Language interface (for single YDLIDAR)
//////////////////////////////////////////
//start ydlidar
void ydlidar_begin(uart_port_t uartNum,int txPin,int rxPing,void (*_onRecv)(void *),void *_onRecvData);
//get current data
#if YDLIDAR_USE_PACKET_DATA
bool ydlidar_get(YDPacket *outData);
#endif
//get current summary data
#if YDLIDAR_USE_SUMMARY_DATA
bool ydlidar_get_summary(ydlidar_summary_t sum);
#endif
//get tick count form last packet
portTickType ydlidar_get_tickcount_from_last();
//ydlidar begin and 1st packet already comes. 
bool ydlidar_is_ready();
//parse fail count 
uint8_t ydlidar_fail_count();
//////////////////
//data packet calulation API
//////////////////
//Preapare to calculate angele
void ydpacket_prepare(YDPacket *packet); //for internal use
//get Angle of pos-th data, total data length is paket.lsn
float ydpacket_angle(YDPacket *packet,uint8_t pos);
//get Length of pos-th data, total data length is paket.lsn
float ydpacket_len(YDPacket *packet,uint8_t pos);

#ifdef __cplusplus
}
///////////////////
//C++ only section
////////////////
class YDLidar{
    public:
    //////////////////////
    //API
    //////////////////////
    void begin(uart_port_t _uartNum,int txPin=UART_PIN_NO_CHANGE,int rxPin=UART_PIN_NO_CHANGE);
    //callBack function on recive a packet
    void setOnRecv(void (*_onRecv)(void *),void *_onRecvData );
    //check 1st packet is arrived
    bool isReady();
    //Number of fail ,on packet parse
    //This will reset to 0 when success to parse
    uint8_t failCount();
    //Copy YDPacket data (thread safe) 
    #if YDLIDAR_USE_PACKET_DATA
    bool getData(YDPacket *out_data);
    #endif
    #if YDLIDAR_USE_SUMMARY_DATA
    bool getSummary(ydlidar_summary_t sum);
    #endif
    portTickType getTickCountFromLast();
    //Very optional: Send YDLidar command (begin() calls startCmd intrnaly)
    void startCmd();
    void stopCmd();
    void resetCmd();
    //Very optional: If you want to stop the task,call stopCmd() and  use this
    xTaskHandle ydlidar_task_handle;

    /////////////
    //End of API
    /////////////
    YDLidar();
    void task();// run the task only from xCreateTask


    private:
    portTickType lastTick;
    void kick();
    void kickAbyte();
    void refresh();
    void lock();
    void unlock();

    void (*onRecv)(void *);
    void *onRecvData;

    portMUX_TYPE muxLock;
    uart_port_t uartNum;
    QueueHandle_t uartQueue;
    YDPacket work,copyBuffer;
    ydlidar_summary_t copyBufferSum;
    uint16_t check;
    const static uint8_t bufLen=128; 
    uint8_t buf[bufLen];
    uint8_t pos=0;//bufferd position
    uint8_t state=0;// parsed position
    uint8_t dataCount;
    uint8_t _failCount;
    uint8_t preByte;
};


#endif

#endif