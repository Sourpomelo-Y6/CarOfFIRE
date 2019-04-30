#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "ydlidarTask.h"
static const char *TAG = "YDLidarTask";

/////helper function
static uint16_t concat(uint8_t hi,uint8_t low){
    return (((uint16_t)hi)<<8 ) + low;
}
static void ydlidar_task(void *obj){
    ((YDLidar*)obj)->task();
}
//methods
void YDLidar::setOnRecv(void (*_onRecv)(void *),void *_onRecvData ){
    onRecv=_onRecv;
    onRecvData=onRecvData;
}
bool YDLidar::isReady(){ 
    return _failCount!=255;
}
uint8_t YDLidar::failCount(){  
    return _failCount;
}

void YDLidar::startCmd(){
    const static char startCmd[]={0xA5,0x60};
    uart_write_bytes(uartNum,startCmd,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(uartNum,startCmd,2);
}
void YDLidar::resetCmd(){
    const static char resetCmd[]={0xA5,0x40};
    uart_write_bytes(uartNum,resetCmd,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(uartNum,resetCmd,2);
    refresh();
    _failCount=255;
}
void YDLidar::stopCmd(){
    const static char stopCmd[]={0xA5,0x65};
    uart_write_bytes(uartNum,stopCmd,2);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uart_write_bytes(uartNum,stopCmd,2);
    refresh();
    _failCount=255;
}
void YDLidar::refresh(){
    pos=0;
    state=0;
    check=0x55AA;
    dataCount=0;
    if(_failCount<254)_failCount++;
}
void YDLidar::kickAbyte(){
    char now=buf[state];
    
    if(state<10)switch (state){
        case 0://header
            state++;
            if(now!=0xAA){
                refresh();
            }
            break;
        case 1://header
            state++;
            if(now!=0x55){
                refresh();
            }
            break;
        case 2://packet type(CT) 0=normal 1=zeropacket
            state++;
            preByte=now;
            if(0x01<now ){
                ESP_LOGE(TAG, "CT");
                refresh();
            }
            break;
        case 3://LSN: data len
            state++;
            check^=concat(now,preByte);
            work.lsn=now;
            if(preByte==1 && work.lsn != 1){
                ESP_LOGE(TAG, "ZP");
                refresh();//zero packet mustbe have 1 data.
            }
            if(MaxLSN<work.lsn){
                ESP_LOGE(TAG,"Too big packet %d",work.lsn);
                refresh();
            }
            break;
        case 4://First byte of FSA and LSA
        case 6:
            state++;
            preByte=now;
            if( (now&1) ==0){
                ESP_LOGE(TAG, "Ang");
                refresh();
            }
            break;
        case 5:// FSA: start Angle
            state++;
            check^=work.fsa=concat(now,preByte);
            work.fsa>>=1;
            break;
        case 7:// LSA: start Angle
            state++;
            check^=work.lsa=concat(now,preByte);
            work.lsa>>=1;
            break;
        case 8://fist byte of CS
            state++;
            preByte=now;
            break;
        case 9:
            state++;
            check ^=concat(now,preByte);
            break;
        default://none 
        break;    
    }
    else{//10<=state 
        state++;
        if((state&1)==1){//preByte
            preByte=now;
        }else{
            //get a data
            check ^= work.s[dataCount]=concat(now,preByte);
            dataCount++;
            if(dataCount==work.lsn){
                if(check !=0){
                    ESP_LOGE(TAG,"checksum error %x LSN %d",check,work.lsn);
                    refresh();
                }else{
                    //////////////packet complite!!!
                    refresh();
                    lock();
                    _failCount=0;///Copy done and set good flag!(be sure to lock free)
                    ydpacket_prepare(&work);
                    #if YDLIDAR_USE_SUMMARY_DATA
                    for(int i=0;i<work.lsn;i++){
                        int pos=(int)(YDLIDAR_SUMMARY_LEN*ydpacket_angle(&work,i)/360);
                        if(pos<0)pos=0;
                        if(YDLIDAR_SUMMARY_LEN<=pos)pos=YDLIDAR_SUMMARY_LEN-1;
                        copyBufferSum[pos]=(int16_t)(ydpacket_len(&work,i));
                    }
                    #endif
                    #if YDLIDAR_USE_PACKET_DATA
                    copyBuffer=work;
                    #endif
                    lastTick=xTaskGetTickCount();
                    unlock();
                    if(onRecv)onRecv(onRecvData);
                }
            }
        }
    }
}
void YDLidar::kick(){
    while(state<pos){
        kickAbyte();
    }

}

void YDLidar::task(){
    uart_event_t event;
    ESP_LOGI(TAG, "Start task");
    for(;;) {
        if(xQueueReceive(uartQueue, (void * )&event, (portTickType)portMAX_DELAY)) {
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    
                    if( bufLen<event.size+pos ){//buffer over run
                        refresh();
                        uart_flush_input(uartNum);
                        ESP_LOGE(TAG, "Buffer overun");
                    }else{
                        int ret=uart_read_bytes(uartNum, buf+pos, event.size, portMAX_DELAY);
                        if(ret<0){
                            refresh();
                            uart_flush_input(uartNum);
                            ESP_LOGE(TAG, "readError");
                        }else{
                            pos+=ret;
                            kick();
                            }
                    }
                    break;
                case UART_FIFO_OVF:
                    ESP_LOGE(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(uartNum);
                    xQueueReset(uartQueue);
                    break;
                default:
                    ESP_LOGE(TAG, "uartError event: %d", event.type);
                    break;
            }
        }
    }
}

void YDLidar::begin(uart_port_t _uartNum,int txPin,int rxPin){
    uartNum=_uartNum;
    uart_config_t uart_config; 
    uart_config.baud_rate = 128000;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh=0;
    if(ESP_OK != uart_param_config(uartNum, &uart_config)){
        ESP_LOGE(TAG,"uart param error");
    }
    /*Default 
    UART1 TX - GPIO 10
    UART1 RX - GPIO 9
    UART2 TX - GPIO 17
    UART2 RX - GPIO 16
    */

    if(ESP_OK !=uart_set_pin(uartNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)){
        ESP_LOGE(TAG,"uart set pin error");
    }
    if(ESP_OK != uart_driver_install(uartNum, 256 /*RX buffer size*/, 0/*TX buffer size*/, 20, &uartQueue, 0)){
        ESP_LOGE(TAG,"uart install error");
    }
    refresh();
    #if YDLIDAR_USE_SUMMARY_DATA
    for(int i=0;i<YDLIDAR_SUMMARY_LEN;i++)copyBufferSum[i]=0;
    #endif
    lastTick=0;
    _failCount=255;
    startCmd();
    xTaskCreatePinnedToCore(ydlidar_task, "YDLidarTask", 2048,this,YLIDAR_TASK_PRIO, &ydlidar_task_handle,YLIDAR_TASK_CORE);
}
#if YDLIDAR_USE_PACKET_DATA
bool YDLidar::getData(YDPacket *out_data){
    lock();
    *out_data = copyBuffer;
    unlock();
    if(_failCount==255)return false;
    //TODO: return false when failCount is too high??
    return true;
}
#endif
#if YDLIDAR_USE_SUMMARY_DATA
bool YDLidar::getSummary(ydlidar_summary_t out){
    lock();
    for(int i=0;i<YDLIDAR_SUMMARY_LEN;i++)out[i]=copyBufferSum[i];
    unlock();
    if(_failCount==255)return false;
    //TODO: return false when failCount is too high??
    return true;
}
#endif
portTickType YDLidar::getTickCountFromLast(){
    if(_failCount==255)return 0x7fffffff;
    portTickType ret;
    lock();
    ret=xTaskGetTickCount()- lastTick;
    unlock();
    return ret;
}
YDLidar::YDLidar(){
    muxLock=portMUX_INITIALIZER_UNLOCKED;
    _failCount=255;//at start thre is no valid data
    onRecv=NULL;
}
void YDLidar::lock(){
    taskENTER_CRITICAL(&muxLock);
}
void YDLidar::unlock(){
    taskEXIT_CRITICAL(&muxLock);
}


extern "C"{
    
    static YDLidar *the_ydlidar;// just for single ydlidar 
    void ydlidar_begin(uart_port_t uartNum,int txPin,int rxPin,void (*_onRecv)(void *),void *_onRecvData)
    {
        the_ydlidar=new YDLidar();
        the_ydlidar->begin(uartNum,txPin,rxPin);
        the_ydlidar->setOnRecv(_onRecv,_onRecvData);
    }
    #if YDLIDAR_USE_PACKET_DATA
    bool ydlidar_get(YDPacket *outData)
    {
        return the_ydlidar->getData(outData);
    }
    #endif
    #if YDLIDAR_USE_SUMMARY_DATA
    bool ydlidar_get_summary(ydlidar_summary_t out)
    {
        return the_ydlidar->getSummary(out);
    }
    #endif
    portTickType ydlidar_get_tickcount_from_last()
    {
         return the_ydlidar->getTickCountFromLast();

    }
    
    bool ydlidar_is_ready()
    {
        return the_ydlidar->isReady();
    }
    uint8_t ydlidar_fail_count()
    {
        return the_ydlidar->failCount();
    }
    
    void ydpacket_prepare(YDPacket *packet){
        packet->startAng=(float)(packet->fsa)/64;
        packet->endAng=(float)(packet->lsa)/64;
        packet->stepAng=(float)(packet->endAng-packet->startAng);
        if(packet->stepAng<0)packet->stepAng+=360;
        packet->stepAng/=(packet->lsn-1);
    }
    float ydpacket_angle(YDPacket *packet,uint8_t pos){
        float ret=packet->startAng + packet->stepAng*pos;
        if(0==packet->s[pos])return ret;
        //Approximate of : 180/pi*atan(21*(155.3-Distance)/(155.3*Distance) )
        ret+=1/( 0.0008162604522317974f/4.0f * packet->s[pos]  -0.00220016337230633f )- 7.990596267964586f;
        if(ret<0)ret+=360;
        if(360<=ret)ret-=360;
        if(ret<0 || ret>360)ESP_LOGE(TAG,"arg calc error");
        return ret;
    }
    float ydpacket_len(YDPacket *packet,uint8_t pos){
        return (float)(packet->s[pos])/4.0f;
    }


}