#line 1 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\Queue.h"
#ifndef _QUEUE_H
#define _QUEUE_H

#include "AP_struct.h"
#include "Wifi.h"
#define QUEUE_TEST_CHL              0
#define Queue_TEST_HDJ              1      

/*Queue Header fnt*/
bool Wifi_enQueue(uint8_t item);
void Wifi_deQueue(void);
bool Wifi_getQueue(uint8_t *item);


//! enq : 꽉찼을 때 알리는 메시지 없음 ,
//^ deQ큐가 비어 있는 경우 아무 작업도 하지 않음. 반환값이 없어 호출자가 성공 여부를 알 수 없음.
//* getQ큐가 비어 있는 경우에 대한 디버깅 메시지가 없음.

#if QUEUE_TEST_CHL == 0



//enQ -> tail +1 ==head ->F , else-> tail에 데이터 넣으면 될ㄹ듯 
bool Wifi_enQueue(uint8_t item)
{
    uint8_t next_tail = (wifi_send.queue.tail + 1) % WIFI_SEND_QUEUE_MAX;
    if(next_tail == wifi_send.queue.head) //head == tail + 1일 때 꼬ㅏㄱ참 
    {
        Serial.printf(" [QUEUE]::FULL QUEUE \r\n");
        return false; 
    }else;
    
    wifi_send.queue.item[wifi_send.queue.tail] = item; 
    wifi_send.queue.tail = next_tail;

    return true;
    
}
#if 1
//deQ -> 안쓸텐데 일단 head+1 == head일때 ? 이건 고민, false 반환까지 짜보기 
void Wifi_deQueue(void)
{
    if(wifi_send.queue.tail != wifi_send.queue.head)
    {
        wifi_send.queue.head = (wifi_send.queue.head + 1) % WIFI_SEND_QUEUE_MAX;
    }else; //* debug? 일단 냅두
}

#else
//^ TEST tail == head, head = tail + 1 ---> return T/F

bool Wifi_deQueue(uint8_t item)
{
    if(wifi_send.queue.tail != wifi_send.queue.head)
    {
        wifi_send.queue.head = (wifi_send.queue.head + 1 ) % WIFI_SEND_QUEUE_MAX;   
    }
    else
    {
        Serial.printf(" [QUEUE]::FAILED...EMPTY QUEUE\r\n");
        return false;
    } 
}
#endif

bool Wifi_getQueue(uint8_t *item)
{
    if(wifi_send.queue.tail == wifi_send.queue.head) 
    {
        //Serial.printf(" [QUEUE]::FAILED...EMPTY QUEUE\r\n");
        return false;
    }else
    {
        *item = wifi_send.queue.item[wifi_send.queue.head];
        wifi_send.queue.head =(wifi_send.queue.head + 1) % WIFI_SEND_QUEUE_MAX;
        return true;
    }
}


#else

bool Wifi_enQueue(uint8_t item)
{
    bool rtn = false;
    if( wifi_send.queue.head>wifi_send.queue.tail )
    {
        if( (wifi_send.queue.head-wifi_send.queue.tail)<(WIFI_SEND_QUEUE_MAX-1) ) rtn = true;
    }else if( wifi_send.queue.head<wifi_send.queue.tail ) 
    {
        if( (wifi_send.queue.tail-wifi_send.queue.head)<(WIFI_SEND_QUEUE_MAX-1) ) rtn = true;
    }else rtn = true;

    if( rtn )
    {
        wifi_send.queue.item[wifi_send.queue.head] = item;

        if( ++wifi_send.queue.head >= WIFI_SEND_QUEUE_MAX ) wifi_send.queue.head=0;
    }

    return rtn;
}
void Wifi_deQueue(void)
{
    if( ++wifi_send.queue.tail >= WIFI_SEND_QUEUE_MAX ) wifi_send.queue.tail=0;
}
bool Wifi_getQueue(uint8_t *item)
{
    if( wifi_send.queue.head != wifi_send.queue.tail )
    {
        *item = wifi_send.queue.item[wifi_send.queue.tail];

        if( ++wifi_send.queue.tail >= WIFI_SEND_QUEUE_MAX ) wifi_send.queue.tail=0;
        return true;
    } //
    else return false;
}
#endif
/**
 * END ... QUEUE ... 
 */

#endif