#include <Arduino.h>
#line 1 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
    
/**
//? 2025-05-13 : CHL 
    - Wifi_handler LOG 수정, 
    - UART 115200 -> 230400 
    - Scan 방식 변경 : Fast scan. 
    
//? 2025-01-17 : JHD
 * - 전체적인 구조, 처리 방식 모두를 변경했기 때문에 이전 개발 로그 필요 없어서 지웠음
 * - AP가 마스터, Peer가 Slave가 되는 통신 방식으로 변경
 * 
 * 
 */
#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"
#include "AP_struct.h"
#include "Queue.h"
#include "Wifi.h"
#include "Tick_Handler.h"
#define CUSTOM
#include "AP_SLAVE.h"
#include "EasyCAT.h"
#include <SPI.h>
EasyCAT EASYCAT(SS);

///////////////////////// TIMER INTERRUPT /////////////////////////////////////
// These define's must be placed at the beginning before #include "TimerInterrupt_Generic.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
#define _TIMERINTERRUPT_LOGLEVEL_     4
#define TIMER0_INTERVAL_MS            1
// To be included only in main(), .ino with setup() to avoid `Multiple Definitions` Linker Error
#include "ESP32TimerInterrupt.h"
#include "common.h"

#define FUNC_REPAIR_SAVE_DATA       0
#define FUNC_PAIRED_VERIFY          1

#define DEBUG

// 무선 Network config
#define MAX_SERIAL_PEER             4           // serial을 사용하는 peer 최대수
#define MAX_CHANNEL                 12          // 2.4GHz channel 한국/중국 1~13

/*********************************************** */
#define AP                          "AP"         //   CHL:temp...
#define VERSION                     "v3.2"       //   CHL:temp...
#define Last_Changed_Date           "25/05/13"   //   CHL:temp...
#define TEST_POINT                  0            //   CHL:temp...
/*********************************************** */

// 부품 Type and id define
#define DIW_FLOW                    0x01
#define CDA_FLOW                    0x02
#define SMART_DAMPER                0x03 
#define X_RAY                       0x04
#define D40A                        0x05
#define D4SL                        0x06
#define TIC                         0x07 
#define LMFC                        0x08
#define MANOMETER                   0x09
#define LCT                         0x0A

// Reqeuset & response HEX
//AP
#define AP_PAIRING_REQ              0x01      
#define AP_PAIRING_CANCEL           0x02
#define AP_IO_GET                   0x03      
#define AP_IO_SET                   0x04
#define AP_SERIAL_SET               0x05
#define AP_SERIAL_GET               0x06

//PEER
#define PEER_PAIRING_OK             0x81
#define PEER_PAIRING_CANCEL         0x82
#define PEER_IO_GET                 0x83
#define PEER_IO_SET                 0x84
#define PEER_SERIAL_SET             0x85      
#define PEER_SERIAL_GET             0x86

#define MAX_PEER                    16      // peer 최대수
#define MAX_IO_BUFF                 4       // in 16bit word
#define MAX_SERIAL_BUFF             70      //
#define MAX_SERIAL_PEER             4       //
#define MAX_PAIRING_BUFF_SIZE       16      //
#define MAX_SEND_BUFF               40      // in word
#define MAX_IO_PEER                 12      //

/********************************************** */
// LAN9252                                      //
#define ALIAS_REG                   0x0012      //
#define ALIAS_REG_H                 0x0012      //
#define ALIAS_REG_L                 0x0013      //
/********************************************** */

/***************************************************** */
// Hardware IO                                         //
#define SW                          0                  // Onboard SW
#define LED                         2                  // Onboard LED Blue
                                                       //
                                                       //
#define MUXInput1                   36                 //
                                                       //
#define NRESET                      4                  // 
#define MUX_16EN1                   32                 //
#define MUX_SEL0                    27                 //
#define MUX_SEL1                    26                 //
#define MUX_SEL2                    33                 //
#define MUX_SEL3                    25                 //
#define DEBUG__CHANG                14
                                                       //
/***************************************************** */
#define SET_TIME_PAIRING    59
#define SET_TIME_PAIRD      29

/***************************************************** */ 
portMUX_TYPE pMUX = portMUX_INITIALIZER_UNLOCKED;      //
SemaphoreHandle_t send_sem;                            //
SemaphoreHandle_t recv_sem;                            //
SemaphoreHandle_t tx_sem;                              //
SemaphoreHandle_t rx_sem;                              //
/***************************************************** */
uint8_t debug_out=0; 
uint8_t iAddr1, iAddr2;                              // Rotary

uint16_t lPeerData[MAX_IO_PEER+1][5];                // AP rev26; IO영역 최대 4페이지; 0페이지 없음 -> 조건에따라 0페이지 있음
uint16_t lSPeerData[MAX_SERIAL_PEER+1][81];          // AP rev29; 240409 시리열영역 최대 데이터 70워드;
uint16_t iAddr;                                      // Rotary
uint16_t *null_ptr = 0;

ESP32Timer ITimer0(0);

/***************************************************** */
                                                       //
/*main*/                                               //
void set_board_Version(void);                          // 
void rssi_display(void);                               //
void Set_GPIO(void);                                   //
void Mux_Sel_16ch(uint8_t Ch);                         //
void Read_Rotary(void);                                //
void Ethercat_Handle(void);                            //
bool wl_init(void);                                    //
void Initialize_PDO(void);                             //
void Wifi_Handle(void);                                //
bool IRAM_ATTR TimerHandler0(void *timerNo);           //
                                                       //
/***************************************************** */

#if 1
#line 659 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void recv_cb(const uint8_t *src_mac, const uint8_t *data, int len);
#line 1002 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void send_cb(const uint8_t *des_addr, esp_now_send_status_t status);
#line 1082 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void debug();
#line 1090 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void serial_cmd();
#line 1879 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void setup();
#line 1939 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void loop();
#line 4 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\common.ino"
void output_toggle( uint8_t PIN );
#line 11 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\common.ino"
uint16_t crc16_modbus(uint16_t init_crc, uint8_t* dat, uint16_t len);
#line 150 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\APv3.2_cust.ino"
void set_board_Version(void)
{
    Serial.printf("\n");
    Serial.printf("[MSG]BOARD INFO::%s::VERSION:%s\r\n",AP,VERSION);//CHL:temp...
    Serial.printf("[MSG]LAST CHANGED DATE...[%s]\n",Last_Changed_Date);
    Serial.printf("\n");
}
#endif 

#if 1
void rssi_display(void)
{
    // AP부팅시 RSSI 초기값 3으로 해둠

    uint16_t tx_p_status = EASYCAT.BufferOut.Cust.pairing_bit;  // 녹색

    for (uint8_t i = 0; i < MAX_PEER; i++)
    {
        if (peer[i].pairFlag == true)
        {
            if ( bitRead(tx_p_status, i) == 0 )     // 페어링되어있고, 요청비트가꺼있을때
            {
                Serial.printf("Disp..RSSI[%02d] = %02d, ", i, RSSI[i]);
            }
            else    // 페어링은 되었으나 요청비트가 아직 안꺼졌을때
            {
                Serial.printf("Disp..RSSI[%02d] = %02d(PairingBit OFF)", i, RSSI[i]);
            }
        }
    }
    Serial.printf("\n");
}
#endif

void Set_GPIO(void)
{
    // Input Setting
    pinMode(SW, INPUT);                 // ESP32 Onabrd SW
    pinMode(MUXInput1, INPUT); 
    
    // Output Setting
    pinMode(LED, OUTPUT);               // ESP32 Onboard Blue LED
    pinMode(NRESET, OUTPUT);           
    pinMode(MUX_16EN1, OUTPUT);           
    pinMode(MUX_SEL0, OUTPUT);           
    pinMode(MUX_SEL1, OUTPUT);           
    pinMode(MUX_SEL2, OUTPUT);           
    pinMode(MUX_SEL3, OUTPUT);           
    pinMode(DEBUG__CHANG, OUTPUT);
    Serial.printf("[MSG]SYSTEM SETUP::MODULE::Set GPIO OK\n");
}

void Mux_Sel_16ch(uint8_t Ch)
{
    switch (Ch)
    {   // S3, S2, S1, S0
        case 0:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 1:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 2:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 3:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 4:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 5:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 6:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 7:        digitalWrite(MUX_SEL3, LOW);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 8:        digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 9:        digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 10:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 11:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, LOW);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 12:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, LOW);        break;
        case 13:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, LOW);        digitalWrite(MUX_SEL0, HIGH);        break;
        case 14:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, LOW);        break;
        case 15:       digitalWrite(MUX_SEL3, HIGH);        digitalWrite(MUX_SEL2, HIGH);        digitalWrite(MUX_SEL1, HIGH);        digitalWrite(MUX_SEL0, HIGH);        break;
        default:
        break;
    }
    delay(10);
}

void Read_Rotary(void)
{
    uint8_t i = 0;
    iAddr1 = 0;
    iAddr2 = 0;

    uint8_t readR3=0, readR4=0;

    digitalWrite(MUX_16EN1, HIGH);
    delay(100);

    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr2 += 4;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr2 += 1;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr2 += 8;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr2 += 2;
    i++;

    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr1 += 4;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr1 += 1;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr1 += 8;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        iAddr1 += 2;

    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR4 += 4;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR4 += 1;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR4 += 8;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR4 += 2;
    i++;

    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR3 += 4;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR3 += 1;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR3 += 8;
    i++;
    Mux_Sel_16ch(i);
    if (!digitalRead(MUXInput1))
        readR3 += 2;     


    digitalWrite(MUX_16EN1, LOW);
    if( (readR3>14) || (readR3==0) ) readR3 = 1;
    if( readR4 > 15 ) readR4 = 1;

    wifi_send.rf_set_channel = readR3;
    wifi_send.rf_set_group = readR4;

    iAddr = (iAddr2 * 16) + iAddr1;

    Serial.printf( "[MSG]SYSTEM::INIT::Address =%d, WIFI Group= %d,WIFI Channel=%d\r\n",iAddr, wifi_send.rf_set_group, wifi_send.rf_set_channel );
}

#if 0
void AP_Serial_SCAN()
{
    // PLC에서 쓰고자 하는데이터를 피어에 전달하기 위함, 피어로부터 데이터를 수신하기 위한 목적은 없음

    /*
        시리얼 영역의 데이터만 처리할것(Ch, Page, R/W; DataCOM2 ~ DataCOM8)
        -> 시리얼 영역의 IO는 AP_IO_SCAN(); 에서 처리함   --- Done

    */
    uint16_t tempData[8];

    uint16_t tx_pStatus = EASYCAT.BufferOut.Cust.pairing_bit;           // 녹색, 페어링비트 읽기
    uint16_t tx_serial = EASYCAT.BufferOut.Cust.com1;                   // 녹색, Serial 영역 Data1, Ch, Page 영역
    uint16_t *serial_ptr = (uint16_t *)&EASYCAT.BufferOut.Cust.com2;    // 녹색, Serial 영역 Data2 ~, 시리얼 데이터 영역

    uint8_t cnt = 0;

    uint8_t tx_w_ch = (tx_serial & 0xF000) >> 12;
    uint8_t tx_w_page = (tx_serial & 0x0F00) >> 8;
    uint8_t tx_r_ch = (tx_serial & 0x00F0) >> 4;
    uint8_t tx_r_page = tx_serial & 0x000F;
    // 채널은 pdo맵상 디바이스번호, 페이지는 해당 디바이스의 채널

    tempData[0] = EASYCAT.BufferOut.Cust.com1;  // ReadCh, Page, WriteCh, Page 싹다 담음
    tempData[1] = EASYCAT.BufferOut.Cust.com2;
    tempData[2] = EASYCAT.BufferOut.Cust.com3;
    tempData[3] = EASYCAT.BufferOut.Cust.com4;
    tempData[4] = EASYCAT.BufferOut.Cust.com5;
    tempData[5] = EASYCAT.BufferOut.Cust.com6;
    tempData[6] = EASYCAT.BufferOut.Cust.com7;
    tempData[7] = EASYCAT.BufferOut.Cust.com8;

    
    static unsigned long preTime = 0;
    static unsigned long rxTime[MAX_SERIAL_PEER] = { 0, };
    uint16_t rx_write_update = 0;

    uint8_t idx = serial_idx(tx_w_ch);  // Max_peer(16) - Max_Serial_Peer(4) + Serial_ch - 1


    uint8_t i;
    if( tx_w_ch == tx_r_ch )        i = tx_w_ch +11;
    else if( tx_w_ch && !tx_r_ch )  i = tx_w_ch +11;
    else if( !tx_w_ch && tx_r_ch )  i = tx_r_ch +11;
    
    if( i > 11 )    // 240112 : 비트 4개다 체크하면 에초에 안넘어고 있음, 수정할것 --- Done
    {
        if( bitRead(tx_pStatus, i) == 0 )   // 녹색, 페어링비트영역의 비트 상태를 읽어서 처리; 요청이 있는지 없는지봄
        {
            if( peer[i].pairFlag == 1 && peer[i].serial > 0 )   // 페어링 플래그가 서있고, 시리얼영역을 사용한다 했을때
            {
                query_serial(i, AP_SERIAL_TX_REQ, tempData, 16); // 데이터 전체 다 보내고 피어에서 판단; 패널,페이지비트만 전송
            }
        }
    }
}
#endif

#if TEST_POINT //! CHL
void Ethercat_Handle(void)
{
    static uint16_t tx_pairStatus;
    static uint16_t rx_pairStatus;

    static uint16_t *tx_ptr;
    static uint16_t *rx_ptr;
    static uint8_t peer_channel=0;

    bool req_pair=false;

    uint8_t device_id, device_type, page;

    tx_pairStatus = EASYCAT.BufferOut.Cust.pairing_bit;

    tx_ptr = (uint16_t *)&EASYCAT.BufferOut.Cust.data1;    
    rx_ptr = (uint16_t *)&EASYCAT.BufferIn.Cust.data1;

    EASYCAT.BufferIn.Cust.pairing_bit = rx_pairing_status;  // 적색; rx_pairing status update

    if(TICK.flag.bit_flag.sec1)
    {
        Serial.printf("chk point\r\n");
    }
    if( bitRead(tx_pairStatus, peer_channel) ) req_pair = true;

    if(req_pair)
    {
        device_type = (uint8_t)(tx_ptr[peer_channel]>>8);  
        device_id = (uint8_t)(tx_ptr[peer_channel]&0x00ff);       
        if(!peer[peer_channel].pairFlag)//pairing 꺼져있으면 
        {
            if(wifi_state_machine[peer_channel].pairing.request == false)//요청아닐때 송신할 것
            {
                peer[peer_channel].typeAddr = tx_ptr[peer_channel];
                Wifi_Peer_State_Set(WIFI_STATE_PAIRING_ADD,peer_channel,AP_PAIRING_REQ,peer[peer_channel].typeAddr,0);
                Serial.printf(" [ECAT]::peer channel[%d] .. NEW ADD::TypeAddr=0x%04x\r\n",peer_channel,peer[peer_channel].typeAddr);
            }
        }else if(peer[peer_channel].pairFlag && device_type && device_id)
        {
            #if TEST_POINT
            if(peer_channel<12)
            {Serial.printf(" [ECAT] :: CHK \r\n");}
            else;
            #endif

        }else if(peer[peer_channel].pairFlag && !tx_ptr[peer_channel])
        {
            //del
            if(wifi_state_machine[peer_channel].pairing.request == false)
            {
                Wifi_Peer_State_Set(WIFI_STATE_PAIRING_DEL,peer_channel,AP_PAIRING_CANCEL,0,0);
                Serial.printf(" [ECAT]::peer channel[%d] DEL\r\n",peer_channel);

            }else;
        }else;
    }else
    {
        /***********
        *
        *       Serial 부분 뺐음 
        *
        *
        *
        * *********/
    }
    if( ++peer_channel>=MAX_PEER ) peer_channel=0;    
    

}
#endif


void Ethercat_Handle(void)
{
    
    // ethercAT 값 Scan 또는 pairing 해야할 목록 업데이트
    // rev25 : IO영역(시리얼을 포함하는)에 대해 PDO맵상 R/W 비트 확인하여 피어에 설정데이터 송신
    static uint8_t peer_channel=0;

    static uint16_t tx_pairStatus;
    static uint16_t rx_pairStatus;
    static uint16_t *tx_ptr; //녹색 data
    static uint16_t *rx_ptr; 

    static uint16_t tempData[17];
    static uint16_t sendData[2];

    static uint16_t tx_serial;      // 녹색, Serial 영역 Data1, Ch, Page 영역   
    static uint16_t *serial_ptr;    // 녹색, Serial 영역 Data2 ~, 시리얼 데이터 영역                    

    static uint8_t tx_w_ch; //W17, 각4bit단위, [write Ch][write Page][Read Ch][Read Page]
    static uint8_t tx_w_page;
    static uint8_t tx_r_ch;
    static uint8_t tx_r_page;  

    uint8_t tmpCh=0;

    bool req_pair=false;
    bool w_serial=false;

    uint8_t device_id, device_type, page;

    tx_pairStatus = EASYCAT.BufferOut.Cust.pairing_bit; //WMX에서 pairing bit 눌렀을 때(녹) 1

    tx_ptr = (uint16_t *)&EASYCAT.BufferOut.Cust.data1;           // 녹색 //0x0617, type+id
    rx_ptr = (uint16_t *)&EASYCAT.BufferIn.Cust.data1;            // 적색
    //rx
    
    EASYCAT.BufferIn.Cust.pairing_bit = rx_pairing_status;  // 적색; rx_pairing status update    
    //Serial.printf("Eterhcan_Handle::Pear[%d]::Statue=0x%02x, TypeId=0x%02x\r\n", addr,tx_pairStatus,tx_ptr[addr]);


    if( TICK.flag.bit_flag.sec1 )
    {
        //LOG_DEL_CHL.. 
        Serial.printf("[MSG]ECAT::Request Pair List = 0x%04x\r\n",tx_pairStatus );
    }

    if( bitRead(tx_pairStatus, peer_channel) ) req_pair = true;

    if( req_pair )
    {
        device_type = (uint8_t)(tx_ptr[peer_channel]>>8);               //d4sl기준, type 0x06, id => alias addr 0x17
        device_id = (uint8_t)(tx_ptr[peer_channel]&0x00ff);             //

        //1. 페어링이 안되어 있고, device id와 device type가 있을 경우(id, type은 ap slave map상의 용어)
        if( !peer[peer_channel].pairFlag && device_type && device_id )  //참고: pairFlag는 위 register()에서 리스트에 peer 등록하고 set시킴 
        {
            
            if( wifi_state_machine[peer_channel].pairing.request == false ) //요청중이지 않을때만 전송
            {
                peer[peer_channel].typeAddr = tx_ptr[peer_channel]; 

                Wifi_Peer_State_Set(WIFI_STATE_PAIRING_ADD, peer_channel, AP_PAIRING_REQ, peer[peer_channel].typeAddr, 0);

                Serial.printf("[MSG]ECAT::Request Pair[%d]::New Add::TypeAddress=0x%04x\r\n",peer_channel,peer[peer_channel].typeAddr);
            }
        //2. 페어링 되어있고, device id와 device type가 있을 경우 
        //여기 추가 됐음 
        }else if( peer[peer_channel].pairFlag && device_type && device_id ) //추가 및 변경
        {
            //페어링 성공 했고, 데이타는 아직 안 받았을때 : 데이타 요청 처리를 해야 함. 
            if( peer_channel<12 )
            {
                
                #if 0 
                if( peer[peer_channel].io_page )  //read and page
                {
                    page = 1;
                    lPeerData[peer_channel][0] = 0; //0은 page가 없을때의 데이타 영역. 쓰레기 값이 들어갈 수 있음
                }else   //read and no page
                {
                    page = 0;

                    wifi_state_machine[peer_channel].peer.set_io_data = 0;
                }
                #endif
            }else if( (peer_channel<16) && peer[peer_channel].serial ) //D4SL 경우 serial 0 들어옴     
            {
                if( !wifi_state_machine[tmpCh].peer.update_serial && !wifi_state_machine[tmpCh].peer.request_serial ) //Serial Device 12-15
                {
                    wifi_state_machine[tmpCh].peer.request_serial = true;
                    
                    memset(&wifi_state_machine[tmpCh].peer.set_serial_Buf[1], 0, 16);
                    // device comm command
                    wifi_state_machine[tmpCh].peer.set_serial_Buf[1] = 0;
                }
            }else;
        //3.peer는 등록되어 있고, ethercat에는 없을 경우 
        }else if( peer[peer_channel].pairFlag && !tx_ptr[peer_channel] )    
        {
            if( wifi_state_machine[peer_channel].pairing.request == false ) //요청중이지 않을때만 전송
            {
                Wifi_Peer_State_Set(WIFI_STATE_PAIRING_DEL, peer_channel, AP_PAIRING_CANCEL, 0, 0);
                Serial.printf("[MSG]ECAT::Request Pair[%d]::Del\r\n",peer_channel);
            }
        }else
        {
            //예외 무시
        }
    }else //req_pair가 false일 때
    {
        if( peer[peer_channel].pairFlag )   //paired 일때
        {
            //20250105 : Serial Deviceeh io를 사용할 수 있기 때문에 15까지 검색해서 해야 하는데, 시스템에서 아직 듀얼로는 사용 안하니까
            //추후에 시스템과 협의해서 작업 필요. 코드는 만들어서 넣어놨음..검증필요!!!!
            if( peer_channel<12 )
            { //i/o device data 처리 //PDO 
                if( (tx_ptr[peer_channel]& BIT15) && (peer[peer_channel].io_page) ) //write and page
                {
                    page = (tx_ptr[peer_channel] & 0x7000) >> 12;
                    //wifi_state_machine[peer_channel].peer.rw = true;    //peer.rw 현재 사용하고 있지 않은데 필요 할수 있어서 Flag 처리리
                    //Write
                    if( !wifi_state_machine[peer_channel].peer.update_io )
                    {
                        wifi_state_machine[peer_channel].peer.update_io = true;
                        wifi_state_machine[peer_channel].peer.set_io_data = tx_ptr[peer_channel];
                    }
                    lPeerData[peer_channel][0] = 0; //0은 page가 없을때의 데이타 영역. 쓰레기 값이 들어갈 수 있음

                }else if( peer[peer_channel].io_page )  //read and page
                {
                    page = (tx_ptr[peer_channel] & 0x7000) >> 12;
                    //wifi_state_machine[peer_channel].peer.rw = false;

                    lPeerData[peer_channel][0] = 0; //0은 page가 없을때의 데이타 영역. 쓰레기 값이 들어갈 수 있음

                }else   //read and no page
                {
                    page = 0;
                    
                    wifi_state_machine[peer_channel].peer.set_io_data = tx_ptr[peer_channel]; //page가 없을때도 쓰기 데이타를 계속 보내야 함
                    //wifi_state_machine[peer_channel].peer.rw = false;
                }
                memcpy(&rx_ptr[peer_channel], (const uint16_t *)&lPeerData[peer_channel][page], 2);

            }else if( (peer_channel<16) && peer[peer_channel].serial )
            {
                //IO와 SERIAL은 같은 채널 동기화 시켜서 처리 : 그래야지 무선으로 데이타를 한번에 보낼 수 있음

                tx_serial = EASYCAT.BufferOut.Cust.com1; //17word놈 
                tx_w_ch = (tx_serial & 0xF000) >> 12;
                tx_w_page = (tx_serial & 0x0F00) >> 8;
                tx_r_ch = (tx_serial & 0x00F0) >> 4;
                tx_r_page = tx_serial & 0x000F;          

                if( tx_w_ch && tx_w_page )
                {
                    tmpCh = 11+tx_w_ch;
                    if( !wifi_state_machine[tmpCh].peer.update_serial ) //Serial Device 12-15
                    {
                        //write
                        //채널 요청 정보가 있을때
                        wifi_state_machine[tmpCh].peer.update_serial = true;
                        wifi_state_machine[tmpCh].peer.request_serial = false;
                        memcpy(&wifi_state_machine[tmpCh].peer.set_serial_Buf[1], &EASYCAT.BufferOut.Cust.com1, 16);
                    }
                }

                if( tx_r_ch && tx_r_page )
                {
                    tmpCh = 11+tx_r_ch;
                    Serial.printf("*********************ECAT[%d]::READ::tx_w_ch=%d, tx_w_page=%d,",tmpCh, tx_r_ch,tx_r_page );
                    //read
                    serial_ptr = (uint16_t *)&EASYCAT.BufferIn.Cust.com1;         // 적색
                
                    memcpy(&serial_ptr[0], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+0], 2);
                    memcpy(&serial_ptr[1], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+1], 2);
                    memcpy(&serial_ptr[2], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+2], 2);
                    memcpy(&serial_ptr[3], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+3], 2);
                    memcpy(&serial_ptr[4], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+4], 2);
                    memcpy(&serial_ptr[5], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+5], 2);
                    memcpy(&serial_ptr[6], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+6], 2);
                    memcpy(&serial_ptr[7], (const uint16_t *)&lSPeerData[tx_r_ch-1][((tx_r_page-1)*8)+7], 2);

                    if( !wifi_state_machine[tmpCh].peer.update_serial && !wifi_state_machine[tmpCh].peer.request_serial ) //Serial Device 12-15
                    {
                        wifi_state_machine[tmpCh].peer.request_serial = true;
                        
                        memset(&wifi_state_machine[tmpCh].peer.set_serial_Buf[1], 0, 16);
                        // device comm command
                        wifi_state_machine[tmpCh].peer.set_serial_Buf[1] = tx_serial;
                    }
                }
                
                if( tmpCh )
                {
                    wifi_state_machine[tmpCh].peer.set_serial_Buf[0] = tx_ptr[tmpCh];
                    memcpy(&rx_ptr[tmpCh], (const uint16_t *)&lPeerData[tmpCh][0], 2);
                }
            }
        }
    }//ap pairing req/ap pairing cancel은 send data 존재,  ap io get/set, ap serial set/get 

    if( ++peer_channel>=MAX_PEER ) peer_channel=0;    
}



void recv_cb(const uint8_t *src_mac, const uint8_t *data, int len)//380,promiscuous_cb
{
    uint8_t peer_channel = 0, packet_len=0, cnt, data_len;
    word_big_endian_t crc16;
    uint16_t readData;

    static uint8_t buff[WIFI_PACKET_MAX];
    uint8_t page=0;
    uint8_t exist_cnt=0,i;

    memcpy(buff, data, len);
    peer_channel = buff[WIFI_PACKET_CHANNEL];
    packet_len = (uint8_t)len;
    //rssi_display(); //:CHL

    if( wifi_send.rf_set_group != buff[WIFI_PACKET_GROUP] )
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Miss match group::AP=%d, Peer=%d\r\n", wifi_send.rf_set_group, buff[WIFI_PACKET_GROUP]);
        return;
    }

    if( packet_len != buff[WIFI_PACKET_LENGTH] )
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Err packet Length::AP=%d, Peer=%d\r\n", packet_len, buff[WIFI_PACKET_LENGTH]);
        return;
    }

    if( !wifi_send.rx_busy[peer_channel] )
    {
        //해당 채널에 대해서 wifi handle이 처리 중
        //if( peer_channel == wifi_state_machine[peer_channel].peer.channel )
        Serial.printf("[MSG]WIFI::CALLBAK::Channel[%d] is not rxBusy\r\n",peer_channel );
        return;
    } 
    wifi_send.doing_recv_cb = true;
    
    if( peer_channel > 15 ) 
    {
        wifi_send.doing_recv_cb = false;
        return;
    }

    if( peer_channel != wifi_state_machine[peer_channel].peer.channel )
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Miss match channel::Response=%d vs Request=%d\r\n",peer_channel, wifi_state_machine[peer_channel].peer.channel);        
        wifi_send.doing_recv_cb = false;
        return;
    }
    if( buff[WIFI_PACKET_TYPE] != wifi_state_machine[peer_channel].peer.device_type ) 
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Miss match type::Response=%d vs Request=%d\r\n",buff[WIFI_PACKET_TYPE], wifi_state_machine[peer_channel].peer.device_type);        
        wifi_send.doing_recv_cb = false;
        return;
    }
    if( buff[WIFI_PACKET_ADDRESS] != wifi_state_machine[peer_channel].peer.device_addr ) 
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Miss match address::Response=%d vs Request=%d\r\n",buff[WIFI_PACKET_ADDRESS], wifi_state_machine[peer_channel].peer.device_addr);        
        wifi_send.doing_recv_cb = false;
        return;
    }

    //CRC 체크
    crc16.flag.wd = crc16_modbus(CRC16_MODBUS_INIT_CODE, buff, packet_len-2);
    if( (crc16.flag.bf.low != buff[packet_len-2]) || (crc16.flag.bf.hi != buff[packet_len-1]) )
    {
        Serial.printf("[MSG]WIFI::CALLBAK::Miss match crc16::low=0x%02x hi=0x%02x vs packet[low]=0x%02x, packet[hi]=0x%02x\r\n",crc16.flag.bf.low, crc16.flag.bf.hi, buff[packet_len-2], buff[packet_len-1]);        
        return;
    }
    //LOG_DEL_CHL..
    Serial.printf("[MSG]WIFI::CALLBAK::Group=%d, IO Channel=%d, Type=%d, Addr=%d, Len=%d\r\n", buff[WIFI_PACKET_GROUP], buff[WIFI_PACKET_CHANNEL], buff[WIFI_PACKET_TYPE], buff[WIFI_PACKET_ADDRESS], buff[WIFI_PACKET_LENGTH] );
                                                                                        //오타 수정 :CHL
    #if 0
    if( peer_channel != wifi_send.peer_addr )
    {
        wifi_send.doing_recv_cb = false;
        return;       //응답한 peer가 요청한 peer가 아니라면 pass
    }
    #endif
    
    data_len = packet_len-8;
    switch (buff[WIFI_PACKET_COMMAND])
    {
        case PEER_PAIRING_OK:           
            if( peer[peer_channel].pairFlag ) 
            {
                switch( wifi_state_machine[peer_channel].peer.state )
                {
                    case WIFI_STATE_PAIRED:
                    case WIFI_STATE_SEND:
                        wifi_state_machine[peer_channel].peer.response = true;
                        Serial.printf("[MSG]WIFI::CALLBAK[%d]::PAIRING_OK\r\n",peer_channel);
                    break;
                }
                break;
            }else;
            
                //PDO 체크해보기.. PDO봐야 알 수 있음... 
            peer[peer_channel].io_usage = buff[WIFI_PACKET_DATA];
            peer[peer_channel].io_page = buff[WIFI_PACKET_DATA+1];
            peer[peer_channel].serial = buff[WIFI_PACKET_DATA+2];

            //프로토콜 수정 후 이거 안나오지 않을까 싶은데... 확인후 삭제
            //usage와 serial이 잘못 들어오는 경우가 있음.... 원인은 아직 못찾음
            //io usage가 1이 아니고, serial 데이타가 있음.....
            //현재는 모드 1 WORD만 사용중
            if( peer[peer_channel].io_usage != 1 ) 
            {
                Serial.printf("[MSG]WIFI::CALLBAK[%d]::PAIRING_OK::io_usage ERR\r\n",peer_channel);
                break;
            }

            wifi_state_machine[peer_channel].pairing.response = true;

            memcpy(peer[peer_channel].mac, src_mac, 6);  //응답 받은 peer mac 저장
            wifi_state_machine[peer_channel].peer.pMac= peer[peer_channel].mac;  //응답 받은 peer mac 저장

            Serial.printf("[MSG]WIFI::CALLBAK[%d]::PAIRING_OK:: usage=%d, page=%d, serial=%d\r\n",peer_channel, peer[peer_channel].io_usage, peer[peer_channel].io_page, peer[peer_channel].serial);

            wifi_state_machine[peer_channel].peer.device_type = buff[WIFI_PACKET_TYPE];

            /////////////////////////////////////////////////////////////////////    
            //연결 초기값 넣기 :: 기존에 정한 것 //3.2 변경

#if FUNC_REPAIR_SAVE_DATA
            if( wifi_state_machine[peer_channel].peer.repair_state==2 )
            {
                wifi_state_machine[peer_channel].peer.repair_state=3;
            }else
            {
                if( peer[peer_channel].io_page > 0 )
                {
                    for( i = 1; i < peer[peer_channel].io_page+1; i++ )
                    {
                        lPeerData[peer_channel][i] = i;
                        lPeerData[peer_channel][i] <<= 12;
                        lPeerData[peer_channel][i] += 2047;  // 0x7FF( 0111 1111 1111 ) - 
                    }
                }
                else
                {
                    lPeerData[peer_channel][0] = 2047;
                }
            }
#else
// D4SL 기준 usage 1, page 0, serial 0 넘어옴 
//
// 
            if( peer[peer_channel].io_page > 0 )
            {
                for( i = 1; i < peer[peer_channel].io_page+1; i++ )
                {
                    lPeerData[peer_channel][i] = i;
                    lPeerData[peer_channel][i] <<= 12;
                    lPeerData[peer_channel][i] += 2047;  // 0x7FF( 0111 1111 1111) - 
                }
            }
            else
            {
                lPeerData[peer_channel][0] = 2047;
            }
#endif

            if( peer[peer_channel].serial > 0 )  
            {
                // 시리얼 영역은 1이 12Ch임, 0 안씀
                for( i = 1; i < 11; i++ )
                {
                    lSPeerData[peer_channel-11][i] = 2047;   // 0x7FF( 0111 1111 1111 )
                }
            }
            
            if( pairing_register(peer_channel, 0) ) wifi_state_machine[peer_channel].pairing.state = true;
        break;
                                    //PEER_PAIRING_OK..

        case PEER_PAIRING_CANCEL:      
            Serial.printf("[MSG]WIFI::CALLBAK[%d]::CANCEL PAIRING OK\n", peer_channel);
            wifi_state_machine[peer_channel].pairing.response = true;
            wifi_state_machine[peer_channel].pairing.state = true;
            del_peer(peer_channel);  
            break;

        case PEER_IO_GET:  

            if( peer[peer_channel].pairFlag == true )
            {
                #if FUNC_PAIRED_VERIFY==1                      // 추가 3.2
                bitWrite(rx_pairing_status, peer_channel, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
                #endif

                wifi_state_machine[peer_channel].peer.response = true;
                if( peer[peer_channel].io_page == 0 )
                {
                    //memcpy(&lPeerData[peer_channel][0], (const uint8_t *)&data[WIFI_PACKET_DATA], packet_len-6);
                    memcpy(&lPeerData[peer_channel][0], &buff[WIFI_PACKET_DATA], data_len);
                    //LOG_DEL_CHL.. 
                    Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA RESPONSE[PAGE0]=0x%04x\n", peer_channel, lPeerData[peer_channel][0]);
                }
                else
                {
                    //memcpy(&lPeerData[peer_channel][1], (const uint8_t *)&data[WIFI_PACKET_DATA], packet_len-6);
                    memcpy(&lPeerData[peer_channel][1], &buff[WIFI_PACKET_DATA], data_len);
                    Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA RESPONSE",peer_channel);
                    for( cnt=0 ; cnt<data_len ; cnt++)
                    {
                        Serial.printf("[%d]:0x%04x ",cnt, lPeerData[peer_channel][cnt+1]);
                    }
                    Serial.printf("\r\n");
                }
                #if FUNC_REPAIR_SAVE_DATA
                wifi_state_machine[peer_channel].peer.repair_counter=0;
                #endif
            }
        break;

        case PEER_IO_SET:
            
            if( peer[peer_channel].pairFlag == true )
            {
                #if FUNC_PAIRED_VERIFY==1                      // 추가 3.2  
                bitWrite(rx_pairing_status, peer_channel, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
                #endif

                wifi_state_machine[peer_channel].peer.response = true;
            }else
            {
                break;
            }

            //set data 일때 응답 데이타가 동일한지 비교할 필요 없음
            if( peer[peer_channel].io_page==0 )
            {
                readData = (uint16_t)buff[WIFI_PACKET_DATA+1]; 
                readData <<= 8;
                readData += buff[WIFI_PACKET_DATA];
                #if 0     

                if( wifi_state_machine[peer_channel].peer.set_io_data == readData )
                {
                    lPeerData[peer_channel][0] = readData;
                    Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA SET:: Page=0, readData = 0x%04x\r\n",peer_channel, readData);
                }
                #else
                lPeerData[peer_channel][0] = readData;
                Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA SET:: Page=0, readData = 0x%04x\r\n",peer_channel, readData);
                #endif
            }else
            {
                //Check set page
                page = uint8_t(wifi_state_machine[peer_channel].peer.set_io_data>>12);
                page = page & 0x07;
                
                #if 0
                //6,7 = 1page, 8,9 = 2page, 10,11 = 3page 12,13 = 4page ~~~
                readData = (uint16_t)buff[page*2+5];
                readData <<= 8;
                readData += buff[page*2+4];

                Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA SET:: Page=0x%02x, Data = 0x%04d\r\n",peer_channel, page, readData);
                //응답 page의 데이타가 맞는지 확인
                if( wifi_state_machine[peer_channel].peer.set_io_data == readData )
                {
                    Serial.printf("RecvCB[%d]::AP_DATA_SET::DATA UPDATE\r\n",peer_channel);
                    //전체 페이지 업데이트
                    memcpy(&lPeerData[peer_channel][1], (const uint8_t *)&data[4], len-4);                
                }
                #else
                memcpy(&lPeerData[peer_channel][1], &buff[WIFI_PACKET_DATA], data_len);

                Serial.printf("[MSG]WIFI::CALLBAK[%d]::DATA SET:: Page=0x%02x ", peer_channel, page);
                for( cnt=0 ; cnt<data_len ; cnt++)
                {
                    Serial.printf("[%d]:0x%04x ",cnt, lPeerData[peer_channel][cnt+1]);
                }
                Serial.printf("\r\n");
                #endif

                #if FUNC_REPAIR_SAVE_DATA
                wifi_state_machine[peer_channel].peer.repair_counter=0;
                #endif
            }
        break;

        case PEER_SERIAL_SET:      // 
            // 0 Page 없으므로 편의상 2차원 배열의 시작은 [][1]
            // 12 Ch는 1Ch로 처리함으로 -11 함

            if( peer_channel > 11 )
            {
                Serial.printf("[MSG]WIFI::CALLBAK[%d]::PEER_ALL_SDATA_RESPONSE\r\n",peer_channel );
                if( peer[peer_channel].pairFlag == true )
                {
                    #if FUNC_PAIRED_VERIFY==1                      // 추가 3.2
                    bitWrite(rx_pairing_status, peer_channel, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
                    #endif
    
                    //IO
                    memcpy(&lPeerData[peer_channel][0], &buff[WIFI_PACKET_DATA],2);
                    //SERIAL
                    wifi_state_machine[peer_channel].peer.response = true;
                    wifi_state_machine[peer_channel].peer.response_type_serial = true;
                    memcpy(&lSPeerData[peer_channel-12][0], &buff[WIFI_PACKET_DATA+2], data_len);  //12 = 0
                    #if FUNC_REPAIR_SAVE_DATA
                    wifi_state_machine[peer_channel].peer.repair_counter=0;
                    #endif
                }
            }
        break;

        case PEER_SERIAL_GET:   //SET과 같은데 일단 나누어 놓음..... 
            if( peer_channel > 11 )
            {
                Serial.printf("[MSG]WIFI::CALLBAK[%d]::PEER_ALL_SDATA_RESPONSE\r\n",peer_channel );
                if( peer[peer_channel].pairFlag == true )
                {
                    #if FUNC_PAIRED_VERIFY==1                       // 추가 3.2      
                    bitWrite(rx_pairing_status, peer_channel, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
                    #endif
    
                    //IO
                    memcpy(&lPeerData[peer_channel][0], &buff[WIFI_PACKET_DATA],2);
                    //SERIAL
                    wifi_state_machine[peer_channel].peer.response = true;
                    wifi_state_machine[peer_channel].peer.response_type_serial = true;
                    memcpy(&lSPeerData[peer_channel-12][0], &buff[WIFI_PACKET_DATA+2], data_len);  //12 = 0
                    #if FUNC_REPAIR_SAVE_DATA
                    wifi_state_machine[peer_channel].peer.repair_counter=0;
                    #endif
                }
            }        
        break;

        default:
            if( peer[peer_channel].pairFlag == true )
            {
                wifi_state_machine[peer_channel].peer.response = true;
                Serial.printf("[MSG]WIFI::CALLBAK::Paired::Command Err=0x%02x\r\n", buff[WIFI_PACKET_COMMAND]);
            }            
        break;
    }
    wifi_send.doing_recv_cb = false;
} // end recv_cb

void send_cb(const uint8_t *des_addr, esp_now_send_status_t status)
{
  // int8_t send_idx = 0;

    for (int i = 0; i <= MAX_PEER; i++)
    {
        if (i == MAX_PEER)
        {
            //send_idx = i;
            break;
        }
        else if (memcmp(peer[i].mac, des_addr, 6) == 0)
        {
            xSemaphoreGive(send_sem); // send list에 있을 경우 semaphore를 릴리즈
            break;
        }
    } 
   //Serial.printf("send_cb::des_addr=0x%x, status=%d\r\n", *des_addr,status );
}


bool wl_init(void)
{
    WiFi.mode(WIFI_STA); 

    if (esp_now_init() != ESP_OK)
        return false;
    if (esp_wifi_start() != ESP_OK)
        return false;

    #if 1
    //if( esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20) == ESP_OK )
    if( esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40) == ESP_OK )
    {
        Serial.printf( "[MSG]WIFI::INIT::Set Bandwidth::OK\r\n");
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::Set Bandwidth::ERR\r\n" );
    }   
    #endif  

    #if 1
    esp_wifi_set_promiscuous(true);
    if( esp_wifi_set_channel(wifi_send.rf_set_channel, WIFI_SECOND_CHAN_NONE) == ESP_OK ) 
    {
        Serial.printf( "[MSG]WIFI::INIT::Set CHANNEL::0x%02x\r\n", wifi_send.rf_set_channel );
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::Set CHANNEL::ERR\r\n" );
    }
    esp_wifi_set_promiscuous(false);
    #endif

    if (esp_wifi_set_promiscuous(true) != ESP_OK)
        return false;
    if (esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_5M_L) != ESP_OK)
        return false; // 1Mbps, 5Mbps, 11Mbps available
    if (esp_now_register_recv_cb(recv_cb) != ESP_OK)
        return false;
    if (esp_wifi_set_promiscuous_rx_cb(promiscuous_rx_cb) != ESP_OK)
        return false;
    if (esp_now_register_send_cb(send_cb) != ESP_OK)
        return false;

    memcpy(&peerInfo.peer_addr, broadcast_addr, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
        return false;

    if( esp_wifi_get_channel( &wifi_send.rf_get_channel, &wifi_send.rf_sencond_channel ) == ESP_OK )
    {
        Serial.printf( "[MSG]WIFI::INIT::GET CHANNEL::Primary=0x%02x\r\n", wifi_send.rf_get_channel );
    }else
    {
        Serial.printf( "[MSG]WIFI::INIT::GET CHANNEL::ERR\r\n" );
    }


    return true;
}

void debug()
{
    Serial.printf("**********디버깅 모드 시작********************\n");
    Serial.printf("시리얼 명령창 마지막에 뉴라인추가, 가로없이 ", "로 쓸것\n");
    Serial.printf("1.EthterCAT io영역 쓰기 : w,(index),(int value)\n");
    Serial.printf("2.EthterCAT io영역 읽기 : r\n");
}

void serial_cmd()
{
    if (Serial.available() > 0)
    {
        uint16_t *in_p = (uint16_t *)&EASYCAT.BufferIn.Cust.pairing_bit;
        uint16_t *out_p = (uint16_t *)&EASYCAT.BufferOut.Cust.pairing_bit;
        String input = "", _idx = "", _value = "";
        char prefix;
        uint8_t index = 0;
        uint16_t value = 0;
        uint8_t idx = 0;
        uint8_t idx2 = 0;

        input = Serial.readStringUntil('\n');
        prefix = input[0];

        debug();
        switch (prefix)
        {
        case 'w':

            idx = input.indexOf(',');
            idx2 = input.indexOf(',', idx + 1);
            _idx = input.substring(idx + 1, idx2);
            _value = input.substring(idx2 + 1);

            index = _idx.toInt();
            value = _value.toInt();

            Serial.printf("wirte ecat memory index# %d, value %d\n", index, value);
            memcpy(&in_p[index], (const uint16_t *)&value, 2);
            break;

        case 'r':
            for (int i = 0; i < 25; i++)
            {
                Serial.printf("output[%d]=%04X,  input[%d]=%04X\n", i, out_p[i], i, in_p[i]);
            }
            break;
        }
    }
}

void Initialize_PDO(void)
{
    uint8_t i;
    uint16_t ltemp;

    memset(&EASYCAT.BufferIn.Cust, 0, sizeof(EASYCAT.BufferIn.Cust));
}

#if 0 //* 내가 만듦. 빠진거 만ㅀ음 
void Wifi_Handle(void)
{
    static uint16_t cnt_loop=0, setTime=SET_TIME_PAIRING;
    uint8_t getAddr = 0;
    uint8_t channel= wifi_send.peer_addr;

    bool save_response=false;
    
    esp_err_t err;

    if( wifi_send.doing_recv_cb ) return; 

    wifi_send.doing_handle = true;

    switch(wifi_state_machine[channel].peer.state)
    {
        case WIFI_STATE_READY:
        if(wifi_state_machine[channel].pairing.request == true)
        {
            //queue확인하고 
            //상태 add로 변경 
            if(Wifi_enQueue(channel))
            {
                wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRING_ADD;
                Serial.printf(" [WIFI]::HANDLE[%d]::PAIRING ADD\r\n",channel);
            }
        }
        break;

        case WIFI_STATE_PAIRING_ADD:
        break;

        case WIFI_STATE_PAIRED://paired 상태에서 del 요청이 들어온다면 삭제
        //queue확인 만약 del 요청오면 상태 del로 변경 
        if(wifi_state_machine[channel].pairing.request == true)
        {
            if(Wifi_enQueue(channel))
            {
                if(wifi_state_machine[channel].pairing.del == true)
                {
                    wifi_state_machine[channel].peer.response = false; //!추가
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRING_DEL;
                    Serial.printf(" [WIFI]::HANDLE[%d]::PAIRING DEL\r\n",channel);
                }
                else if(peer[channel].pairFlag == true)
                {
                    wifi_state_machine[channel].peer.state = WIFI_STATE_WAIT;
                    Serial.printf(" [WIFI]::HANDLE[%d]::PAIRING WAIT\r\n",channel);   
                }
                else
                {
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                    del_peer(channel);
                    Serial.printf(" [WIFI]::HANDLE[%d]:: PAIRED UNKNOWN DEL\r\n",channel);
                }
            }
            else//Full Queue
            {
                Serial.printf(" [WIFI]::HANDLE[%d]:: ERROR QUEUE\r\n",channel);
            }
        }
        else// state가 paired인데 pairing req가 false라서 데이터 save 필요? 
        {
//!
            if(wifi_state_machine[channel].peer.response)
            {
                save_response = true;
                break;
            }
        }

        if(wifi_state_machine[channel].peer.update_serial == true)
        {
            if(Wifi_enQueue(channel))
            {
                wifi_state_machine[channel].peer.update_serial == false;
                //etherCAT handler에서 마지막에 버퍼 처리함 
                Wifi_Peer_Data_Set(channel, AP_SERIAL_SET, wifi_state_machine[channel].peer.set_serial_Buf, 9);
                Serial.printf(" [WIFI]::HANDLE[%d]::STATE::PAIRED::REQUEST::UPDATE::SERIAL\r\n",channel);
            }
        }
        else if(wifi_state_machine[channel].peer.request_serial == true)
        {
            if(Wifi_enQueue(channel))
            {
                wifi_state_machine[channel].peer.request_serial == false;
                Wifi_Peer_Data_Set(channel,AP_SERIAL_GET,wifi_state_machine[channel].peer.set_serial_Buf, 9);
                Serial.printf(" [WIFI]::HANDLE[%d]::STATE::PAIRED::REQUEST::DATA::SERIAL\r\n",channel);
            }
        }
        else if(wifi_state_machine[channel].peer.update_io == true)
        {
            if(Wifi_enQueue(channel))
            {
                wifi_state_machine[channel].peer.update_io == false;
                Wifi_Peer_Data_Set(channel,AP_IO_SET,&wifi_state_machine[channel].peer.set_io_data,1);
                Serial.printf(" [WIFI]::HANDLE[%d]::STATE::PAIRED::REQUEST::UPDATE::IO\r\n",channel);
            }
        }
        else//  request_io case...
        {
            if(Wifi_enQueue(channel))
            {
                Wifi_Peer_Data_Set(channel,AP_IO_GET,&wifi_state_machine[channel].peer.set_io_data,1);
                Serial.printf(" [WIFI]::HANDLE[%d]::STATE::PAIRED::REQUEST::DATA::IO %d\r\n",channel,wifi_state_machine[channel].peer.set_io_data);
            }
        }
        break;

        case WIFI_STATE_PAIRING_DEL:
        if(wifi_state_machine[channel].pairing.add == true)
        {
            if(++wifi_state_machine[channel].pairing.timeout > 5)
            {
                wifi_state_machine[channel].pairing.timeout = 0;
                wifi_state_machine[channel].peer.state = WIFI_STATE_READY;
                Serial.printf(" [WIFI]::HANDLE[%d]::STATE::PAIRING DEL TIMEOUT\r\n",channel);
            }
        }
        break;
        default:
        break;

    }//end switch
                        //* * 데이터 송수신 처리 * *//
            //! !wifi_send.rx_busy[channel])가 아니라 wifi_send.rx_busy[channel]해놨었으..
    if(Wifi_getQueue(&getAddr) && !wifi_send.tx_busy[channel] && !wifi_send.rx_busy[channel]) // peer_channel > 15)
    {
        channel = getAddr;
        
        //전송할거니까 tx_busy set
        wifi_send.tx_busy[channel] = true;
        err = esp_now_send(wifi_state_machine[channel].peer.pMac,(const uint8_t *)wifi_state_machine[channel].peer.txBuf,wifi_state_machine[channel].peer.txLen);
        switch (err)
        {
        case ESP_OK:
            wifi_send.tx_busy[channel] = false;
            wifi_send.rx_busy[channel] = true;
            //! 이거 빠짐 ㅠ
            wifi_send.peer_req = true;
            
            Serial.printf(" [WIFI]::HANDLE[%d]::SEND OK::Channel=0x%02x, Command=0x%02x, Type=0x%02x, Addr=0x%02x\r\n",channel,wifi_state_machine[channel].peer.txBuf[1],wifi_state_machine[channel].peer.txBuf[2], wifi_state_machine[channel].peer.txBuf[3], wifi_state_machine[channel].peer.txBuf[4]);
            break;
        
        default:
            Serial.printf(" [WIFI]::HANDLE[%d] SEND ERROR=%d\r\n",channel,err);
            break;
        }
    }
    //busy일때 조건 처리 
    if(wifi_send.tx_busy[channel])  
    {
        //* Wait... *//
    }
    else if(wifi_send.rx_busy[channel])
    {
        if(wifi_state_machine[channel].pairing.response == true)
        {
            //* flags reset
            wifi_send.peer_req = false;
            wifi_state_machine[channel].pairing.response = false;
            wifi_state_machine[channel].pairing.request = false;
            wifi_send.rx_busy[channel] = false;
            
            //* 페어링 정상 상태 처리
            if(wifi_state_machine[channel].pairing.state == true)
            {
                //* Pairing flag reset
                wifi_state_machine[channel].pairing.state = false;
                Serial.printf(" [WIFI]::HANDLE[%d]::RESPONSE:: PAIRING OK..\r\n",channel);
                
                if(wifi_state_machine[channel].pairing.add == true) //추가면 상태를 PAIRED로 
                {
                    //* flag reset
                    wifi_state_machine[channel].pairing.add = false;
                    //*Set STATE..
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRED;
                }
                else if(wifi_state_machine[channel].pairing.del == true)
                {
                    //* flag reset & 초기화
                    wifi_state_machine[channel].pairing.del = false;
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine));
                    //*                 Q. 여기에 DEL 상태천이 필요없나? ...
                }else;
            }
            else //정상응답이 아니면 
            {
                //* case1. 페어링 추가 요청이었는데 페어링이 안되었다면 재시도 3번.          
                if(wifi_state_machine[channel].pairing.add && !peer[channel].pairFlag)
                {
                    Serial.printf(" [WIFI]::HANDLE[%d]::RESPONSE::ADD-NonPaired\r\n",channel);
                    //* 재시도 3회  
                    if(++wifi_state_machine[channel].pairing.retry > 3 )
                    {
                        //* 초기화 
                        wifi_state_machine[channel].pairing.retry = 0;
                        memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine));
                    }
                    else //* 상태 천이
                    {   //! state set 첫 번째 인자에 ADD말고 ready박음 ㅋ
                        Wifi_Peer_State_Set(WIFI_STATE_PAIRING_ADD,channel,AP_PAIRING_REQ,peer[channel].typeAddr,0);
                        wifi_state_machine[channel].peer.state = WIFI_STATE_READY;
                        Serial.printf(" [WIFI]::HANDLE[%d]RESPONSE::ERROR.. SO RETRY\r\n",channel);
                    }
                //* case2. 페어링 삭제 요청이었는데 페어링이 유지된다면 
                }else if(wifi_state_machine[channel].pairing.del && peer[channel].pairFlag)
                {
                    Serial.printf(" [WIFI]::HANDLE[%d]::RESPONSE::DEL-PAIRED\r\n",channel);
                    //* 재시도 3회
                    if(++wifi_state_machine[channel].pairing.retry > 3 )
                    {
                        //* 초기화 
                        wifi_state_machine[channel].pairing.retry = 0;
                        memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                        del_peer(channel);
                    }
                    else //* 상태 천이 
                    {
                        Wifi_Peer_State_Set(WIFI_STATE_PAIRING_DEL,channel,AP_PAIRING_CANCEL,0,0);
                        wifi_state_machine[channel].peer.state = WIFI_STATE_READY;
                        wifi_send.tx_busy[channel] =false;//여기 왜reset시키는지 모르것다 ㅜ 위에서 조건분기되는데 흠 느낌적인 그냥 ? 
                    }
                //* case3. 예외처리, 강제 초기화 
                }else
                {
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                    peer[channel].pairFlag = false; //* del peer fnt에서도 flag reset 하긴 하는데 한 번 더 하는듯한?
                    del_peer(channel);
                }
            }
            
        }else
        {//! 빼먹음 
            if(wifi_state_machine[channel].peer.response == true)
            {
                save_response = true;
            }
        }
    }else; // !rx_busy ...
//? peer pairFlag, pairing req, 
    if(save_response)
    {
        //여기도 변수 잘못넣음 response임 
        wifi_state_machine[channel].peer.response = false;
        wifi_send.rx_busy[channel]= false;
        wifi_send.peer_req = false;

        if(wifi_state_machine[channel].peer.response_type_serial == true)
        {
            wifi_state_machine[channel].peer.response_type_serial = false;
            Serial.printf(" [WIFI]::HANDLE[%d]::RESPONSE::SERIAL::D[0=%d\r\n]",channel,lPeerData);
        }

        if(peer[channel].io_page == true)
        {
            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::IO-PAGE::D[1]=%d, D[2]=%d, D[3]=%d, D[4]=%d\r\n",channel, lPeerData[channel][1], lPeerData[channel][2], lPeerData[channel][3], lPeerData[channel][4]);
        }
        else
        {
            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::IO::D[0]=%d\r\n",channel, lPeerData[channel][0]);
        }

        if(peer[channel].pairFlag == false) //페어링이 유지 되고있지 않다면 
        {
            memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine));
        }
    }else; //save_resp end

    if(++cnt_loop > setTime) //set Time 59ms... 60ms * 16 = 960ms . 
    {
        cnt_loop = 0; //! 각 queue 지점에 초기화 안넣어줬음 일단 
        wifi_send.tx_busy[channel] = false;
        
        if(wifi_send.peer_req == true) //* peer req flag는 잘 송신했을 때 set됨 
        {//다음 순서까지 peer_req인데 응답이 없어서 flag 살아있다면 
            wifi_send.peer_req = false;
            if(++wifi_state_machine[channel].peer.cnt_disconnet > 20)
            {
                //* 강제 DEL
                wifi_state_machine[channel].peer.cnt_disconnet = 0;
                wifi_send.rx_busy[channel] = false;
                
                if(peer[channel].pairFlag == true)
                {
                    del_peer(channel);
                }
                memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                Serial.printf(" [WIFI]::HANDLE[%d]::TIME OUT::DEL PEER\r\n",channel);
            }
            else //paired state 그대로 
            {
                if(peer[channel].pairFlag == true)
                {
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::TIME OUT::Disconnet Counter=%d\r\n", channel, wifi_state_machine[channel].peer.cnt_disconnet);
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRED;
                }
                else//paired state가 아닌 req 상태면 초기화 해버림 
                {
                    wifi_send.rx_busy[channel] = false; 
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                }
            }
        }  
        else//loopTime늦어지고 송신 문제 생겼다면
        {
            if(peer[channel].pairFlag == true) // pairFlag는 피어등록하면 set됨 
            {
                if(wifi_state_machine[channel].pairing.del == true)
                {
                    wifi_send.rx_busy[channel] = false;
                    Serial.printf(" [WIFI]::HANDLE[%d]::TIME OUT::Del Peer::No response from paired peer, Pairing Del=%d\r\n",channel);
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                    del_peer(channel);
                }
                else // state 유지 ...  
                {
                    wifi_state_machine[channel].pairing.state = WIFI_STATE_PAIRED;
                }
            }
            else //! case 이해가 안 됨!!! 
            { //* peer data를 받으면 pairing response set됨 
                if(wifi_state_machine[channel].pairing.response == true)
                {
                    Serial.printf("WIFI HANDLE[%d]::TIME OUT::Del Peer::No Rx Busy Check\r\n",channel);
                    memset(&wifi_state_machine[channel],0,sizeof(wifi_state_machine[channel]));
                }
                else if(wifi_state_machine[channel].pairing.request == true)
                {
                    wifi_state_machine[channel].pairing.request = false;
                }
            }
        }
        Wifi_Peer_Rotation();
    } 
    wifi_send.doing_handle = false;
}
#else

void Wifi_Handle(void)
{
    static uint16_t cnt_loop=0, setTime=SET_TIME_PAIRING;
    static uint8_t send_cmd=0;
    uint8_t channel=wifi_send.peer_addr;
    uint8_t getAddr=0;
    bool save_response=false;
    esp_err_t err;

   // output_toggle(LED);
                        //* 페어링 끊기면 Queue에서 채널 삭제 해야하지 않을까?
    if( wifi_send.doing_recv_cb ) return;
    wifi_send.doing_handle = true;
    
    switch( wifi_state_machine[channel].peer.state )
    {
        case WIFI_STATE_READY:
            //페어링 요청 확인하고 queue에 넣고 상태 변경
            wifi_state_machine[channel].pairing.timeout = 0;
            if( wifi_state_machine[channel].pairing.request == true )
            {
                if( Wifi_enQueue(channel) )
                {
                    if( wifi_state_machine[channel].pairing.add )
                    {
                        wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRING_ADD;
                        cnt_loop = 0;
                        setTime = SET_TIME_PAIRING;
                        Serial.printf("[MSG]WIFI::HANDLE[%d]::PAIRING ADD\r\n",channel);

                    }
                }
            }
        break;

        case WIFI_STATE_PAIRING_ADD:
        break;

        case WIFI_STATE_PAIRED: 
            //페어링 요청 확인하고 queue에 넣고 상태 변경
            wifi_state_machine[channel].pairing.timeout = 0;
            if( wifi_state_machine[channel].pairing.request == true )
            {
                setTime = SET_TIME_PAIRING;
                if( Wifi_enQueue(channel) )
                {
                    if( wifi_state_machine[channel].pairing.del )
                    {
                        wifi_state_machine[channel].peer.response = false; //이전 call back 응답이 있을 수 있음.
                        wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRING_DEL;
                        //pairing case는 delay를 더 100ms로 조정
                        cnt_loop = 0;
                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::DEL\r\n",channel );

                        //이더캣 요청에 의해서 unpairing시 
                        #if FUNC_REPAIR_SAVE_DATA
                        wifi_state_machine[channel].peer.repair_counter=0;
                        wifi_state_machine[channel].peer.repair_state=0; 
                        #endif
                    }else if( peer[channel].pairFlag == true ) //이미 페어링 되어 있음
                    {
                        wifi_state_machine[channel].peer.state = WIFI_STATE_WAIT;
                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::WAIT\r\n",channel );
                    }else   
                    {
                        //이상한 상태 //초기화 시킴
                        memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                        del_peer(channel);
                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::UNKNOWN - DEL\r\n",channel );

                        //이더캣 요청에 의해서 unpairing시 
                        #if FUNC_REPAIR_SAVE_DATA
                        wifi_state_machine[channel].peer.repair_counter=0;
                        wifi_state_machine[channel].peer.repair_state=0; 
                        #endif
                    }
                }else
                {
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::NONE QUE\r\n",channel );
                }
            }else   //paired인데, plc에서 pairing del 요청이 없을 경우 : 주기적으로 해당 peer에 데이타 요청
            {
                //받아둔 데이타가 있다면
                if( wifi_state_machine[channel].peer.response )
                {
                    save_response = true;
                    break;
                }

                if( wifi_state_machine[channel].peer.update_serial )
                {
                    if( Wifi_enQueue(channel) )
                    {
                        wifi_state_machine[channel].peer.update_serial= false;    
                        Wifi_Peer_Data_Set(channel, AP_SERIAL_SET, wifi_state_machine[channel].peer.set_serial_Buf, 9);
                        //아래 초기화는 어차피 update_send 할때 설정되기 때문에 없어도 괜찮음
                        cnt_loop = 0;
                        setTime = SET_TIME_PAIRD;

                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::REQUEST::UPDATE::SERIAL\r\n",channel );
                    }                    
                }else if( wifi_state_machine[channel].peer.request_serial ) //Serial data 요청
                {
                    if( Wifi_enQueue(channel) )
                    {
                        wifi_state_machine[channel].peer.request_serial= false;    
                        Wifi_Peer_Data_Set(channel, AP_SERIAL_GET, wifi_state_machine[channel].peer.set_serial_Buf, 9);
                        //아래 초기화는 어차피 update_send 할때 설정되기 때문에 없어도 괜찮음
                        cnt_loop = 0;
                        setTime = SET_TIME_PAIRD;

                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::REQUEST::DATA::SERIAL\r\n",channel );
                    }                    
                }else if( wifi_state_machine[channel].peer.update_io )
                {
                    if( Wifi_enQueue(channel) )
                    {
                        wifi_state_machine[channel].peer.update_io = false;    
                        Wifi_Peer_Data_Set(channel, AP_IO_SET, &wifi_state_machine[channel].peer.set_io_data, 1);
                        //아래 초기화는 어차피 update_send 할때 설정되기 때문에 없어도 괜찮음

                        cnt_loop = 0;
                        setTime = SET_TIME_PAIRD;
                        Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRED::REQUEST::UPDATE::IO\r\n",channel );
                    }
                }else //IO data 요청
                {
                    if( Wifi_enQueue(channel) ) //d4sl에 보냄
                    {
                        setTime = SET_TIME_PAIRD;
                        Wifi_Peer_Data_Set(channel, AP_IO_GET, &wifi_state_machine[channel].peer.set_io_data, 1);
                        //! Modified... : CHL, 05.15
                        Serial.printf("[**changchang**]::HANDLE[%d]::STATE::PAIRED::REQUEST::DATA::IO=%d\r\n",channel,wifi_state_machine[channel].peer.set_io_data);
                    }                                                                                   
                }

                wifi_state_machine[channel].peer.state = WIFI_STATE_SEND;
            }
        break;

        case WIFI_STATE_PAIRING_DEL:
            if( wifi_state_machine[channel].pairing.add == true )
            {
                if( ++wifi_state_machine[channel].pairing.timeout > 5 )
                {
                    wifi_state_machine[channel].pairing.timeout = 0;
                    wifi_state_machine[channel].peer.state = WIFI_STATE_READY;

                    Serial.printf("[MSG]WIFI::HANDLE[%d]::STATE::PAIRING_DEL::TIMEOUT\r\n",channel);
                }
            }
        break;

        case WIFI_STATE_SEND:

        break;

        case WIFI_STATE_RECEIVE:
        break;

        case WIFI_STATE_WAIT:
        break;

        case WIFI_STATE_RETRY:
        //    Serial.printf("WIFI HANDLE[%d]::RE-TRY\r\n",channel);
        break;
    }
    if( Wifi_getQueue(&getAddr) && !wifi_send.tx_busy[channel] && !wifi_send.rx_busy[channel] )
    {   
        channel=getAddr;
        cnt_loop = 0;
        wifi_send.tx_busy[channel] = true;
        err = esp_now_send(wifi_state_machine[channel].peer.pMac, (const uint8_t *)wifi_state_machine[channel].peer.txBuf, wifi_state_machine[channel].peer.txLen);
        switch( err )
        {
            case ESP_OK:
                wifi_send.tx_busy[channel] = false;
                wifi_send.rx_busy[channel] = true;
                wifi_send.peer_req = true;
                //LOG_DEL_CHL.. 
                Serial.printf("[MSG]WIFI::HANDLE[%d]::SEND OK::Channel=0x%02x, Command=0x%02x, Type=0x%02x, Addr=0x%02x\r\n",channel,wifi_state_machine[channel].peer.txBuf[1],wifi_state_machine[channel].peer.txBuf[2], wifi_state_machine[channel].peer.txBuf[3], wifi_state_machine[channel].peer.txBuf[4]);
            break;

            case ESP_ERR_ESPNOW_NOT_FOUND:
            break;

            default:
                Serial.printf("[MSG]WIFI::HANDLE[%d]::SEND ERR=%d\r\n",channel, err);
            break;
        }
    }
    //* * 데이터 송수신 시...  * */
    if( wifi_send.tx_busy[channel] )            
    {
        //Wait...                           
    }else if( wifi_send.rx_busy[channel]) //데이터 수신
    {
        //check of peer response 
        if( wifi_state_machine[channel].pairing.response )// 아래ㅇ는 수신된 데이터에 대한 응답 처리
        {
            wifi_send.peer_req = false;
            //Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::PAIRING\r\n",channel);

            wifi_state_machine[channel].pairing.response = false;
            wifi_state_machine[channel].pairing.request = false;
            wifi_send.rx_busy[channel] = false;

            if( wifi_state_machine[channel].pairing.state==true )//페어링 정상 응답 완료
            {
                Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::PAIRING OK\r\n",channel);
                wifi_state_machine[channel].pairing.state=false;
                //reset pairing flag

                if( wifi_state_machine[channel].pairing.add )//아래 elseif에서 페어링 추가 요청이었는데 페어링이 안붙었을때 stateSet cmd 들어가면 true됨
                {
                    wifi_state_machine[channel].pairing.add = false;            
                    //set peer state
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRED;
                }else if( wifi_state_machine[channel].pairing.del )
                {
                    wifi_state_machine[channel].pairing.del = false;
                    memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                }
            }else
            {   
                if( wifi_state_machine[channel].pairing.add && !peer[channel].pairFlag )
                {
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::Add-NonPaired\r\n",channel);
                    //페어링 추가 요청이었는데, 페어링이 안되었다면... 재시도 3번

                        if( ++wifi_state_machine[channel].pairing.retry > 3 )
                        {
                            wifi_state_machine[channel].pairing.retry = 0;
                            memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                        }else
                        {
                            Wifi_Peer_State_Set(WIFI_STATE_PAIRING_ADD, channel, AP_PAIRING_REQ, peer[channel].typeAddr, 0);
                            wifi_state_machine[channel].peer.state = WIFI_STATE_READY;
                            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::NG - Retry\r\n",channel);
                        }

                }else if( wifi_state_machine[channel].pairing.del && peer[channel].pairFlag )
                {
                    //페어링 제거 요청이었는데, 제거가 안됐다면
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::Del-Paired\r\n",channel);
                    if( ++wifi_state_machine[channel].pairing.retry > 3 )
                    {
                        wifi_state_machine[channel].pairing.retry = 0;
                        memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                        //peer 강제 제거
                        del_peer(channel);

                        //ethercat도 클리어...
                    }else
                    {
                        Wifi_Peer_State_Set(WIFI_STATE_PAIRING_DEL, channel, AP_PAIRING_CANCEL, 0, 0);
                        wifi_state_machine[channel].peer.state = WIFI_STATE_READY;                        
                        wifi_send.tx_busy[channel] = false;
                    }                
                }else
                {
                    //예외처리, 페어링 프로세스 강제 초기화
                    memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                    peer[channel].pairFlag=false;
                    del_peer(channel);
                }
            }
        }else//response가 꺼져있거나,.. 
        {
            //일반 응답 
            if( wifi_state_machine[channel].peer.response )
            {
                save_response = true;
                #if FUNC_REPAIR_SAVE_DATA
                wifi_state_machine[channel].peer.repair_timeout=0;
                wifi_state_machine[channel].peer.repair_counter=0;
                wifi_state_machine[channel].peer.repair_state=0;
                #endif
            }
        }
    }

    if( save_response )
    {
        wifi_state_machine[channel].peer.response = false;
        wifi_state_machine[channel].peer.cnt_disconnet = 0;
        wifi_send.rx_busy[channel] = false; 
        wifi_send.peer_req = false;

        if( wifi_state_machine[channel].peer.response_type_serial )
        {
            wifi_state_machine[channel].peer.response_type_serial = false;

            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::Seiral::D[0]=%d\r\n",channel, lPeerData[channel][0]);
        }

        if( peer[channel].io_page )
        {
            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::IO-PAGE::D[1]=%d, D[2]=%d, D[3]=%d, D[4]=%d\r\n",channel, lPeerData[channel][1], lPeerData[channel][2], lPeerData[channel][3], lPeerData[channel][4]);
        }else
        {
            //LOG_DEL_CHL.. 
            Serial.printf("[MSG]WIFI::HANDLE[%d]::RESPONSE::IO::D[0]=%d\r\n",channel, lPeerData[channel][0]);
        }


        if( peer[channel].pairFlag==false )    
        {
            memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
        }
    }

    if( ++cnt_loop>setTime ) //60ms, 60 * 16 = 960ms : this is 16 device scan loop time
    {
        cnt_loop = 0;
        wifi_send.tx_busy[channel] = false;
        //wifi_send.rx_busy[addr] = false;

        //Send timeout //esp now send가 잘 보내졌을 때 peer_req true됨 
        if( wifi_send.peer_req )    //다음 순서까지 peer_req인데 응답이 없어서 flag가 살아 있다면
        {
            wifi_send.peer_req = false;
            if( ++wifi_state_machine[channel].peer.cnt_disconnet>=20 ) 
            {
                //강제 del
                wifi_send.rx_busy[channel] = false;
                wifi_state_machine[channel].peer.cnt_disconnet=0;
                if( peer[channel].pairFlag ) del_peer(channel);  
                memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                Serial.printf("[MSG]WIFI::HANDLE[%d]::TIME OUT::Del Peer\r\n",channel);

                #if FUNC_REPAIR_SAVE_DATA
                wifi_state_machine[channel].peer.repair_state=1;
                #endif
            }else
            {
                //페어드는 페어드 상태로
                if( peer[channel].pairFlag == true)
                {
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::TIME OUT::Disconnet Counter=%d\r\n", channel, wifi_state_machine[channel].peer.cnt_disconnet);
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRED; 
                }else   
                {
                    //페어링 요청중이면 초기 상태로
                    wifi_send.rx_busy[channel] = false;
                    memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                }
            }
        }else
        { 
            if( peer[channel].pairFlag == true)    //요청 상태 없이 타임아웃되었을 경우
            {
                if( wifi_state_machine[channel].pairing.del )
                {
                    wifi_send.rx_busy[channel] = false;
                    Serial.printf("[MSG]WIFI::HANDLE[%d]::TIME OUT::Del Peer::No response from paired peer, Pairing Del=%d\r\n",channel);
                    memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                    del_peer(channel);    
                }else
                {
                    wifi_state_machine[channel].peer.state = WIFI_STATE_PAIRED;
                }
            }else
            {   //! case, 주석 이해 안 됨 ㅠㅜ 
                //예외처리 : delete를 진행했는데, 다른 응답으로 rx_busy가 클리어 되어 있을 경우.
                if( wifi_state_machine[channel].pairing.response )
                {
                    Serial.printf("WIFI HANDLE[%d]::TIME OUT::Del Peer::No Rx Busy Check\r\n",channel);
                    memset(&wifi_state_machine[channel],0, sizeof(wifi_state_machine[channel]));
                }else if( wifi_state_machine[channel].pairing.request )
                {
                    wifi_state_machine[channel].pairing.request = false;
                }
            }
        } 
        Wifi_Peer_Rotation(); //* FAST SCAN으로 변경... 05/13
    }    
    wifi_send.doing_handle = false;       
    //& 현재 Queue 상태 체크 
#if 0
    Serial.printf(" [QUEUE]::");//몇개 들어가있는지 체크 예정 
#endif
}
#endif


// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR TimerHandler0(void *timerNo)
{
    // SysTick_Set(true);
    TICK.f_ms1 = 1;

    return true;
    ///////////////////////////////////////////////////////////
}

void setup()
{
    Serial.begin(230400);   //* 115200 -> 230400, 05.13부
    //Serial.begin(115200);
    while(!Serial);
    //Serial.printf("[MSG]BOARD INFO::%s::VERSION:%s\r\n",AP,VERSION);//CHL:temp...
    set_board_Version();
    Set_GPIO();
    Read_Rotary();
    Serial.println("");


    send_sem = xSemaphoreCreateBinary();
    recv_sem = xSemaphoreCreateBinary();
    tx_sem = xSemaphoreCreateBinary();
    rx_sem = xSemaphoreCreateBinary();

    if( EASYCAT.Init() )
    {
        Serial.printf("[MSG]SYSTEM INFO::WIFI::ECAT OK\r\n");
    }else
    {
        Serial.printf("[MSG]SYSTEM INFO::WIFI::ECAT NG\r\n");
    }

    if( wl_init() ) 
    {
        Serial.printf("[MSG]SYSTEM INFO::WIFI::INIT OK\r\n");
    }else
    {
        Serial.printf("[MSG]SYSTEM INFO::WIFI::INIT NG\r\n");
    }


    uint32_t ll32temp;

    EASYCAT.SPIWriteRegisterIndirect(iAddr, ALIAS_REG_H, 2);
    delay(100);

    ll32temp = EASYCAT.SPIReadRegisterIndirect(ALIAS_REG_H, 2);
    Serial.printf("[MSG]SYSTEM INFO::ECAT::0x0012 : 0x%x(%d)\n", ll32temp, ll32temp );
    
    Initialize_PDO();
    delay(100);
    EASYCAT.MainTask();

    delay(1000);
  ////////////////////////////////////////////////////////////////////////////////
	// Using ESP32  => 80 / 160 / 240MHz CPU clock ,
	// For 64-bit timer counter
	// For 16-bit timer prescaler up to 1024
	// Interval in microsecs

	if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
	{
		Serial.print(F("[MSG]SYSTEM INFO::Starting  ITimer0 OK\r\n"));
		//Serial.println(millis());
	}
}
//uint8_t togl2 =0;
void loop()
{
    static unsigned int input_cnt=0;
    static bool cnt_set = false;

    if( TICK.f_ms1 )
    {
        TICK.f_ms1 = 0;
        Tick_Handle();
        digitalWrite(DEBUG__CHANG,0);
        Wifi_Handle();
        digitalWrite(DEBUG__CHANG,0);
        if( TICK.flag.bit_flag.ms10 ) Ethercat_Handle();
        if( TICK.flag.bit_flag.ms25 )
        {
            if( !digitalRead(SW) )
            {
                if( cnt_set == false )
                {
                    if( ++input_cnt>80 ) 
                    {
                        debug_out ^= 1;
                        cnt_set = true;
                    }
                }
            }
            else
            {
                cnt_set = false;
                input_cnt = 0;
            }
        }
#if 0
        if(TICK.flag.bit_flag.ms250)
        {
            digitalWrite(DEBUG__CHANG,togl2); 
            Serial.printf("togg...:%d\r\n",togl2);
            togl2 ^= 1;
        }
#endif     
        if( TICK.flag.bit_flag.ms500 )
        {
            if( EASYCAT.BufferIn.Cust.pairing_bit ) 
            {
                output_toggle(LED);
            }else;
        }
        EASYCAT.MainTask();
    }
    
} // end loop
#line 1 "C:\\개인폴더\\FirmWare\\무선화FW\\TEST\\CHANG__cust_0508\\JUST_TEST\\APv3.2_cust\\common.ino"

#include "common.h"

void output_toggle( uint8_t PIN )
{
    if( digitalRead(PIN) )          digitalWrite(PIN, 0);
    else if( !digitalRead(PIN) )    digitalWrite(PIN, 1);

}

uint16_t crc16_modbus(uint16_t init_crc, uint8_t* dat, uint16_t len)
{
    uint8_t crc[2];
    uint16_t tmp;
    crc[0] = init_crc >> 8;
    crc[1] = init_crc & 0xFF;
    for (uint16_t i = 0; i < len; i++) {
        tmp = crc_16_table[crc[0] ^ dat[i]];
        crc[0] = (tmp & 0xFF) ^ crc[1];
        crc[1] = tmp >> 8;
    }
    return crc[0] | ((uint16_t)crc[1] << 8);
}

