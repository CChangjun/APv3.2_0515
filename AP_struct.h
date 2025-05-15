#ifndef AP_STRUCT_H
#define AP_STRUCT_H

#define WIFI_HANDLE             1
#define WIFI_SEND_BUF_MAX       40
#define WIFI_SEND_QUEUE_MAX     50
#define MAX_PEER                16      // peer 최대수

typedef struct
{
    uint8_t mac[6];         // mac 주소
    uint16_t typeAddr;        // typeID
    bool regFlag;           // 등록 여부, 등록 삭제시 false,
    bool pairFlag;          // 페어링 상태, 알람 발생 시 false 닫혀있는게 1 열린게 0 이란것과 동?
    uint8_t io_usage;       // IO영역 점유 word (In/out)
    uint8_t io_page;        // IO영역 페이지 사용, 1이상일 경우 Tx요청에 의해 출력값 선택
    uint8_t serial;         // Serial. page 사용, 0일 경우 Serial 미사용, 1이상일 경우 Serial. 사용
} peer_t;
//* pairFlag peer 등록하면 set


typedef struct _tick_t
{
    bool f_ms1;

    uint16_t cnt_ms2;
    uint16_t cnt_ms5;
    uint16_t cnt_ms10;
    uint16_t cnt_ms25;
    uint16_t cnt_ms50;
    uint16_t cnt_ms100;
    uint16_t cnt_ms250;
    uint16_t cnt_ms500;
    uint16_t cnt_ms750;
    uint16_t cnt_sec1;
    uint16_t cnt_sec2;
    
    union
    {
        uint16_t bit_F;
        struct
        {
            uint16_t ms2 : 1;
            uint16_t ms5 : 1;
            uint16_t ms10 : 1;
            uint16_t ms25 : 1;

            uint16_t ms50 : 1;
            uint16_t ms100 : 1;
            uint16_t ms250 : 1;
            uint16_t ms500 : 1;

            uint16_t ms750 : 1;
            uint16_t sec1 : 1;
            uint16_t sec2 : 1;
            uint16_t sec3 : 1;

            uint16_t sec4 : 1;
            uint16_t sec5 : 1;
            uint16_t sec10 : 1;
            uint16_t min1 : 1;
        } bit_flag;
    } flag;
} _Tick_t;

typedef struct _wifi_state_machine_t
{
    struct
    {
        bool request;
        bool response;//* peer_ok cmd 받으면 pairing response set 
        bool add;
        bool del;
        bool state; //recv_cb에서 페어링하면 set시킴

        uint8_t retry;
        uint16_t timeout;
    }pairing;

    struct 
    {
        bool rw;
        bool response;
        bool response_type_serial;
        bool update_io;        //wifi H에서 state- 사용
        bool update_serial;        //wifi H에서 state- 사용
        bool request_serial;           //wifi H에서 state- 사용

        uint8_t state; //상태 천이 여기@
        uint8_t *pMac;
        uint8_t device_type;
        uint8_t device_addr;
        uint8_t channel;
        uint8_t send_cmd;
        uint8_t txBuf[WIFI_SEND_BUF_MAX];
        uint16_t txLen;
        uint8_t repair_counter;
        uint8_t repair_state;
        uint32_t repair_timeout;

        uint16_t set_io_data;
        uint16_t set_serial_Buf[WIFI_SEND_BUF_MAX];
        uint16_t bak_send;
        uint32_t cnt_disconnet;
    }peer;

} wifi_state_machine_t;

typedef struct _wifi_send_t
{
    struct 
    {
        uint8_t tail;
        uint8_t head;
        uint8_t item[WIFI_SEND_QUEUE_MAX];
    }queue;
    
    bool tx_busy[MAX_PEER]; //채널이 데이터 전송 중
    bool rx_busy[MAX_PEER]; //채널이 데이터 수신 중

    uint8_t peer_addr;
    bool peer_req; //위 pairing.response와 짝궁

    bool doing_recv_cb;
    bool doing_handle;

    uint8_t paired_cnt;

    uint8_t rf_set_channel;
    uint8_t rf_get_channel;

    uint8_t rf_set_group;

    wifi_second_chan_t rf_sencond_channel;

} wifi_send_t;

typedef struct
{
    unsigned frame_ctrl : 16;
    unsigned duration_id : 16;
    uint8_t addr1[6];   // src address
    uint8_t addr2[6];   // recv address
    uint8_t addr3[6];   // broadcast address
    uint8_t addr4[6];   // non 
    unsigned sequence_ctrl : 16;
} wifi_ieee80211_mac_hdr_t;

typedef struct
{
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0];
} wifi_ieee80211_packet_t;



#endif