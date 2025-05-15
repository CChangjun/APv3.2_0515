#ifndef _WIFI_H
#define _WIFI_H
#include <esp_now.h>
#include "esp_wifi.h"
#include "AP_struct.h"
#include <Arduino.h> 
#include "common.h"
#include "AP_SLAVE.h"
#include "Queue.h"

#define AP_PAIRING_REQ              0x01      
#define AP_PAIRING_CANCEL           0x02
#define AP_IO_GET                   0x03      
#define AP_IO_SET                   0x04
#define AP_SERIAL_SET               0x05
#define AP_SERIAL_GET               0x06

#define TEST_POINT                  1           //CHL:temp...

/*Wifi Header fnt*/
bool pairing_register(uint8_t idx, uint16_t channel);
bool Wifi_Set_Channel(uint8_t channel);
void del_peer(uint8_t index);
void Wifi_Peer_Data_Set(uint8_t channel, uint8_t cmd, uint16_t *tx_data, uint8_t wlen);
void Wifi_Peer_State_Set(WIFI_STATE_MACHINE state, uint8_t channel, uint8_t cmd, uint16_t typeAddr, uint16_t wLen);
void Wifi_Peer_Rotation(void);


peer_t peer[MAX_PEER];
esp_now_peer_info_t peerInfo;
wifi_state_machine_t wifi_state_machine[MAX_PEER];
wifi_send_t wifi_send;

uint16_t rx_pairing_status = 0;

uint8_t broadcast_addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // broadcast Mac address

int RSSI[MAX_PEER];


// promiscuous_rx call back 함수, recev packet 분석, RSSI
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)//893 recv_cb
{
    if (type != WIFI_PKT_MGMT)
        return;
    
    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    for (int i = 0; i < MAX_PEER; i++)
    {
        if (memcmp(peer[i].mac, hdr->addr2, 6) == 0)
        {
            RSSI[i] = ppkt->rx_ctrl.rssi;
            #if 0
            Serial.printf("\r\n"); //:CHL
            Serial.printf("[****][RSSI_TEST]promiscuous : %d\n", RSSI[i]); //:CHL
            #endif
            break;
        }
    }
}

bool pairing_register(uint8_t idx, uint16_t channel)     // recv_cb에서 PEER_PAIRING_OK(9)로 수신했을때 실행
{
    // esp_err_t pairing_result;
    memcpy(&peerInfo.peer_addr, peer[idx].mac, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    esp_err_t pairing_result = esp_now_add_peer(&peerInfo);

    if ( pairing_result == ESP_OK )
    {
        peer[idx].pairFlag = true;

        #if FUNC_PAIRED_VERIFY==0
        bitWrite(rx_pairing_status, idx, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
        #endif

        Serial.printf("[MSG]WIFI::MODULE::REGISTER[%2d] : OK\n", idx);
        
        //페어링 완료

        return true;
    }
    else if ( pairing_result == ESP_ERR_ESPNOW_EXIST )
    {
        peer[idx].pairFlag = true;
        
        #if FUNC_PAIRED_VERIFY==0
        bitWrite(rx_pairing_status, idx, 1);  //페어드 이후 데이타 요청이 성공 했을때 SET
        #endif

        Serial.printf("[MSG]WIFI::MODULE::REGISTER[%2d] : Already Paired\n", idx);
        return true;    //버그수정, 추가 했음 : HDJung
    }
    else
    {
        Serial.printf("[MSG]WIFI::MODULE::REGISTER[%2d] : Failed\n", idx);
        return false;
    }
}

bool Wifi_Set_Channel(uint8_t channel)
{
    bool rtn=true;

    esp_wifi_set_promiscuous(true);
    if( esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) == ESP_OK ) 
    {
        Serial.printf( "[MSG]WIFI::SET CHANNEL::Primary=0x%02x\r\n", channel );
    }else
    {
        Serial.printf( "[MSG]WIFI::SET CHANNEL::ERR\r\n" );
    }
    esp_wifi_set_promiscuous(false);

    return rtn;
}

void del_peer(uint8_t index)
{ // delete peer from pairing info and index
    peer[index].pairFlag = false;
    esp_now_del_peer(peer[index].mac);
    bitWrite(rx_pairing_status, index, 0);
    memset(&peer[index], 0, sizeof(peer[index]) );

    wifi_state_machine[index].pairing.del = false;  

    Serial.printf("[MSG]WIFI::MODULE::DEL PEER[%d]::Removed from peer list\r\n", index);
}
/***/

void Wifi_Peer_Data_Set(uint8_t channel, uint8_t cmd, uint16_t *tx_data, uint8_t wlen)
{ 
    word_big_endian_t crc16;
    uint16_t len;
    memset(wifi_state_machine[channel].peer.txBuf, 0, sizeof(wifi_state_machine[channel].peer.txBuf));

    len = wlen*2+6; //data 까지의 길이 //serial일 때 -> wlen 9, i/o일때 wlen 1.
    wifi_state_machine[channel].peer.txLen = len+2; //전체 길이

    wifi_state_machine[channel].peer.txBuf[0] = wifi_send.rf_set_group;
    wifi_state_machine[channel].peer.txBuf[1] = channel;
    wifi_state_machine[channel].peer.txBuf[2] = cmd;
    wifi_state_machine[channel].peer.txBuf[3] = wifi_state_machine[channel].peer.device_type;
    wifi_state_machine[channel].peer.txBuf[4] = wifi_state_machine[channel].peer.device_addr;
    wifi_state_machine[channel].peer.txBuf[5] = wifi_state_machine[channel].peer.txLen;

    switch( cmd )
    {
        //데이타 0인 케이스
        case AP_IO_GET:
            memcpy( &wifi_state_machine[channel].peer.txBuf[6], tx_data, wlen*2);
        break;

        //데이타 있을 경우
        default:
            memcpy( &wifi_state_machine[channel].peer.txBuf[6], tx_data, wlen*2);
        break;
    }

    crc16.flag.wd = crc16_modbus(CRC16_MODBUS_INIT_CODE, wifi_state_machine[channel].peer.txBuf, len);
    wifi_state_machine[channel].peer.txBuf[len] = crc16.flag.bf.low;
    wifi_state_machine[channel].peer.txBuf[len+1] = crc16.flag.bf.hi;

    wifi_send.tx_busy[channel] = false;
    wifi_send.rx_busy[channel] = false;
}

void Wifi_Peer_State_Set(WIFI_STATE_MACHINE state, uint8_t channel, uint8_t cmd, uint16_t typeAddr, uint16_t wLen)
{
    word_big_endian_t crc16;
    uint16_t len;
    
    len = 6; //data 까지의 길이

    wifi_state_machine[channel].peer.txBuf[0] = wifi_send.rf_set_group;
    wifi_state_machine[channel].peer.txBuf[1] = channel;
    wifi_state_machine[channel].peer.txBuf[2] = cmd;

    wifi_state_machine[channel].peer.channel = channel;
    
    if( typeAddr )
    {
        wifi_state_machine[channel].peer.device_type = (uint8_t)(typeAddr>>8);
        wifi_state_machine[channel].peer.device_addr = (uint8_t)(typeAddr&0x00ff);

    }else
    {
        //이미 등록되어 있음.
    }
    wifi_state_machine[channel].peer.txBuf[3] = wifi_state_machine[channel].peer.device_type;
    wifi_state_machine[channel].peer.txBuf[4] = wifi_state_machine[channel].peer.device_addr;
    wifi_state_machine[channel].peer.txBuf[5] = 8;

    crc16.flag.wd = crc16_modbus(CRC16_MODBUS_INIT_CODE, wifi_state_machine[channel].peer.txBuf, 6);
    wifi_state_machine[channel].peer.txBuf[len] = crc16.flag.bf.low;
    wifi_state_machine[channel].peer.txBuf[len+1] = crc16.flag.bf.hi;

    wifi_state_machine[channel].peer.txLen = 8;

    wifi_state_machine[channel].peer.send_cmd = cmd;

    switch(state)
    {
        case WIFI_STATE_PAIRING_ADD:
            wifi_state_machine[channel].pairing.request = true;
            wifi_state_machine[channel].pairing.add = true;
            wifi_state_machine[channel].peer.pMac = broadcast_addr;
            
            Serial.printf("[MSG]WIFI::STATE SET::PAIRING ADD::PEER[%d] STATE=%d\r\n",channel,wifi_state_machine[channel].peer.state);
        break;

        case WIFI_STATE_PAIRING_DEL:
            wifi_state_machine[channel].pairing.request = true;
            wifi_state_machine[channel].pairing.del = true;  
            wifi_state_machine[channel].peer.pMac = peer[channel].mac;     

            Serial.printf("[MSG]WIFI::STATE SET::PAIRING DEL::PEER[%d] STATE=%d\r\n",channel,wifi_state_machine[channel].peer.state); 
        break;

        default:
        break;
    }
}

#if 1
//! TEST POINT 
void Wifi_Peer_Rotation(void)
{
    #if 1 //fast scan
    //페어드와 페어링 요청이 없는 peer는 pass
    unsigned char i=0;
    
    for( i=0 ; i<MAX_PEER ; i++)
    {
        if( ++wifi_send.peer_addr>=MAX_PEER ) wifi_send.peer_addr=0;

        if( peer[wifi_send.peer_addr].pairFlag || wifi_state_machine[wifi_send.peer_addr].pairing.request )
        {
            break;
        }
    }
    #else   //same time division rotation
    static unsigned char i=0;
    if( ++wifi_send.peer_addr>=MAX_PEER ) wifi_send.peer_addr=0;

    #endif 
}
#endif 



#endif