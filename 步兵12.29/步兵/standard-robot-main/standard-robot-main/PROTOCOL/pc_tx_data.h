#ifndef _pc_tx_data_H
#define _pc_tx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_TX_FIFO_BUFLEN 500

typedef enum
{
	TRACK_AMOR_MODE			=0,
	BIG_BUFF_MODE				=1,
	NORMAL_CTRL_MODE		=2,
	SMALL_BUFF_MODE			=3,
}mode_t;

typedef enum
{
	red  = 0,
	blue = 1,
}enemy_color_t;

/**
 * @brief  鐢垫帶->PC鐨勬暟鎹robot_tx_data (13瀛楄妭) 锛屾暟鎹抚闀匡細5 + 13 + 2 = 20 (瀛楄妭)
 */
#pragma pack(1)
typedef struct
{
  uint8_t robot_color : 1;  // 棰滆壊 (0/1)
  uint8_t task_mode : 1;    // 璇嗗埆妯″紡  锛�0 鑷瀯 / 1 澶у皬绗︼級
  uint8_t visual_valid : 1; // 瑙嗚鏈夋晥浣� (0/1)
  uint8_t direction : 2;    // 鎷撳睍瑁呯敳鏉挎爣蹇椾綅 (0-3)
  uint8_t bullet_level : 3; // 弹丸发射速度			寮归€熺瓑绾� 1 2 3绾�
  float robot_pitch;        // 娆ф媺瑙�(搴�)
  float robot_yaw;          // 娆ф媺瑙�(搴�)
  float time_stamp;         // 鐢垫帶鏃堕棿鎴�(ms)
} robot_tx_data;
#pragma pack()

extern fifo_s_t  pc_txdata_fifo;
extern robot_tx_data pc_send_mesg;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);


#endif
