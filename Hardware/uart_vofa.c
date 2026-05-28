/*******************************************************************************
 * @file        uart_vofa.c
 * @brief       VOFA+ 串口数据收发模块
 *              实现了与VOFA+上位机软件的数据通信功能，包括发送数据用于波形显示
 *              和接收上位机下发的参数调节指令
 * @author      eternal_fu
 * @version     V0.0.1
 * @date        2024-12-11 01:05:50
 * @lastEditTime 2024-12-11 04:28:17
 *******************************************************************************/
#include "uart_vofa.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // 包含字符串操作函数的头文件


/* VOFA 数据缓冲区，用于存储接收的数据包 */
uint8_t g_ucVofaBuf[VOFA_DATAPACK_MAXLEN];

/* 私有函数声明 */
static float vofa_get_data(uint8_t, uint8_t);
static void vofa_set_sram_data(uint8_t, float);
static void vofa_set_flash_data(float);

/*******************************************************************************
 * @brief   向VOFA+上位机发送数据，用于图形化显示
 * @param   _dataflag 当前发送的数据类型标志位，用于选择不同的数据显示模式
 * @return  none
 * @note    使用FireWater协议格式："<prefix>:ch0,ch1,ch2,...,chN\n"
 *          支持多种数据类型的发送，通过_dataflag参数选择不同的数据显示模式
 *******************************************************************************/
void vofa_draw_graphical(uint8_t _dataflag)
{
    /*
    FireWater协议格式:
        "<any>:ch0,ch1,ch2,...,chN\n"
        - any可以为空，前缀和(\n)可以省略
        - any也可以为"image"前缀，用于发送图片数据
        - 此处\n为换行，即字符+字母n组合
        - \n也可以是\n\r或\r\n
    示例:
        VOFA_printf("%.0f,"		
					"%.0f,"
					"%.4f,"
					"%.4f,"
                    "%.4f\r\n"
					,dis_pid.target, dis_pid.current, dis_pid.kp, dis_pid.kd,  dis_pid.out);
        发送4个通道数据:
        VOFA_printf("channels: 1.386578,0.977929,-0.628913,-0.942729\n");
        VOFA_printf("1.386578,0.977929,-0.628913,-0.942729\n")
    */

	/* 通过标志位选择打印指定的数据 */
    switch(_dataflag)
	{
        /* 左电机PID - 速度环 */
		case PID_LEFT_MOTOR:
            VOFA_printf("%.2f,%.2f,%.2f,%.2f\r\n", 
						speed_left.Actual, speed_left.Target, speed_left.Kp, speed_left.Ki);
			break;
		// /* 角度PID */
		// case PID_ANGLE:
        //     VOFA_printf("%.2f,%.2f,%.2f,%.2f\r\n",
		// 				speed_left.Actual, speed_left.target, speed_left.Kp, speed_left.Kd);
		// 	break;
		/* 右电机PID - 速度环 */
		case PID_RIGHT_MOTOR:
            VOFA_printf("%.2f,%.2f,%.2f,%.2f\r\n", speed_right.Actual, speed_right.Target, speed_right.Kp, speed_right.Ki);
			break;
        default:
            break;
	}

}

/*******************************************************************************
 * @brief   通过串口中断接收来自VOFA+的数据包，并根据数据包执行相应操作
 * @param   _rx_byte 接收到的字节数据
 * @return  none
 * @note    目前实现了VOFA数据接收的解析功能
 *******************************************************************************/
void vofa_set_data(uint8_t _rx_byte)
{
    static uint8_t end_pos = 0;         // 记录vofa数据包接收的末尾位置，标记当前指针
    static uint8_t head_pos = 0;
    uint8_t frame_done = 0;

    /* 添加到缓冲区 */
    g_ucVofaBuf[end_pos] = _rx_byte;

    if (_rx_byte == VOFA_DATAPACK_HEAD) {   /* 找到帧头 */
        head_pos = end_pos;                 /* 记录当前位置为帧头 */
    }
    else if(_rx_byte == VOFA_DATAPACK_END && g_ucVofaBuf[head_pos] == VOFA_DATAPACK_HEAD) { /* 找到帧尾且有帧头时 */
        /* 解析VOFA数据 */
        float VofaData = vofa_get_data(head_pos, end_pos);
        
        /* 根据数据，写存储器SRAM变量 */
        vofa_set_sram_data(head_pos, VofaData);

        /* 临时调试回显：每收到完整一帧都打印当前启动状态 */
       // VOFA_printf("dbg car_started=%d\r\n", (int)car_started);

        /* 清空缓冲区 */
        end_pos = 0;
        head_pos = 0;
        frame_done = 1;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }

    /* 如果缓冲区大小超过最大值则清空 */
    if(frame_done == 0) {
        end_pos++;
    }

    if(end_pos >= VOFA_DATAPACK_MAXLEN) {
        end_pos = 0;
        memset(g_ucVofaBuf, 0x00, VOFA_DATAPACK_MAXLEN);
    }
}

/*******************************************************************************
 * @brief   从缓冲区中解析出数据值
 * @param   _head 帧头位置，_end 帧尾位置，数据在_head+1开始
 * @return  解析得到的浮点数值
 * @note    需要包含头文件 stdlib.h
 *          - atof() 字符串转为浮点数
 *          - atoi() 字符串转为整数
 *******************************************************************************/
static float vofa_get_data(uint8_t _head, uint8_t _end)
{
    uint8_t len = (uint8_t)(_end - (_head + 1));
    char _VofaData[VOFA_DATAPACK_MAXLEN] = {0};

	for(uint8_t i = 0; i < len; i++) {
		_VofaData[i] = (char)g_ucVofaBuf[i + _head + 1];
	}

    _VofaData[len] = '\0';
    return atof(_VofaData);	/* 字符串转浮点数 */
}

/*******************************************************************************
 * @brief   将接收到的数据，直接写入SRAM空间，可能丢失
 *          当前先把接收到的值，存放到需要的变量中
 * @param   _head 帧头位置，包含数据标识符信息
 * @param   _data 接收到的数据值
 * @return  none
 * @note    数据格式如下所示：
 *          [数据标识符(用于分辨下发的是什么数据,1-2byte)] [帧头('=',1byte)] [数据位(n byte)] [帧尾('\n',1byte)]
 *          
 *          例如:
 *              "P1=%.2f\n"
 *              "V1=%.2f\n"
 *          
 *          P1/V1是数据标识符，根据标识符分别赋值给对应变量
 *          例如：
 *              P1 - g_tAnglePID.kp
 *              P2 - g_tLeftMotorPID.kp
 *              I1 - g_tAnglePID.ki
 *              I2=%.2f\n
 *******************************************************************************/
static void vofa_set_sram_data(uint8_t _head, float _data)
{
    uint8_t _id_pos1 = _head - 2;   /* 数据标识符第1位 - P/I/... */
    uint8_t _id_pos2 = _head - 1;   /* 数据标识符第2位 - 1/2/3/4/...*/
	
    /* 角度环[1] PID参数设置 ********************************************************/
    /* P1 - speed_left.Kp */
	if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '1') { 
        speed_left.Kp = _data;
    }
    /* I1 - speed_left.Ki */
    else if (g_ucVofaBuf[_id_pos1] == 'I' && g_ucVofaBuf[_id_pos2] == '1') {
        speed_left.Ki = _data;
    }
	/* T1 - speed_left.target */
    else if (g_ucVofaBuf[_id_pos1] == 'T' && g_ucVofaBuf[_id_pos2] == '1') {
        speed_left.Target = (int)_data;
    }
    /* 右电机速度环[2] PID参数设置 *************************************************/
    /* P2 - speed_right.Kp */
	else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '2') { 
        speed_right.Kp = _data;
    }
    /* I2 - speed_right.Ki */
    else if (g_ucVofaBuf[_id_pos1] == 'I' && g_ucVofaBuf[_id_pos2] == '2') {
        speed_right.Ki = _data;
    }
	/* T2 - speed_right.target */
    else if (g_ucVofaBuf[_id_pos1] == 'T' && g_ucVofaBuf[_id_pos2] == '2') {
        speed_right.Target = _data;
    }

    /* 其他参数设置 */
    /* 注意：如果是整型参数-需要强制转换(int) */ 
	else if (g_ucVofaBuf[_id_pos1] == 'S' && g_ucVofaBuf[_id_pos2] == '1') {
        car_started = (int)_data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '3') {
        Turn.Kp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'D' && g_ucVofaBuf[_id_pos2] == '3') {
        Turn.Kd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '4') {
        turn_speed_left.Kp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'I' && g_ucVofaBuf[_id_pos2] == '4') {
        turn_speed_left.Ki = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '5') {
        turn_speed_right.Kp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'I' && g_ucVofaBuf[_id_pos2] == '5') {
        turn_speed_right.Ki = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '6') {
        Straight.Kp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'D' && g_ucVofaBuf[_id_pos2] == '6') {
        Straight.Kd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'B' && g_ucVofaBuf[_id_pos2] == '1') {
        Basic_Speed= (int)_data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'P' && g_ucVofaBuf[_id_pos2] == '7') {
        Xunji.Kp = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'D' && g_ucVofaBuf[_id_pos2] == '7') {
        Xunji.Kd = _data;
    }
    else if (g_ucVofaBuf[_id_pos1] == 'M' && g_ucVofaBuf[_id_pos2] == '1') {
        task_mode = (uint8_t)_data;
    }

}

/*******************************************************************************
 * @brief   将接收到的数据，直接写入FLASH空间，不会丢失
 *          当前FLASH空间已预留参数空间，实际内部没有FLASH API函数
 * @param   _data 接收到的数据值
 * @return  none
 *******************************************************************************/
static void vofa_set_flash_data(float _data)
{
    
}
