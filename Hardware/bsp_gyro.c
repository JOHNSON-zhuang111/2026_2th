/*
 * 立创开发板软硬件资料与相关扩展板软硬件资料官网全部开源
 * 开发板官网：www.lckfb.com
 * 文档网站：wiki.lckfb.com
 * 技术支持常驻论坛，任何技术问题欢迎随时交流学习
 * 嘉立创社区问答：https://www.jlc-bbs.com/lckfb
 * 关注bilibili账号：【立创开发板】，掌握我们的最新动态！
 * 不靠卖板赚钱，以培养中国工程师为己任
 */

#include "bsp_gyro.h"
#include "stdio.h"
#include "string.h"
#include "ti_msp_dl_config.h" // 增加 SysConfig 生成的头文件以使用硬件I2C

static Gyro_Struct Gyro_Structure;

void jy61pInit(void)
{

	/*================Z轴归零==================*/

	// 寄存器解锁
	uint8_t unlock_reg1[2] = {0x88,0xB5};
	writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg1,2);
	delay_ms(200);
	// Z轴归零
	uint8_t z_axis_reg[2] = {0x04,0x00};
	writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,z_axis_reg,2);
	delay_ms(200);
	// 保存
	uint8_t save_reg1[2] = {0x00,0x00};
	writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg1,2);
	delay_ms(200);

	/*================角度归零==================*/

	// 寄存器解锁
	uint8_t unlock_reg[2] = {0x88,0xB5};
	writeDataJy61p(IIC_ADDR,UN_REG,unlock_reg,2);
	delay_ms(200);
	// 角度归零
	uint8_t angle_reg[2] = {0x08,0x00};
	writeDataJy61p(IIC_ADDR,ANGLE_REFER_REG,angle_reg,2);
	delay_ms(200);
	// 保存
	uint8_t save_reg[2] = {0x00,0x00};
	writeDataJy61p(IIC_ADDR,SAVE_REG,save_reg,2);
	delay_ms(200);

	// 清空结构体
	memset((void *)&Gyro_Structure,0,sizeof(Gyro_Structure));
}

/******************************************************************
 * 函 数 名 称：writeDataJy61p
 * 函 数 说 明：写数据(硬件I2C)
 * 函 数 形 参：dev 设备地址
				reg 寄存器地址
				data 数据首地址
				length 数据长度
 * 函 数 返 回：返回0则写入成功
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
uint8_t writeDataJy61p(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
    uint8_t txBuffer[16]; 
    txBuffer[0] = reg;
    for (uint32_t i = 0; i < length; i++) {
        txBuffer[i + 1] = data[i];
    }
    
    uint32_t timeout;
    
    // 等待总线空闲
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 0;
    }
    
    DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);
    DL_I2C_fillControllerTXFIFO(I2C_GYRO_INST, txBuffer, length + 1);
    
    DL_I2C_startControllerTransfer(I2C_GYRO_INST, dev, DL_I2C_CONTROLLER_DIRECTION_TX, length + 1);

    // 轮询直到发送完毕（BUSY变为0）
    timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) return 0;
    }
           
    if (DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return 0; // 失败
    }

    return 1; // 成功
}

/******************************************************************
 * 函 数 名 称：readDataJy61p
 * 函 数 说 明：读数据(硬件I2C)
 * 函 数 形 参：dev 设备地址
				reg 寄存器地址
				data 数据存储地址
				length 数据长度
 * 函 数 返 回：返回0则写入成功
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
uint8_t readDataJy61p(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
    uint32_t timeout;
    
    // 1. 等待总线空闲
    timeout = 100000;
    while (!(DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (--timeout == 0) return 0;
    }

    DL_I2C_flushControllerTXFIFO(I2C_GYRO_INST);
    DL_I2C_flushControllerRXFIFO(I2C_GYRO_INST);

    // 2. 发送寄存器地址 (带START，*不带*STOP -> 准备 Repeated Start)
    DL_I2C_fillControllerTXFIFO(I2C_GYRO_INST, &reg, 1);
    DL_I2C_startControllerTransferAdvanced(I2C_GYRO_INST, dev, 
        DL_I2C_CONTROLLER_DIRECTION_TX, 1, 
        DL_I2C_CONTROLLER_START_ENABLE, 
        DL_I2C_CONTROLLER_STOP_DISABLE,   // <== 关键：不发STOP
        DL_I2C_CONTROLLER_ACK_DISABLE);

    // 轮询直到发送完毕（BUSY变为0）
    timeout = 100000;
    while (DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_BUSY) {
        if (--timeout == 0) return 0;
    }
    
    if (DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
        return 0; // 失败
    }
    
    // 3. 发起读传输 (此时触发 Repeated Start，并且最后带STOP)
    DL_I2C_startControllerTransferAdvanced(I2C_GYRO_INST, dev, 
        DL_I2C_CONTROLLER_DIRECTION_RX, length, 
        DL_I2C_CONTROLLER_START_ENABLE, 
        DL_I2C_CONTROLLER_STOP_ENABLE, 
        DL_I2C_CONTROLLER_ACK_DISABLE);
                                   
    // 4. 从RX FIFO逐字节读出数据
    for (uint32_t i = 0; i < length; i++) {
        // 等待FIFO收到数据
        timeout = 100000;
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_GYRO_INST)) {
            if (--timeout == 0) return 0;
            if (DL_I2C_getControllerStatus(I2C_GYRO_INST) & DL_I2C_CONTROLLER_STATUS_ERROR) {
                return 0;
            }
        }
        data[i] = DL_I2C_receiveControllerData(I2C_GYRO_INST);
    }
    
    return 1; // 成功
}

/******************************************************************
 * 函 数 名 称：get_angle
 * 函 数 说 明：读角度数据
 * 函 数 形 参：无
 * 函 数 返 回：返回结构体
 * 作       者：LCKFB
 * 备       注：无
******************************************************************/
/**
 * @brief 获取陀螺仪角度数据
 * @return 指向Gyro_Struct结构体的指针，包含X、Y、Z三个轴的角度数据
 * @note 从JY61P陀螺仪传感器读取角度数据，并进行处理和转换
 */
Gyro_Struct *get_angle(void)
{
	// 数据缓存，用于存储从陀螺仪读取的原始数据
	uint8_t sda_angle[6] = {0};
	int ret = 0; // 读取操作的返回值
	uint8_t retry = 5; // 最大重试次数

	while(retry--)
	{
		memset((void *)sda_angle, 0, sizeof(sda_angle));
		
		//__disable_irq(); // 【关键】临时关闭所有中断，保护软I2C时序绝对不被打断
		ret = readDataJy61p(IIC_ADDR, 0x3D, sda_angle, 6);
		//__enable_irq();  // 【关键】读取完毕，火速恢复中断

		// 如果读取成功，且数据不全为0(全0通常代表总线卡死/错位)，则跳出重试
		if(ret != 0 && (sda_angle[0]!=0 || sda_angle[1]!=0 || sda_angle[2]!=0))
		{
			break;
		}
		delay_us(500); // 如果失败，稍等片刻后重读
	}

	if(ret == 0)
	{
		// 5次都失败了，只能保命返回上一次的旧数据，防止突变
		return &Gyro_Structure;
	}

	#if GYRO_DEBUG
	// 调试信息：打印原始寄存器数据
	lc_printf("RollL = %x\r\n", sda_angle[0]);  // 横滚角低字节
	lc_printf("RollH = %x\r\n", sda_angle[1]);  // 横滚角高字节
	lc_printf("PitchL = %x\r\n", sda_angle[2]); // 俯仰角低字节
	lc_printf("PitchH = %x\r\n", sda_angle[3]); // 俯仰角高字节
	lc_printf("YawL = %x\r\n", sda_angle[4]);   // 偏航角低字节
	lc_printf("YawH = %x\r\n", sda_angle[5]);   // 偏航角高字节
	#endif

    // 计算RollX（横滚角），将16位数据转换为角度值
    // 公式：角度值 = (高字节<<8 | 低字节) / 32768.0 * 180.0
    float RollX = (float)(((sda_angle[1] << 8) | sda_angle[0]) / 32768.0 * 180.0);
    // 将角度值限制在-180到180度范围内
    if (RollX > 180.0)
    {
        RollX -= 360.0;
    }
    else if (RollX < -180.0)
    {
        RollX += 360.0;
    }

    // 计算PitchY（俯仰角）
    float PitchY = (float)(((sda_angle[3] << 8) | sda_angle[2]) / 32768.0 * 180.0);
    // 将角度值限制在-180到180度范围内
    if (PitchY > 180.0)
    {
        PitchY -= 360.0;
    }
    else if (PitchY < -180.0)
    {
        PitchY += 360.0;
    }

    // 计算YawZ（偏航角）
    float YawZ = (float)(((sda_angle[5] << 8) | sda_angle[4]) / 32768.0 * 180.0);
    // 将角度值限制在-180到180度范围内
    if (YawZ > 180.0)
    {
        YawZ -= 360.0;
    }
    else if (YawZ < -180.0)
    {
        YawZ += 360.0;
    }

    // 将计算结果保存到全局结构体Gyro_Structure中
    Gyro_Structure.x = RollX;  // 横滚角（X轴）
    Gyro_Structure.y = PitchY; // 俯仰角（Y轴）
    Gyro_Structure.z = YawZ;   // 偏航角（Z轴）

	// 返回包含角度数据的结构体指针
	return &Gyro_Structure;
}