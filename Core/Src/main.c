/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define PI 3.141592653589793

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//使用双缓冲
uint8_t imuRecvBuff1[64] = {0};
int imuRecvOffset1 = 0;
uint8_t imuRecvBuff2[64] = {0};
int imuRecvOffset2 = 0;
int imuRecvStatus = 0;
uint8_t* imuPackage = 0; //等待处理的数据包
uint8_t imuBuff[1024] = {0};
uint8_t imuBuffOffset = 0;

static const uint8_t CRC8Table[] = {
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};

static const uint16_t CRC16Table[256] = {
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7, 0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF, 0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6, 0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE, 0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485, 0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D, 0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4, 0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC, 0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823, 0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B, 0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41, 0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49, 0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70, 0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78, 0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F, 0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067, 0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E, 0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256, 0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D, 0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C, 0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634, 0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB, 0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3, 0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A, 0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92, 0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9, 0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1, 0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8, 0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint8_t CRC8_Table(uint8_t* p, uint8_t counter) {
    uint8_t crc8 = 0;
    for (int i = 0; i < counter; i++) {
        uint8_t value = p[i];
        uint8_t new_index = crc8 ^ value; crc8 = CRC8Table[new_index];
    }
    return (crc8);
}

uint16_t CRC16_Table(uint8_t* p, uint8_t counter) {
    uint16_t crc16 = 0;
    for (int i = 0; i < counter; i++) {
        uint8_t value = p[i];
        crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8); }
    return (crc16);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
    if (huart->Instance == USART2) {
        if (imuRecvStatus == 0) {
            imuRecvOffset1 += 1;
            if (imuRecvOffset1 == 64) {
                imuRecvStatus = 1;
                imuRecvOffset1 = 0;
                imuPackage = imuRecvBuff1;
                HAL_UART_Receive_IT(&huart2, imuRecvBuff2 + imuRecvOffset2, 1);
            } else {
                HAL_UART_Receive_IT(&huart2, imuRecvBuff1 + imuRecvOffset1, 1);
            }
        } else {
            imuRecvOffset2 += 1;
            if (imuRecvOffset2 == 64) {
                imuRecvStatus = 0;
                imuRecvOffset2 = 0;
                imuPackage = imuRecvBuff2;
                HAL_UART_Receive_IT(&huart2, imuRecvBuff1 + imuRecvOffset1, 1);
            } else {
                HAL_UART_Receive_IT(&huart2, imuRecvBuff2 + imuRecvOffset2, 1);
            }
        }
    }
}


/*
 * 默认机器人的前方相对坐标系x轴，左方为相对坐标系y轴
 * 启动时，默认前方为绝对坐标系x轴，左方为绝对坐标系y轴
 */

volatile struct WheelPWM
{
    double speed; //当前速度
    double target; //目标速度
    double minPWM; //限制pwm输出范围
    double maxPWM;
    double kp;
    double ki;
    double kd;
    double error;
    double lastError;
    double lastError2;
    double pwm; //pwm输出值
    double pulse; //脉冲数
}wheelPWM1, wheelPWM2;

volatile struct RobotInfo
{
    double VL; //左轮速度
    double VR; //右轮速度
    double VM; //平均速度
    double r; //运行半径
    double L; //车轮间距
    double XPos; //绝对坐标
    double YPos;
    double radian; //方向, 逆时针, 起点x轴
}robotInfo;

volatile struct RobotIMU
{
    double roll;
    double pitch;
    double heading;
    double x; //向量x坐标
    double y; //向量y坐标
    double len; //向量长度
}robotIMU;

void ClearSpeed()
{
    robotInfo.VL = 0;
    robotInfo.VR = 0;
}

void CommitSpeed()
{
    wheelPWM2.target = robotInfo.VL;
    wheelPWM1.target = -robotInfo.VR;

}

void Stop()
{
    ClearSpeed();
    CommitSpeed();
}


void SpinLeft(double speed)
{
    robotInfo.VL -= speed;
    robotInfo.VR += speed;
}

void SpinRight(double speed)
{
    robotInfo.VL += speed;
    robotInfo.VR -= speed;
}

void MoveForwardV1(double speed)
{
    robotInfo.VL += speed;
    robotInfo.VR += speed;
}


/*
 * @param speed 运行速度 r 运行半径
 */
void MoveForward(double speed, double r)
{
    robotInfo.VM = speed;
    robotInfo.r = r;
    robotInfo.VL = (1 - robotInfo.L / (2 * robotInfo.r)) * robotInfo.VM;
    robotInfo.VR = (1 + robotInfo.L / (2 * robotInfo.r)) * robotInfo.VM;
}

/*
 * @param speed 运行速度 r 运行半径 dis 运行距离
 */
void MoveForwardV2(double speed, double r, double dis)
{
    ClearSpeed();
    MoveForward(speed, r);
    CommitSpeed();
    wheelPWM1.pulse = 0;
    wheelPWM2.pulse = 0;
    while (1) {
        double meanDis = (fabs(wheelPWM1.pulse) + fabs(wheelPWM2.pulse)) / 2;
        if (fabs(meanDis) > dis) break;
    }
    Stop();
}


void RobotInit()
{
    robotInfo.L = 13.5;

    wheelPWM1.target = 0;
    wheelPWM1.minPWM = -1000;
    wheelPWM1.maxPWM = 1000;
    wheelPWM1.kp = 960;
    wheelPWM1.ki = 240;
    wheelPWM1.kd = 0;

    wheelPWM2.target = 0;
    wheelPWM2.minPWM = -1000;
    wheelPWM2.maxPWM = 1000;
    wheelPWM2.kp = 960;
    wheelPWM2.ki = 240;
    wheelPWM2.kd = 0;
}

void Do_PID(struct WheelPWM* wheelPWM)
{
    wheelPWM->lastError2 = wheelPWM->lastError;
    wheelPWM->lastError = wheelPWM->error;
    wheelPWM->error = wheelPWM->target - wheelPWM->speed;

    wheelPWM->pwm += wheelPWM->kp * (wheelPWM->error - wheelPWM->lastError) + wheelPWM->ki * wheelPWM->error + wheelPWM->kd * (wheelPWM->error - 2 * wheelPWM->lastError + wheelPWM->lastError2);
    if (wheelPWM->pwm > wheelPWM->maxPWM) wheelPWM->pwm = wheelPWM->maxPWM;
    if (wheelPWM->pwm < wheelPWM->minPWM) wheelPWM->pwm = wheelPWM->minPWM;
}

void PID_Tick()
{
    wheelPWM1.speed = (short) __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    wheelPWM1.pulse += wheelPWM1.speed;

    wheelPWM2.speed = (short) __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    wheelPWM2.pulse += wheelPWM2.speed;

    Do_PID(&wheelPWM1);
    if (wheelPWM1.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wheelPWM1.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else if (wheelPWM1.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -wheelPWM1.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    }


    Do_PID(&wheelPWM2);
    if (wheelPWM2.pwm > 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wheelPWM2.pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    } else if (wheelPWM2.pwm < 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -wheelPWM2.pwm);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim->Instance == TIM6) {
        PID_Tick();
    }
}


struct MSG_IMU
{
    float Gyroscope_X; //机体系X轴角速度
    float Gyroscope_Y; //机体系Y轴角速度
    float Gyroscope_Z; //机体系Z轴角速度
    float Accelerometer_X; //机体系X轴加速度(未分离重力加速度)
    float Accelerometer_Y; //机体系Y轴加速度(未分离重力加速度)
    float Accelerometer_Z; //机体系Z轴加速度(未分离重力加速度)
    float Magnetometer_X; //机体系X轴磁感应强度
    float Magnetometer_Y; //机体系Y轴磁感应强度
    float Magnetometer_Z; //机体系Z轴磁感应强度
    float IMU_Temperature; //如果IMU数据由多个传感器组成则该值为这些传感器的平均温度
    float Pressure; //气压值
    float Pressure_Temperature; //气压计的温度值
    int64_t Timestamp; //时间戳
};

struct MSG_AHRS
{
    float RollSpeed; //横滚角速度
    float PitchSpeed; //俯仰角速度
    float HeadingSpeed; //偏航角速度
    float Roll; //横滚
    float Pitch; //俯仰
    float Heading; //偏航
    float Q1; //四元数Q1
    float Q2; //四元数Q2
    float Q3; //四元数Q3
    float Q4; //四元数Q4
    int64_t Timestamp; //时间戳
};


void HandleIMUPackage(int cmd, uint8_t* data, int len)
{
    if (cmd == 0x40) {
        struct MSG_IMU* msgImu = (struct MSG_IMU*)data;
    } else if (cmd == 0x41) {
        struct MSG_AHRS* msgAhrs = (struct MSG_AHRS*)data;

        robotIMU.roll = -msgAhrs->Roll;
        robotIMU.pitch = -msgAhrs->Pitch;
        robotIMU.heading = msgAhrs->Heading;
        robotIMU.y = sin(robotIMU.roll);
        robotIMU.x = cos(robotIMU.roll) * sin(robotIMU.pitch);

        //缩放
        robotIMU.x *= -100;
        robotIMU.y *= -100;
        robotIMU.len = sqrt(pow(robotIMU.x, 2) + pow(robotIMU.y, 2));


//        char buff[64] = {0};
//        snprintf(buff, 64, "roll:%.4f pitch:%.4f heading:%.4f\r\n", robotIMU.roll, robotIMU.pitch, robotIMU.heading);
//        HAL_UART_Transmit(&huart1, buff, strlen(buff), 100);

//        char buff[64] = {0};
//        snprintf(buff, 64, "x:%.4lf y:%.4lf len:%.4lf\r\n", robotIMU.x, robotIMU.y, robotIMU.len);
//        HAL_UART_Transmit(&huart1, buff, strlen(buff), 100);

    }
}

void DecodeIMUPackage()
{
    if (imuPackage) {
        memcpy(imuBuff + imuBuffOffset, imuPackage, 64);
        imuBuffOffset += 64;
        if (imuBuffOffset >= 1024) imuBuffOffset = 0;
        imuPackage = 0;
    }
    //定位帧头
    int packageStart = 0;
    for (; packageStart < imuBuffOffset - 4; ++packageStart) {
        uint8_t crc8 = CRC8_Table(imuBuff + packageStart, 4);
        if (imuBuff[packageStart] == 0xFC && crc8 == imuBuff[packageStart + 4]) {
            break;
        }
    }
    //将帧头移动到起始位置
    memcpy(imuBuff, imuBuff + packageStart, imuBuffOffset - packageStart);
    imuBuffOffset -= packageStart;

    //判断帧头
    uint8_t crc8 = CRC8_Table(imuBuff, 4);
    if (imuBuff[0] == 0xFC && crc8 == imuBuff[4]) { //是帧头
        int cmd = imuBuff[1]; //指令类别
        int dataLen = imuBuff[2]; //数据长度

        //判断是否接收完整个数据
        if (imuBuffOffset >= 5 + 2 + dataLen + 1) { //5->帧头 2->crc16 1->帧尾标记
            uint16_t crc16FromBuff = (imuBuff[5] << 8) | imuBuff[6];
            uint16_t crc16 = CRC16_Table(imuBuff + 7, dataLen);

            if (crc16 == crc16FromBuff) HandleIMUPackage(cmd, imuBuff + 7, dataLen);

            int frameLen = 5 + 2 + dataLen + 1;
            memcpy(imuBuff, imuBuff + frameLen, imuBuffOffset - frameLen);
            imuBuffOffset -= frameLen;
        }
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    HAL_TIM_Base_Start_IT(&htim6);

    HAL_UART_Receive_IT(&huart2, imuRecvBuff1, 1);

    RobotInit();

    Stop();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      DecodeIMUPackage();

      if (robotIMU.len > 5) {
          double rad = 0;
          if (robotIMU.x > 0 && robotIMU.y > 0) {
              rad = atan(robotIMU.y / robotIMU.x);
          } else if (robotIMU.x < 0 && robotIMU.y > 0) {
              rad = atan(robotIMU.y / robotIMU.x) + PI;
          } else if (robotIMU.x < 0 && robotIMU.y < 0) {
              rad = atan(robotIMU.y / robotIMU.x) - PI;
          } else {
              rad = atan(robotIMU.y / robotIMU.x);
          }

          if (rad > 0) {
              ClearSpeed();
              MoveForwardV1(150);
              double spinSpeed = 90 * rad;
              if (spinSpeed > 80) spinSpeed = 80;
              SpinLeft(spinSpeed);
              CommitSpeed();
          } else if (rad < 0) {
              ClearSpeed();
              MoveForwardV1(150);
              double spinSpeed = -90 * rad;
              if (spinSpeed > 80) spinSpeed = 80;
              SpinRight(spinSpeed);
              CommitSpeed();
          }
      } else {
          Stop();
      }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

