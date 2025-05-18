#ifndef __USERTASK_H__
#define __USERTASK_H__

#include "ti_msp_dl_config.h"
#include "../USER/UART/UART.h"
#include "../USER/BTN/BTN.h"
#include "../USER/TICK/Tick.h"
#include "../USER/GUI/GUI.h"
#include <string.h>
#include <stdlib.h>

#define RX_BUFFER_SIZE 30
#define ADC_NUM     5       //测量输出电阻时，开路和负载采样的点数  
#define SIGNAL_IN_mVPP  62.0f
#define RESISTOR_BING   10000.0f    //与输入串联电阻并联的电阻
#define RESISTOR_IN    3300.0f     //信号源与放大器之间串联的电阻大小
#define RESISTOR_LOAD  2000.0f     //放大器负载电路大小

#define MAX_POINTS  1000      //扫频点数
#define GAIN_MAX 200.0f     //最大增益
#define FREQ_START 100   //起始频率
#define FREQ_END 1000000  //终止频率

//电平容忍度
#define BEAR_LEVEL 0.10f

#define R1_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.04f
#define R2_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.12f
#define R3_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.03f
#define R4_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.01f
#define R1_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.75f
#define R2_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.30f
#define R3_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.23f
#define R4_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.72f

#define C1_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.10f
#define C2_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.83f
//#define C3_BROKEN_AD8310OUT_ELECTRICAL_LEVEL 1.04f
#define C1_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.78f
#define C2_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.73f
//#define C3_BROKEN_AD8310IN_ELECTRICAL_LEVEL 1.04f

#define R1_SHORT_AD8310OUT_ELECTRICAL_LEVEL 2.82f
#define R2_SHORT_AD8310OUT_ELECTRICAL_LEVEL 1.05f
#define R3_SHORT_AD8310OUT_ELECTRICAL_LEVEL 1.37f
#define R4_SHORT_AD8310OUT_ELECTRICAL_LEVEL 1.10f
#define R1_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.20f
#define R2_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.35f
#define R3_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.62f
#define R4_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.36f

#define C1_SHORT_AD8310OUT_ELECTRICAL_LEVEL 2.18f
#define C2_SHORT_AD8310OUT_ELECTRICAL_LEVEL 1.10f
#define C3_SHORT_AD8310OUT_ELECTRICAL_LEVEL 1.01f
#define C1_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.72f
#define C2_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.28f
#define C3_SHORT_AD8310IN_ELECTRICAL_LEVEL 1.23f





enum {
    InputImp = 1,   //输入阻抗
    OutputImp = 2,  //输出阻抗
    Gain = 3,       //增益
    AmpFreq = 4,    //幅频曲线
    ErrGet = 5,     //测量错误
};
//测量输出电阻时的状态
enum {
    Open = 1,       //负载开路
    Load = 2,       //负载接入
};
//非阻塞任务函数状态变量
typedef enum {
    IDLE,
    WAIT,
    PROCESS
}TaskState_t;
//绘制增益曲线任务状态变量
typedef enum {
    WAIT_COMMAND,
    START_ADC,
    WAIT_ADC,
    DRAW_POINT
}PlotState_t;

typedef struct Circuit_Paramter {
    float Ri;   //输入电阻
    float Ro;   //输出电阻
    float Av;   //电压增益
    float fh;   //3dB上截止频率
}Cir_param_t;

//定义错误码
typedef enum {
    NORMAL,
    R1_BROKEN,
    R2_BROKEN,
    R3_BROKEN,
    R4_BROKEN,
    C1_BROKEN,
    C2_BROKEN,
    R1_SHORT,
    R2_SHORT,
    R3_SHORT,
    R4_SHORT,
    C1_SHORT,
    C2_SHORT,
    C3_SHORT

}Err_code_t;


extern volatile uint8_t TaskMark;
extern BTNData_t BTNData;

extern volatile bool uart_rx_finish;
extern volatile char uart_rx_buffer[RX_BUFFER_SIZE];
extern volatile uint8_t uart_rx_index;
extern volatile bool gCheckADC;

extern Cir_param_t c_param;
extern TaskState_t InputImpState;
extern TaskState_t OutputImpState;
extern TaskState_t GainState;
extern TaskState_t ErrGetState;

//测量输出阻抗需要的全局变量
extern volatile uint8_t Out_state;
extern volatile float Vout_Load;     //接入负载时的输出电压
extern volatile float Vout_Open;     //负载开路时的输出电压
extern volatile bool Vout_flag;      //是否测量完成开路和负载输出电压的标志位
extern volatile uint8_t count_open;
extern volatile uint8_t count_load;
extern volatile float Vout_Load_buffer[ADC_NUM];
extern volatile float Vout_Open_buffer[ADC_NUM];
extern volatile uint8_t vout_index;

//绘制增益曲线需要的全局变量
extern PlotState_t PlotState;
extern volatile uint16_t point_index;
extern volatile uint16_t lastX;
extern volatile uint16_t lastY;
extern volatile float input_freq;    
extern volatile float input_amp;     
extern volatile float freq_buffer[MAX_POINTS];
extern volatile float gain_buffer[MAX_POINTS];

//检测错误所需
extern Err_code_t Err_code;

void TaskScan(void);
void BTNScan(void);
void CalInputImp(void);
void CalOutputImp(void);
void CalGain(void);
void PlotAmpFreq(void);
float AD8310_Map(float Amp);
void parse_rx_buffer(char* , float* , float* );
uint16_t mapFreqToX(float freq);
uint16_t mapGainToY(float gain);

void get_3dbcutoff_freq(float *freq, float *gain);
void Param_update(Cir_param_t cp);

void CalErrGet(void);
void Err_Screen_Update();


#endif