#include "UserTask.h"

/*
@brief: 轮询扫描当前该执行的任务
*/
void TaskScan(void) {
    switch (TaskMark) {
        case InputImp:
            //sendString("CalInputImp()\r\n");
            CalInputImp();
            break;
        case OutputImp:
            //sendString("CalOutputImp()\r\n");
            CalOutputImp();
            break;
        case Gain:
            //sendString("CalGain()\r\n");
            CalGain();
            break;
        case AmpFreq:
            //sendString("PlotAmpFreq()\r\n");
            PlotAmpFreq();
            break;
        case ErrGet:
            CalErrGet();
        default:
            break;
    }
}

/*
@brief: 轮询扫描按键的状态，判断哪个按键被按下
*/
void BTNScan(void) {
    BTN_getData(&BTNData);
    if(BTNData.left) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_B_PIN);
        //sendString("left pressed\r\n");
        TaskMark = InputImp;    //left按键对应“测量输入阻抗”
    }
    if(BTNData.right) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_B_PIN);
        //sendString("right pressed\r\n");
        TaskMark = OutputImp;   //right按键对应“测量输出阻抗”
    }
    if(BTNData.down) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_B_PIN);
        //sendString("down pressed\r\n");
        TaskMark = Gain;        //down按键对应“测量增益”
    }
    if(BTNData.up) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_B_PIN);
        //sendString("up pressed\r\n");
        TaskMark = AmpFreq;     //up按键对应“绘制幅频曲线”
    }
    if(BTNData.mid) {
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_B_PIN);
        //sendString("mid pressed\r\n");
        TaskMark = ErrGet;     //up按键对应“绘制幅频曲线”
    }
}

/*
@brief: 测量输入阻抗
*/
void CalInputImp(void) {
    switch (InputImpState) {
        case IDLE:
            NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);  //使能ADC中断
            DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);
            DL_Common_delayCycles(32000000);    //延时1s
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_startConversion(ADC12_0_INST);
            InputImpState = WAIT;
            break;
        case WAIT:
            if(gCheckADC) {
                InputImpState = PROCESS;
            }
            break;
        case PROCESS:
            uint16_t InVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
            //sendNum(InVol, 2);  sendString("\r\n");
            float InAmp = AD8310_Map((float)InVol * 3300 / 4095);
            float R_i = InAmp / (SIGNAL_IN_mVPP - InAmp) * RESISTOR_IN;
            R_i = 1 / (1 / R_i - 1 / RESISTOR_BING);
            c_param.Ri = R_i;
            sendString("Input Resistor: ",UART_1_INST); sendNum(R_i, 2,UART_1_INST); sendString("\r\n",UART_1_INST);

            Param_update(c_param);
            gCheckADC = false;
            InputImpState = IDLE;
            break;
    }
    //按键打断当前任务
    if(TaskMark != InputImp) {
        InputImpState = IDLE;
        NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
    }
}

/*
@brief: 测量输出阻抗
*/
void CalOutputImp(void) {
    switch (OutputImpState) {
        case IDLE:
            NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);  //使能ADC中断
            DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);
            DL_Common_delayCycles(32000000);    //延时1s
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_startConversion(ADC12_0_INST);
            Out_state = Open;       //先测量开路的输出电压
            Vout_flag = false;
            OutputImpState = WAIT;
            break;
        case WAIT:
            if(gCheckADC) {
                OutputImpState = PROCESS;
            }
            break;
        case PROCESS:
            if(Out_state == Open) {
                uint16_t OutVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
                float vout_open = AD8310_Map((float)OutVol * 3300 / 4095);
                sendString("Vout_Open: ",UART_1_INST); sendNum(vout_open, 2,UART_1_INST); sendString("\r\n",UART_1_INST);
                
                count_open++;
                if(count_open >= ADC_NUM - 1) {
                    Vout_Open = 0.0;
                    for(int i = 0; i < ADC_NUM; i++) {
                        Vout_Open += Vout_Open_buffer[i];
                    }
                    Vout_Open /= ADC_NUM;
                    vout_index = 0;
                    Out_state = Load;
                    DL_GPIO_setPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //吸合继电器 -> 连接负载
                    DL_Common_delayCycles(32000000);    //延时1s
                    count_open = 0;
                }
                else {
                    Vout_Open_buffer[vout_index++] = vout_open;
                }

                DL_ADC12_enableConversions(ADC12_0_INST);
                DL_ADC12_startConversion(ADC12_0_INST);
                gCheckADC = false;
                OutputImpState = WAIT;
            }
            else if(Out_state == Load) {
                uint16_t OutVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
                float vout_load = AD8310_Map((float)OutVol * 3300 / 4095);
                sendString("Vout_Load: ",UART_1_INST); sendNum(vout_load, 2,UART_1_INST); sendString("\r\n",UART_1_INST);

                count_load++;
                if(count_load >= ADC_NUM - 1) {
                    Vout_Load = 0.0;
                    for(int i = 0; i < ADC_NUM; i++) {
                        Vout_Load += Vout_Load_buffer[i];
                    }
                    Vout_Load /= ADC_NUM;
                    vout_index = 0;
                    count_load = 0;
                    float R_o = (Vout_Open / Vout_Load - 1.0f) * RESISTOR_LOAD;
                    c_param.Ro = R_o;
                    sendString("Output Resistor: ",UART_1_INST); sendNum(R_o, 2,UART_1_INST); sendString("\r\n",UART_1_INST);
                    Param_update(c_param);
                    OutputImpState = IDLE;
                    gCheckADC = false;
                }
                else {
                    Vout_Load_buffer[vout_index++] = vout_load;
                    gCheckADC = false;
                    OutputImpState = WAIT;
                    DL_ADC12_enableConversions(ADC12_0_INST);
                    DL_ADC12_startConversion(ADC12_0_INST);
                }
            }
            break;
    }
    if(TaskMark != OutputImp) {
        OutputImpState = IDLE;
        NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
        DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //断开负载
        DL_Common_delayCycles(32000000);    //延时1s
    }
}

/*
@brief: 测量增益
*/
void CalGain(void) {
    switch (GainState) {
        case IDLE:
            NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);  //使能ADC中断
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_startConversion(ADC12_0_INST);
            DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //断开负载
            DL_Common_delayCycles(32000000);    //延时1s
            GainState = WAIT;
            break;
        case WAIT:
            if(gCheckADC) {
                GainState = PROCESS;
            }
            break;
        case PROCESS:
            uint16_t OutVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
            uint16_t InVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
            float InAmp = AD8310_Map((float)InVol * 3300 / 4095);
            float OutAmp = AD8310_Map((float)OutVol * 3300 / 4095);
            float Gain = OutAmp / InAmp;
            c_param.Av = Gain;

            sendString("Gain: ",UART_1_INST); sendNum(Gain, 2,UART_1_INST); sendString("\r\n",UART_1_INST);
            Param_update(c_param);
            gCheckADC = false;
            GainState = IDLE;
            NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
            break;
    }
    if(TaskMark != Gain) {
        GainState = IDLE;
        NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
    }
}

/*
@brief: 绘制幅频曲线
*/
void PlotAmpFreq(void) {
    switch (PlotState) {
        case WAIT_COMMAND:
            NVIC_EnableIRQ(UART_0_INST_INT_IRQN);   //开启串口中断
            DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //断开负载
            //DL_Common_delayCycles(32000000);    //延时1s
            NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);  //使能ADC中断
            if(uart_rx_finish) {
                parse_rx_buffer((char *)uart_rx_buffer, &input_freq, &input_amp);
                uart_rx_finish = false;
                
                uart_rx_index = 0;
                NVIC_DisableIRQ(UART_0_INST_INT_IRQN);  //关闭中断直到采样完成
                PlotState = START_ADC;
                
            }
            break;
        case START_ADC:
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_startConversion(ADC12_0_INST);
            PlotState = WAIT_ADC;
            break;
        case WAIT_ADC:
            if(gCheckADC) {
                gCheckADC = false;
                PlotState = DRAW_POINT;
            }
            break;
        case DRAW_POINT:
            uint16_t OutVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
            uint16_t InVol = DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
            float InAmp = AD8310_Map((float)InVol * 3300 / 4095);
            float OutAmp = AD8310_Map((float)OutVol * 3300 / 4095);
            float gain = OutAmp / InAmp;
            //float gain = 100;
            if(input_freq >= 100.0f && input_freq <= 1000000.0f){
                if(point_index < MAX_POINTS) {
                    freq_buffer[point_index] = input_freq;
                    gain_buffer[point_index] = gain;
                    point_index++;
                }
            }
            uint16_t x = mapFreqToX(input_freq);
            uint16_t y = mapGainToY(gain);
            
            if(lastX >= 400 && lastX <= 700) {
                //画线
                draw_line(x, y, lastX, lastY, BLUE);
            }
            //画第一个点
            else{
                draw_dot(x,y, BLUE);
            }
            lastX = x; lastY = y;
            //绘制完毕，可以计算上截频并更新参数了
            if(point_index == MAX_POINTS - 1 || input_freq >= 1000000.0f) 
            {
                get_3dbcutoff_freq(freq_buffer, gain_buffer);
                Param_update(c_param);
                point_index = 0;
            }
            PlotState = WAIT_COMMAND;
            break;
    }
    if(TaskMark != AmpFreq) {
        PlotState = WAIT_COMMAND;
        point_index = 0;
        lastX = lastY = -1;
        DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //断开负载
        DL_Common_delayCycles(32000000);    //延时1s
        NVIC_DisableIRQ(UART_0_INST_INT_IRQN);
        NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
    }
}

/*
@brief: AD8310输出电平 -> 输入信号幅值
*/
float AD8310_Map(float Amp) {
    float res = 0.0;
    //拟合公式: Vout = 491.7170 * log10(Vin) + 930.7041
    //res = 491.7170 * log10(Amp) + 930.7041;
    res = pow(10, (Amp - 930.7041) / 491.7170);
    return res;
}

/*
@brief: 将串口接收的信号相关信息解包
@param: buffer - 串口缓冲区指针
@param: freq - 频率
@param: mag - 幅度
*/
void parse_rx_buffer(char* buffer, float* freq, float* mag) {
    *freq = strtof(buffer, NULL);
    *mag = strtof(strchr(buffer, ' ') + 1, NULL);
}

/*
@brief: 将频率映射到显示屏的X坐标
*/
uint16_t mapFreqToX(float freq) {
    float logF = log10f(freq) * 10 - 20;
    float logStart = log10f(FREQ_START) * 10 - 20;
    float logEnd = log10f(FREQ_END) * 10 - 20;
    uint32_t res = 400 + (uint32_t)(((logF - logStart) / (logEnd - logStart)) * 300.0 /*此处应为显示屏的宽度*/);
    uint16_t result = (res <= 700) ? res : 700;
    return result;
}

/*
@brief: 将增益映射到显示屏的Y坐标
*/
uint16_t mapGainToY(float gain) {
    if(gain > GAIN_MAX) gain = GAIN_MAX;
    if(gain < 0) gain = 0;
    uint32_t res = 350 - (uint32_t)((gain / GAIN_MAX) * 300.0 /*此处应为显示屏的高度*/);
    uint16_t result = (res >= 50) ? res : 50;
    return result;
}

/**
 * @brief 获取-3dB点的对应频率（上截止频率）
 * 
 * @param freq 
 * @param gain 
 */
void get_3dbcutoff_freq(float *freq, float *gain)
{
    float gain_max = gain[0];
    for(int i = 1; i < MAX_POINTS; i++)
    {
        if(gain[i] > gain_max)      gain_max = gain[i];
    }

    float gain_cutoff = 0.707 * gain_max;
    //反方向遍历寻找上截频
    for(int j = MAX_POINTS; j > 0; j--)
    {
        //找到第一个大于等于-3dB增益的那个频率点
        if(gain[j] >= gain_cutoff)
        {
            c_param.fh = freq[j];
            break;
        }
    }
}


/**
 * @brief 更新屏幕上电路参数
 * 
 * @param cp 
 */
void Param_update(Cir_param_t cp)
{
    char str[30];
    sprintf(str, "Ri.txt=\"%.2fΩ\"\xff\xff\xff",cp.Ri);
    sendString(str, UART_2_INST);
    sprintf(str, "Ro.txt=\"%.2fΩ\"\xff\xff\xff",cp.Ro);
    sendString(str, UART_2_INST);
    sprintf(str, "Av.txt=\"%.2f\"\xff\xff\xff",cp.Av);
    sendString(str, UART_2_INST);
    sprintf(str, "fh.txt=\"%.2fHz\"\xff\xff\xff",cp.fh);
    sendString(str, UART_2_INST);  
}

void CalErrGet(void){
    switch (ErrGetState) {
        case IDLE:
            NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);  //使能ADC中断
            DL_ADC12_enableConversions(ADC12_0_INST);
            DL_ADC12_startConversion(ADC12_0_INST);
            DL_GPIO_clearPins(GPIO_CON_PORT, GPIO_CON_OUT_PIN);   //断开负载
            DL_Common_delayCycles(32000000);    //延时1s
            ErrGetState = WAIT;
            break;
        case WAIT:
            if(gCheckADC) {
                ErrGetState = PROCESS;
            }
            break;
        case PROCESS:
            uint32_t OutVolSum = 0;
            uint32_t InVolSum = 0;
            for(int i = 0; i < 10 && gCheckADC; i++){
                OutVolSum += DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_1);
                InVolSum += DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);
                gCheckADC = false;
                while (!gCheckADC)
                    ;
            }
            uint16_t OutVol = OutVolSum / 10;
            uint16_t InVol = InVolSum / 10;
            float InAD8310_Electrical_Level = (float)InVol * 3.3f / 4096;
            float OutAD8310_Electrical_Level = (float)OutVol * 3.3f / 4096;

            //开路检测
            if(fabsf(OutAD8310_Electrical_Level - R1_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R1_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R1_BROKEN;
                sendString("Err: R1 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);
            }else if(fabsf(OutAD8310_Electrical_Level - R2_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R2_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R2_BROKEN;
                sendString("Err: R2 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);
            }else if(fabsf(OutAD8310_Electrical_Level - R3_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R3_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R3_BROKEN;
                sendString("Err: R3 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);
            }else if(fabsf(OutAD8310_Electrical_Level - R4_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R4_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R4_BROKEN;
                sendString("Err: R4 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - C1_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - C1_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = C1_BROKEN;
                sendString("Err: C1 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - C2_BROKEN_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - C2_BROKEN_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = C2_BROKEN;
                sendString("Err: C2 Broken",UART_1_INST);sendString("\r\n",UART_1_INST);

            }
            //短路检测
            else if(fabsf(OutAD8310_Electrical_Level - R1_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R1_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R1_SHORT;
                sendString("Err: R1 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - R2_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R2_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R2_SHORT;
                sendString("Err: R2 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - R3_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R3_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R3_SHORT;
                sendString("Err: R3 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - R4_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - R4_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = R4_SHORT;
                sendString("Err: R4 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - C1_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - C1_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = C1_SHORT;
                sendString("Err: C1 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - C2_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - C2_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = C2_SHORT;
                sendString("Err: C2 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }else if(fabsf(OutAD8310_Electrical_Level - C3_SHORT_AD8310OUT_ELECTRICAL_LEVEL) <= BEAR_LEVEL && fabsf(InAD8310_Electrical_Level - C3_SHORT_AD8310IN_ELECTRICAL_LEVEL) <= BEAR_LEVEL){
                Err_code = C3_SHORT;
                sendString("Err: C3 Short",UART_1_INST);sendString("\r\n",UART_1_INST);

            }
            //都没检测到，为正常状态
            else{
                Err_code = NORMAL;
                sendString("Normal :-)",UART_1_INST);sendString("\r\n",UART_1_INST);

            }

            Err_Screen_Update();
            gCheckADC = false;
            ErrGetState = IDLE;
            NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
            break;
    }
    if(TaskMark != ErrGet) {
        ErrGetState = IDLE;
        NVIC_DisableIRQ(ADC12_0_INST_INT_IRQN);
    }
}

void Err_Screen_Update(){
    char str[30];
    switch (Err_code) {
        /*  正常状态    */
        case NORMAL:
            sprintf(str,"status.pco=GREEN\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"正常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=GREEN\xff\xff\xff");
            sprintf(str,"eor.txt=\"无\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        /*  元件开路    */
        case R1_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R1 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R2_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R2 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R3_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R3 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R4_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R4 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case C1_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"C1 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case C2_BROKEN:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"C2 开路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        /*  元件短路    */
        case R1_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R1 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R2_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R2 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R3_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R3 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case R4_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"R4 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case C1_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"C1 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case C2_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"C2 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        case C3_SHORT:
            sprintf(str,"status.pco=RED\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"status.txt=\"异常\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            sprintf(str,"eor.pco=RED\xff\xff\xff");
            sprintf(str,"eor.txt=\"C3 短路\"\xff\xff\xff");
            sendString(str,UART_2_INST);
            break;
        
    }
}
