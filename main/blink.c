#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_timer.h"
#include "task_triggle_scan.h"


#define BLINK_GPIO 26       //灯

//模拟开关
#define CD0_EN   27        //模拟开关0使能
#define CD0_A    12        //模拟开关0A
#define CD0_B    14        //模拟开关0B
#define CD1_EN   21        //模拟开关1使能
#define CD1_A    18        //模拟开关1A
#define CD1_B    19        //模拟开关2B

#define SIGNAL_0 23       //信号0
#define SIGNAL_1 22       //信号1

#define SW_0 33       //信号0--S2--Read2--Machine2--CD0
#define SW_1 25       //信号1--S1--Read1--Machine1—--CD1

#define U1_TXD (GPIO_NUM_4)
#define U1_RXD (GPIO_NUM_13)
#define U2_TXD (GPIO_NUM_16)
#define U2_RXD (GPIO_NUM_17)
#define BUF_SIZE (1024)

//--------------------ADC----------------------------
#define NO_OF_SAMPLES   128          //Multisampling
static const adc_channel_t channel = ADC_CHANNEL_7;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_0;


uart_config_t uart_config = {
    .baud_rate = 115200,            //波特率
    .data_bits = UART_DATA_8_BITS,  //数据位数
    .parity = UART_PARITY_DISABLE,  //奇偶控制
    .stop_bits = UART_STOP_BITS_1,  //停止位
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, //流控位
};

unsigned char time_stamp[4] = {0};


char cmd_list[7][8]={
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},//等待初始化
    {0x40,0x03,0x00,0x00,0x00,0x00,0x00,0x00},//Reset
    {0x00,0x00,0x05,0x00,0x00,0x00,0x00,0x00},//check error--repeat
    {0x00,0x00,0x06,0x00,0x00,0x00,0x00,0x00},//check last error--repeat
    {0x01,0x00,0x00,0xf0,0x18,0x00,0x00,0x00},//RF POWER OFF
    {0x01,0x00,0x00,0xf0,0x27,0x00,0x00,0x00},//"RF POWER ON"
    {0x01,0x00,0x00,0xf0,0x0f,0x00,0x00,0x00},//"Read card"
};

uint32_t get_light(){
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
    printf("%d\n", adc_reading);
    return adc_reading;
}

void int_all(){

    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);
    server_triggle();



    uart_driver_install(UART_NUM_1,BUF_SIZE,0,0,NULL,0);
    uart_param_config(UART_NUM_1,&uart_config);
    uart_set_pin(UART_NUM_1, U1_TXD, U1_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_2,BUF_SIZE,0,0,NULL,0);
    uart_param_config(UART_NUM_2,&uart_config);
    uart_set_pin(UART_NUM_2, U2_TXD, U2_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    gpio_reset_pin(BLINK_GPIO);
    gpio_reset_pin(SIGNAL_0);
    gpio_reset_pin(SIGNAL_1);

    gpio_reset_pin(CD0_EN);
    gpio_reset_pin(CD0_A);
    gpio_reset_pin(CD0_B);
    gpio_reset_pin(CD1_EN);
    gpio_reset_pin(CD1_A);
    gpio_reset_pin(CD1_B);


    gpio_reset_pin(SW_0);
    gpio_reset_pin(SW_1);

    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(SIGNAL_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(SIGNAL_1, GPIO_MODE_OUTPUT);

    gpio_set_direction(CD0_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(CD0_A, GPIO_MODE_OUTPUT);    
    gpio_set_direction(CD0_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(CD1_EN, GPIO_MODE_OUTPUT);
    gpio_set_direction(CD1_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(CD1_B, GPIO_MODE_OUTPUT);

    gpio_set_direction(SW_0, GPIO_MODE_INPUT);
    gpio_set_direction(SW_1, GPIO_MODE_INPUT);
};


void update_timestamp_ms(){
    int64_t temp_time = esp_timer_get_time()/1000;
    time_stamp[0] = (unsigned char)((temp_time)&0xff);
    time_stamp[1] = (unsigned char)((temp_time>>8)&0xff);
    time_stamp[2] = (unsigned char)((temp_time>>16)&0xff);
    time_stamp[3] = (unsigned char)((temp_time>>24)&0xff);
};

void begin_resp(uart_port_t uart_num){
    char begin[8]={
        0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x06, 0x02
    };
    uart_write_bytes(uart_num, begin, 8);
};

void reset_resp(uart_port_t uart_num){
    char check_last_error[8]={
        0x40, 0x03, 0xbf, 0xfc, 0xbf, 0xfc, 0xbf, 0xfc
    };
    uart_write_bytes(uart_num, check_last_error, 8);
};

void check_error_resp(uart_port_t uart_num){
    char check_error[8]={
        0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uart_write_bytes(uart_num, check_error, 8);
};

void check_last_error_resp(uart_port_t uart_num){
    char check_last_error[8]={
        0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uart_write_bytes(uart_num, check_last_error, 8);
};


void power_off_resp(uart_port_t uart_num){
    char power_off_resp_1[16]={
        0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
    };
    char power_off_resp_2[16]={
        0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00
    };

    update_timestamp_ms();
    power_off_resp_1[12] =  time_stamp[0];
    power_off_resp_1[13] =  time_stamp[1];
    power_off_resp_1[14] =  time_stamp[2];
    power_off_resp_1[15] =  time_stamp[3];
    uart_write_bytes(uart_num, power_off_resp_1, 16);
    update_timestamp_ms();
    power_off_resp_2[8]  =  time_stamp[0];
    power_off_resp_2[9]  =  time_stamp[1];
    power_off_resp_2[10] =  time_stamp[2];
    power_off_resp_2[11] =  time_stamp[3];
    uart_write_bytes(uart_num, power_off_resp_2, 16);
};

void power_on_resp(uart_port_t uart_num){
    
    char power_on_resp_1[16]={
        0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x27, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
    };
    char power_on_resp_2[16]={
        0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00
    };

    update_timestamp_ms();
    power_on_resp_1[12] =  time_stamp[0];
    power_on_resp_1[13] =  time_stamp[1];
    power_on_resp_1[14] =  time_stamp[2];
    power_on_resp_1[15] =  time_stamp[3];
    uart_write_bytes(uart_num, power_on_resp_1, 16);
    update_timestamp_ms();
    power_on_resp_2[8]  =  time_stamp[0];
    power_on_resp_2[9]  =  time_stamp[1];
    power_on_resp_2[10] =  time_stamp[2];
    power_on_resp_2[11] =  time_stamp[3];
    uart_write_bytes(uart_num, power_on_resp_2, 16);
};

void read_card_resp(uart_port_t uart_num,bool state){
    //01 00 00 00 02 00 00 00 0F 00 00 00 D4 3F 1F 00
    //01 12 05 00 07 00 00 00 E6 3F 1F 00 64 84 04 01 
    //AA FE 59 3F 30 00 E2 80 69 95 00 00 50 03 1C 73 A8 23 A5 68 
    //01 00 01 00 02 00 00 00 E7 3F 1F 00 00 00 00 00 

    char read_card_resp_1[16]={
        0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
    };
    char read_card_resp_2_0[16]={
        0x01, 0x12, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x5a, 0x80, 0x04, 0x01
    };
    char read_card_resp_2_1[20]={
        0x5e, 0xfe, 0x0a, 0x3f, 0x30, 0x00, 0xe2, 0x80, 0x69, 0x95, 0x00, 0x00, 0x50, 0x02, 0x7f, 0x56, 0xce, 0x2f, 0xc0, 0x01
    };
    char read_card_resp_3[16]={
        0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00
    };
    update_timestamp_ms();
    read_card_resp_1[12] =  time_stamp[0];
    read_card_resp_1[13] =  time_stamp[1];
    read_card_resp_1[14] =  time_stamp[2];
    read_card_resp_1[15] =  time_stamp[3];
    uart_write_bytes(uart_num, read_card_resp_1, 16);

    if(state){
        update_timestamp_ms();
        read_card_resp_2_0[8]  =  time_stamp[0];
        read_card_resp_2_0[9]  =  time_stamp[1];
        read_card_resp_2_0[10] =  time_stamp[2];
        read_card_resp_2_0[11] =  time_stamp[3];
        uart_write_bytes(uart_num, read_card_resp_2_0, 16);
        uart_write_bytes(uart_num, read_card_resp_2_1, 20);

    }

    update_timestamp_ms();
    read_card_resp_3[8]  =  time_stamp[0];
    read_card_resp_3[9]  =  time_stamp[1];
    read_card_resp_3[10] =  time_stamp[2];
    read_card_resp_3[11] =  time_stamp[3];
    uart_write_bytes(uart_num, read_card_resp_3, 16);
};

void inventor_resp(uart_port_t uart_num,bool state){

    char inventor_resp_1[16]={
        0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
    };
    char inventor_resp_1_1[12]={
        0x01, 0x12, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff
    };  
    char inventor_resp_1_2[36]={
        0x4c,0x76,0x04,0x01,0x00,0x00,0x7c,0x3f,0x30,0x00,0xe2,0x80,0x68,0x94,0x00,0x00,0x50,0x1d,0xd1,0x36,0x01,0xa8,0x11,0x86,0x01,0x00,0x06,0x00,0x06,0x00,0x00,0x00,0xff,0xff,0xff,0xff
    }; 
    char inventor_resp_1_3[20]={
        0xc2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe2, 0x80, 0x68, 0x94,0x00,0x00,0x50,0x1d,0xd1,0x36,0x01,0xa8
    };       
    //4C 76 04 01 00 00 7C 3F 30 00 E2 80 68 94 00 00 50 1D D1 36 01 A8 
    //11 86 01 00 06 00 06 00 00 00 E3 0D DE 1E 
    //C2 00 00 00 00 00 00 00 E2 80 68 94 00 00 50 1D D1 36 01 A8 
    char inventor_resp_2[16]={
        0x01, 0x00, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00
    };

    update_timestamp_ms();
    inventor_resp_1[12] =  time_stamp[0];
    inventor_resp_1[13] =  time_stamp[1];
    inventor_resp_1[14] =  time_stamp[2];
    inventor_resp_1[15] =  time_stamp[3];
    uart_write_bytes(uart_num, inventor_resp_1, 16);
    // printf("state %d\n",state);

    if(state){   

        update_timestamp_ms();
        inventor_resp_1_1[8] =  time_stamp[0];
        inventor_resp_1_1[9] =  time_stamp[1];
        inventor_resp_1_1[10] =  time_stamp[2];
        inventor_resp_1_1[11] =  time_stamp[3];
        uart_write_bytes(uart_num, inventor_resp_1_1, 12);
        update_timestamp_ms();
        inventor_resp_1_2[32] =  time_stamp[0];
        inventor_resp_1_2[33] =  time_stamp[1];
        inventor_resp_1_2[34] =  time_stamp[2];
        inventor_resp_1_2[35] =  time_stamp[3];
        uart_write_bytes(uart_num, inventor_resp_1_2, 32);
        // uart_write_bytes(uart_num, inventor_resp_1_3, 20);

    }

    update_timestamp_ms();
    inventor_resp_2[8]  =  time_stamp[0];
    inventor_resp_2[9]  =  time_stamp[1];
    inventor_resp_2[10] =  time_stamp[2];
    inventor_resp_2[11] =  time_stamp[3];
    uart_write_bytes(uart_num, inventor_resp_2, 16);
};

void app_main(void)
{
    int_all();


    gpio_set_level(SIGNAL_0, 1);
    gpio_set_level(SIGNAL_1, 1);
    int rxBytes = 0;
    uint8_t rxData[8]={0};
    uart_port_t temp_uart[2] = {UART_NUM_1,UART_NUM_2};
    int state_flag[2] = {0};
    gpio_num_t signal[2] = {SIGNAL_0,SIGNAL_1};
    int count = 0;
    int sw0flag=0,sw1flag = 0;
    if(gpio_get_level(SW_1)){
        if(sw1flag!=1){
            gpio_set_level(CD0_EN, 0);
            gpio_set_level(CD0_A, 1);
            gpio_set_level(CD0_B, 0);
            sw1flag=1;
        };

    }else{
        if(sw1flag!=2){
            gpio_set_level(CD0_EN, 0);
            gpio_set_level(CD0_A, 0);
            gpio_set_level(CD0_B, 1);
            sw1flag=2;

        };
    };
    if(gpio_get_level(SW_0)){
        if(sw0flag!=1){
            gpio_set_level(CD1_EN, 0);
            gpio_set_level(CD1_A, 0);
            gpio_set_level(CD1_B, 1);
            sw0flag=1;
        };
    }else{
        if(sw0flag!=2){
            gpio_set_level(CD1_EN, 0);
            gpio_set_level(CD1_A, 1);
            gpio_set_level(CD1_B, 0);
            sw0flag=2;
        };
    };
    int test = 0;
    int triggle_flag_temp =999 ;
    
    while(1) {


        // gpio_set_level(BLINK_GPIO, 1);
        // if(get_light()<4000)break;

        // printf("switch 0 %d switch 1 %d\n",gpio_get_level(SW_0),gpio_get_level(SW_1));
        for(int i = 0;i<2;i++){
            
            rxBytes = uart_read_bytes(temp_uart[i],rxData,8,20 / portTICK_PERIOD_MS);
            if(rxBytes==1){
                if(rxData[0]==0x00){
                    state_flag[i]++;
                    // printf("0x00\n");
                }else{
                    state_flag[i]=0;
                }
                if(state_flag>=7){
                    begin_resp(temp_uart[i]);
                    state_flag[i]=0;
                    // printf("send_begin\n");
                }
            }
            if(rxBytes==8){
                // printf("serial[%d]:cmd[%x][%x][%x][%x][%x][%x][%x][%x]\n",i,rxData[0],rxData[1],rxData[2],rxData[3],rxData[4],rxData[5],rxData[6],rxData[7]);
                // printf("serial[%d]\n",i+1);

                if((rxData[0]==0x40)&&(rxData[1]==0x03)&&(rxData[2]==0x00)&&(rxData[3]==0x00)&&(rxData[4]==0x00)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    reset_resp(temp_uart[i]);
                    // printf("reset_resp\n");
                };
                if((rxData[0]==0x00)&&(rxData[1]==0x00)&&(rxData[2]==0x05)&&(rxData[3]==0x00)&&(rxData[4]==0x00)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    check_error_resp(temp_uart[i]);
                    // printf("check_error_resp\n");
                };
                if((rxData[0]==0x00)&&(rxData[1]==0x00)&&(rxData[2]==0x06)&&(rxData[3]==0x00)&&(rxData[4]==0x00)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    check_last_error_resp(temp_uart[i]);
                    // printf("check_last_error_resp\n");
                };
                if((rxData[0]==0x01)&&(rxData[1]==0x00)&&(rxData[2]==0x00)&&(rxData[3]==0xf0)&&(rxData[4]==0x18)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    power_off_resp(temp_uart[i]);
                    // printf("power_off_resp\n");
                };
                if((rxData[0]==0x01)&&(rxData[1]==0x00)&&(rxData[2]==0x00)&&(rxData[3]==0xf0)&&(rxData[4]==0x27)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    power_on_resp(temp_uart[i]);
                    // printf("power_off_resp\n");
                };  
                if((rxData[0]==0x01)&&(rxData[1]==0x00)&&(rxData[2]==0x00)&&(rxData[3]==0xf0)&&(rxData[4]==0x0f)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){
                    printf("read card\n");
                    triggle_flag_temp = triggle_flag;
                    gpio_set_level(signal[i], 0);
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                    gpio_set_level(signal[i], 1);
                    vTaskDelay(60 / portTICK_PERIOD_MS);


                    if(triggle_flag!=triggle_flag_temp){
                        test = gpio_get_level(32);
                    }            
                    if(test){
                        read_card_resp(temp_uart[i],false);
                    }else{
                        read_card_resp(temp_uart[i],true);
                    }
                    vTaskDelay(10 / portTICK_PERIOD_MS);

                }; 

                if((rxData[0]==0x01)&&(rxData[1]==0x00)&&(rxData[2]==0x00)&&(rxData[3]==0xf0)&&(rxData[4]==0x10)&&(rxData[5]==0x00)&&(rxData[6]==0x00)&&(rxData[7]==0x00)){

                    // triggle_flag=0;
                    triggle_flag_temp = triggle_flag;
                    gpio_set_level(signal[i], 0);
                    vTaskDelay(5 / portTICK_PERIOD_MS);
                    gpio_set_level(signal[i], 1);
                    vTaskDelay(60 / portTICK_PERIOD_MS);


                    if(triggle_flag!=triggle_flag_temp){
                        test = gpio_get_level(32);
                    }            
                    if(test){
                        inventor_resp(temp_uart[i],false);
                    }else{
                        inventor_resp(temp_uart[i],true);
                    }
                    printf("inventor_resp %d\n",triggle_flag);
 
                }; 
                
            }else{
                // printf("unknow cmd[%x][%x][%x][%x][%x][%x][%x][%x]\n",rxData[0],rxData[1],rxData[2],rxData[3],rxData[4],rxData[5],rxData[6],rxData[7]);

            }    
            vTaskDelay(10 / portTICK_PERIOD_MS);

        }



        // vTaskDelay(2030 / portTICK_PERIOD_MS);
        //读取数据如果收到卡查询--signal脚电平转换
        //读取trigger脚，如果收到电平信号，返回数据




    }
}
