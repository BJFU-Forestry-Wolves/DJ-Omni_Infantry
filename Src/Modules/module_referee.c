/*
 *  Project      : Polaris
 * 
 *  file         : module_referee.c
 *  Description  : Polaris
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 11:18:25
 */


#include "module_referee.h"

#include "periph_referee.h"
#include "cmsis_os.h"




uint8_t referee_setup_flag = 0;


/********** Drawing Constants **********/

// 关于图层：图层0 ~ 9，高图层遮盖低图层
// 对于经常更新的分图层功能，建议前景图层使用3，背景图层使用2
// 其他功能在不产生遮挡的情况下建议使用图层2

// 关于坐标：左下角为 (0, 0)，水平方向为 X，垂直方向为 Y

const uint8_t AIM_LINE_LAYER        = 2;
const Draw_Color AIM_LINE_COLOR     = Draw_COLOR_GREEN;
const uint8_t AIM_LINE_LINE_MODE    = 3;
const uint8_t AIM_LINE_LINE_NUM     = 3 + 1;
const uint16_t AIM_LINES[AIM_LINE_LINE_MODE][AIM_LINE_LINE_NUM][6] = {  
    // ID, Width, X1, Y1, X2, Y2
    {       // Mode 0: 15 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 560, 950, 560},     // Horizontal Line 2
        {0x104, 2, 870, 520, 930, 520}      // Horizontal Line 3
    }, {    // Mode 1: 18 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 540, 950, 540},     // Horizontal Line 2
        {0x104, 2, 870, 500, 930, 500}      // Horizontal Line 3
    }, {    // Mode 2: 30 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 580, 950, 580},     // Horizontal Line 2
        {0x104, 2, 870, 560, 930, 560}      // Horizontal Line 3
    }
};

const uint8_t CROSSHAIR_LAYER       = 2;
const Draw_Color CROSSHAIR_COLOR    = Draw_COLOR_GREEN;
const uint16_t CROSSHAIR[5]         = {0x201, 2, 960, 560, 10};  // ID, Width, X, Y, R

const uint8_t WIDTH_MARK_LAYER      = 2;
const Draw_Color WIDTH_MARK_COLOR   = Draw_COLOR_YELLOW;
const uint16_t WIDTH_MARK_GYRO[2][6] = {      // ID, Width, X0, Y0, X1, Y1
    {0x301, 2, 660, 400, 660, 200},     // Left Mark Line, Normal
    {0x302, 2, 1260, 400, 1260, 200},     // Right Mark Line, Normal
};
const uint16_t WIDTH_MARK_NORMAL[2][6] = {
    {0x301, 2, 660, 400, 960, 200},     // Left Mark Line, Gyro Mode
    {0x302, 2, 1260, 400, 960, 200},     // Right Mark Line, Gyro Mode
};

const uint8_t CAP_STATE_LAYER[2]    = {3, 2};   // Foreground, Background
const Draw_Color CAP_STATE_COLOR[5] = {
    Draw_COLOR_WHITE,           // Background
    Draw_COLOR_GREEN,           // Text
    Draw_COLOR_GREEN,           // Foreground, Full (50% ~ 100%)
    Draw_COLOR_YELLOW,          // Foreground, Insufficient (10% ~ 50%)
    Draw_COLOR_ORANGE           // Foreground, Empty (0% ~ 10%)
};
const uint16_t CAP_STATE[4]         = {6, 960, 240, 40};     // Width, X, Y, R
const uint16_t CAP_STATE_CIRCLE     = 0x401;            // Background Circle ID
const uint16_t CAP_STATE_ARC        = 0x402;            // Foreground Arc ID
const uint16_t CAP_STATE_TEXT[5]    = {0x403, 20, 2, 900, 240};     // ID, Font Size, Width, X, Y
const char *CAP_STATE_TEXT_STR      = "CAP";

const uint8_t PITCH_METER_LAYER     = 2;
const Draw_Color PITCH_METER_COLOR  = Draw_COLOR_GREEN;
const uint16_t PITCH_METER_TEXT[5]  = {0x501, 20, 2, 1500, 540};     // ID, Font Size, Width, X, Y
const char *PITCH_METER_TEXT_STR    = "PITCH:";
const uint16_t PITCH_METER_VALUE[6] = {0x502, 20, 3, 2, 1200, 540};  // ID, Font Size, Precision, Width, X, Y

const uint8_t AIM_MODE_LAYER        = 2;
const Draw_Color AIM_MODE_COLOR     = Draw_COLOR_GREEN;
const uint16_t AIM_MODE_TEXT[5]     = {0x501, 20, 2, 600, 840};     // ID, Font Size, Width, X, Y
const uint16_t AIM_MODE_VALUE_TEXT[5]  = {0x501, 20, 2, 1000, 840};     // ID, Font Size, Width, X, Y
const char *AIM_MODE_TEXT_STR       = "AIM_MODE:";
const char *NORMAL_AIM_TEXT_STR     = "NORMAL";
const char *ARMOR_AIM_TEXT_STR      = "ARMOR";
const char *BIG_BUFF_AIM_TEXT_STR   = "BIG_BUF";
const char *SMALL_BUFF_AIM_TEXT_STR = "SMALL_BUF";

const uint8_t CHASSIS_MODE_LAYER    = 2;
const Draw_Color CHASSIS_MODE_COLOR = Draw_COLOR_GREEN;
const uint16_t CHASSIS_MODE_TEXT[5] = {0x501, 20, 2, 600, 540};     // ID, Font Size, Width, X, Y
const uint16_t CHASSIS_MODE_VALUE_TEXT[5]  = {0x501, 20, 2, 1000, 540};     // ID, Font Size, Width, X, Y
const char *CHASSIS_MODE_TEXT_STR  = "CHASSIS_MODE:";
const char *NORMAL_RUN_TEXT_STR    = "NORMAL";
const char *GYRO_RUN_TEXT_STR      = "GYRO";



/********** END OF Drawing Constants **********/


Referee_DrawDataTypeDef Referee_DrawData;



uint8_t syj = 0;

/**
  * @brief      初始化各绘制功能
  * @param      无
  * @retval     无
  */
void Referee_Setup() {     
   
}


/**
  * @brief      更新各绘制功能
  * @param      无
  * @retval     无
  */
void Referee_Update() {                

}
