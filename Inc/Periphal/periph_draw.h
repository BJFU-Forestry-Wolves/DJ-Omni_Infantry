#ifndef REFEREE_DRAW_H
#define REFEREE_DRAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_uart.h"
#include "alg_crc.h"
#include "string.h"

typedef struct {
    uint8_t width_mode, width_mode_last;        // 1 for gyro mode, 0 for normal mode
    uint8_t aim_mode, aim_mode_last;            // 0~2 correspond to 15,18,30 m/s bullet speed
    uint8_t auto_aim_mode, auto_aim_mode_last;
    uint8_t cha_mode, cha_mode_last;
    uint8_t cap_state;                          // cap percent, 0 ~ 100
    float pitch_angle;
} Referee_DrawDataTypeDef;

typedef __packed struct {
    uint8_t figure_name[3];         //图形名称
	uint32_t operate_tpye:3;        //图形配置0空操作，1增加，2修改，3删除
	uint32_t figure_tpye:3;         //图形类型，直线，椭圆，字符串等等
	uint32_t layer:4;               //图层数
	uint32_t color:4;               //颜色
	uint32_t details_a:9;           //根据图形决定
	uint32_t details_b:9; 
	uint32_t width:10;              //线宽
	uint32_t start_x:11;            //起点或圆心xy
	uint32_t start_y:11;  
	uint32_t details_c:10;          //根据图形决定
	uint32_t details_d:11;  
	uint32_t details_e:11;
} graphic_data_struct_t;

typedef struct {
	graphic_data_struct_t graphic_buf[30];      // 客户端自定义绘图缓冲区
    uint8_t graphic_buf_len;                    // 客户端自定义绘图缓冲区已占用长度
} graphic_data;



typedef __packed struct { 
    graphic_data_struct_t grapic_data_struct; 
} ext_client_custom_graphic_single_t;

typedef __packed struct { 
    graphic_data_struct_t grapic_data_struct[2]; 
} ext_client_custom_graphic_double_t;

typedef __packed struct { 
    graphic_data_struct_t grapic_data_struct[5]; 
} ext_client_custom_graphic_five_t;

typedef __packed struct { 
    graphic_data_struct_t grapic_data_struct[7]; 
} ext_client_custom_graphic_seven_t;

typedef __packed struct { 
    graphic_data_struct_t grapic_data_struct; 
    uint8_t data[30]; 
} ext_client_custom_character_t;

typedef __packed struct { 
    uint8_t delete_type; 
    uint8_t layer; 
} interaction_layer_delete_t;

typedef enum {
    Draw_OPERATE_NULL   = 0,    // 空操作
    Draw_OPERATE_ADD    = 1,    // 增加
    Draw_OPERATE_MODIFY = 2,    // 修改
    Draw_OPERATE_DELETE = 3     // 删除
} Draw_OperateType;             // 图形操作

typedef enum {
    Draw_TYPE_LINE      = 0,    // 直线
    Draw_TYPE_RECTANGLE = 1,    // 矩形
    Draw_TYPE_CIRCLE    = 2,    // 整圆
    Draw_TYPE_ELLIPSE   = 3,    // 椭圆
    Draw_TYPE_ARC       = 4,    // 圆弧
    Draw_TYPE_FLOAT     = 6,    // 浮点数
    Draw_TYPE_INT       = 5,    // 整型数
    Draw_TYPE_STRING    = 7     // 字符
} Draw_GraphicType;             // 图形类型

typedef enum {
    Draw_COLOR_RED_BLUE  = 0,   // 红蓝主色
    Draw_COLOR_YELLOW    = 1,   // 黄色
    Draw_COLOR_GREEN     = 2,   // 绿色
    Draw_COLOR_ORANGE    = 3,   // 橙色
    Draw_COLOR_VIOLET    = 4,   // 紫红色
    Draw_COLOR_PINK      = 5,   // 粉色
    Draw_COLOR_CYAN      = 6,   // 青色
    Draw_COLOR_BLACK     = 7,   // 黑色
    Draw_COLOR_WHITE     = 8    // 白色
} Draw_Color;                   // 颜色


void Referee_DrawingBufferPushDummy(void) ;
void Referee_DrawingBufferFlush(void);
void Referee_DrawingBufferPush(graphic_data_struct_t *pgraph);

void Referee_UpdateAimLine();
void Referee_SetupAimLine();
void Draw_ClearAll();
void Draw_AddLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                  uint16_t start_x, uint16_t start_y, uint16_t end_x, uint16_t end_y);
	

void Referee_Update();
void Referee_Setup();
void Referee_SetAimMode(uint8_t mode);
void Referee_SetupModeDisplay();


uint32_t Referee_PackGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                 Draw_OperateType operate_tpye, Draw_GraphicType figure_tpye, uint8_t layer,
                                 Draw_Color color, uint16_t start_angle, uint16_t end_angle, 
                                 uint8_t width, uint16_t start_x, uint16_t start_y,
                                 uint16_t radius, uint16_t end_x, uint16_t end_y) ;
#endif

#ifdef __cplusplus
}
#endif
