#include "periph_referee.h"
#include "periph_draw.h"



const uint16_t Const_Referee_GRAPHIC_BUFFER_MAX_LENGTH              = 21;                   // 图形缓冲区最大长度
const Referee_RefereeCmdTypeDef Const_Referee_DATA_CMD_ID_LIST[6]   = {                     // 裁判系统交互数据内容ID
    {0x0100,    2,      NULL},              // 客户端删除图形
    {0x0101,    15,     NULL},              // 客户端绘制一个图形
    {0x0102,    30,     NULL},              // 客户端绘制二个图形
    {0x0103,    75,     NULL},              // 客户端绘制五个图形
    {0x0104,    105,    NULL},              // 客户端绘制七个图形
    {0x0110,    45,     NULL}               // 客户端绘制字符图形
 };

/********** Drawing Constants **********/

// 关于图层：图层0 ~ 9，高图层遮盖低图层
// 对于经常更新的分图层功能，建议前景图层使用3，背景图层使用2
// 其他功能在不产生遮挡的情况下建议使用图层2

// 关于坐标：左下角为 (0, 0)，水平方向为 X，垂直方向为 Y
//右上角为(1920,1080)
const uint8_t AIM_LINE_LAYER        = 2;                      //线层
const Draw_Color AIM_LINE_COLOR     = Draw_COLOR_BLACK;
const uint8_t AIM_LINE_LINE_MODE    = 3;                     //3种弹速模式
const uint8_t AIM_LINE_LINE_NUM     = 3 + 1;                 //4条直线
const uint32_t AIM_LINES[AIM_LINE_LINE_MODE][AIM_LINE_LINE_NUM][6] = {     //一个三维数组
    // ID, Width, X1, Y1, X2, Y2
    {       // Mode 0: 15 m/s
        {0x101, 6, 920, 400, 920, 520},     // Vertical Line
        {0x102, 8, 860, 500, 980, 500},     // Horizontal Line 1
        {0x103, 6, 880, 460, 960, 460},     // Horizontal Line 2
        {0x104, 6, 900, 420, 940, 420}      // Horizontal Line 3
    }, {    // Mode 1: 18 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 540, 950, 540},     // Horizontal Line 2
        {0x104, 2, 870, 500, 930, 500}      // Horizontal Line 3
    }, {    // Mode 2: 30 m/s
        {0x101, 6, 900, 440, 900, 560},     // Vertical Line
        {0x102, 8, 840, 540, 900, 540},     // Horizontal Line 1
        {0x103, 6, 880, 500, 900, 500},     // Horizontal Line 2
        {0x104, 6, 920, 460, 900, 460}      // Horizontal Line 3
    }
};
graphic_data_struct_t Referee_dummyGraphicCmd = {{0x00, 0x00, 0x00}, Draw_OPERATE_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

graphic_data graphicData;
Referee_DrawDataTypeDef Referee_DrawData ;
/**
  * @brief      发送客户端自定义图形命令组
  * @param      graph: 数组包括指定数量个图形命令
  * @param      mode: 发送模式，1、2、3、4对应1、2、5、7个一组
  * @retval     无
  */
void Referee_SendDrawingCmd(graphic_data_struct_t graph[], uint8_t mode) {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
    if (mode == 0 || mode >= 5) return;
    
    uint8_t buf[120], cellsize = sizeof(graphic_data_struct_t);
    if (mode >= 1) {
        memcpy(buf, graph, cellsize);
    }
    if (mode >= 2) {
        memcpy(buf + cellsize, graph + 1, cellsize);
    }
    if (mode >= 3) {
        memcpy(buf + cellsize * 2, graph + 2, cellsize);
        memcpy(buf + cellsize * 3, graph + 3, cellsize);
        memcpy(buf + cellsize * 4, graph + 4, cellsize);
    }
    if (mode >= 4) {
        memcpy(buf + cellsize * 5, graph + 5, cellsize);
        memcpy(buf + cellsize * 6, graph + 6, cellsize);
    }
    
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[mode].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[mode].data_length);
}


/**
  * @brief      客户端自定义图形缓冲区是否为空
  * @param      无
  * @retval     1为空，0为非空
  */
uint8_t Referee_IsDrawingBufferEmpty() {
    graphic_data* referee = &graphicData;
    return referee->graphic_buf_len == 0;
}

/**
  * @brief      客户端自定义图形缓冲区刷写函数
  * @param      无
  * @retval     无
  */
void Referee_DrawingBufferFlush() {
    graphic_data* referee = &graphicData;
    if (Referee_IsDrawingBufferEmpty()) return;
    uint8_t cur = 0;
    while (cur + 7 < referee->graphic_buf_len) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
        cur += 7;
    }
    uint8_t remain = referee->graphic_buf_len - cur;
    if (remain > 5) {
        for (int i = remain; i < 7; ++i)
            Referee_DrawingBufferPushDummy();
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 4);
    }
    else if (remain > 2) {
        for (int i = remain; i < 5; ++i)
            Referee_DrawingBufferPushDummy();
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 3);
    }
    else if (remain == 2) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 2);
    }
    else if (remain == 1) {
        Referee_SendDrawingCmd(referee->graphic_buf + cur, 1);
    }
    referee->graphic_buf_len = 0;
}

/**
  * @brief      将空图形命令加入客户端自定义图形缓冲区（占位用）
  * @param      无
  * @retval     无
  */
void Referee_DrawingBufferPushDummy() {
    graphic_data* referee = &graphicData;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, &Referee_dummyGraphicCmd, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
}

/**
  * @brief      将图形命令加入客户端自定义图形缓冲区
  * @param      pgraph: 指针指向图形命令
  * @retval     无
  */
void Referee_DrawingBufferPush(graphic_data_struct_t *pgraph) {
    graphic_data* referee = &graphicData;
    memcpy(referee->graphic_buf + referee->graphic_buf_len, pgraph, sizeof(graphic_data_struct_t));
    ++referee->graphic_buf_len;
    if (referee->graphic_buf_len >= 7) {
        Referee_DrawingBufferFlush();
    }
}

/**
  * @brief      绘图函数，清空全部
  * @param      无
  * @retval     无
  */
void Draw_ClearAll() {
    Referee_RefereeDataTypeDef* referee = &Referee_RefereeData;
	graphic_data* data = &graphicData;
    Referee_DrawingBufferFlush();
    data->graphic_buf_len = 0;   // 直接抛弃缓冲区中的绘图指令
    uint8_t buf[2];
    buf[0] = 2;
    buf[1] = 0;
    Referee_SendInteractiveData(Const_Referee_DATA_CMD_ID_LIST[0].cmd_id, referee->client_id, 
                                buf, Const_Referee_DATA_CMD_ID_LIST[0].data_length);
}

/**
  * @brief      打包图形命令
  * @param      详见协议及头文件定义
  * @retval     是否合法（1为是，0为否）
  */
uint32_t Referee_PackGraphicData(graphic_data_struct_t *pgraph, uint32_t graph_id, 
                                 Draw_OperateType operate_tpye, Draw_GraphicType figure_tpye, uint8_t layer,
                                 Draw_Color color, uint16_t start_angle, uint16_t end_angle, 
                                 uint8_t width, uint32_t start_x, uint32_t start_y,
                                 uint16_t radius, uint32_t end_x, uint32_t end_y) 
{
    if (graph_id > 0xffffff) return PARSE_FAILED;
    pgraph->figure_name[0] = graph_id & 0xff;
    pgraph->figure_name[1] = (graph_id >> 8) & 0xff;
    pgraph->figure_name[2] = (graph_id >> 16) & 0xff;
    
    pgraph->operate_tpye = (uint8_t) operate_tpye;
    pgraph->figure_tpye = (uint8_t) figure_tpye;
    
    if (layer > 9) return PARSE_FAILED;
    pgraph->layer = layer;
    
    pgraph->color = (uint8_t) color;
    
    if (start_angle > 0x7ff || end_angle > 0x7ff) return PARSE_FAILED;
    pgraph->details_a = start_angle;
    pgraph->details_b = end_angle;
    
    pgraph->width = width;
    
    if (start_x > 0x7ff || start_x > 0x7ff || radius > 0x3ff || end_x > 0x7ff || end_y > 0x7ff) return PARSE_FAILED;
    pgraph->start_x = start_x;
    pgraph->start_y = start_y;
    pgraph->details_c = radius;
    pgraph->details_d = end_x;
    pgraph->details_e = end_y;
    
    return PARSE_SUCCEEDED;
}



/**
  * @brief      绘图函数，画直线（新增）
  * @param      详见协议及头文件定义
  * @retval     无
  */
void Draw_AddLine(uint32_t graph_id, uint8_t layer, Draw_Color color, uint8_t width, 
                  uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) 
{
    graphic_data_struct_t graph;
    if (Referee_PackGraphicData(&graph, graph_id, Draw_OPERATE_ADD, Draw_TYPE_LINE, layer, color, 
                                0, 0, width, start_x, start_y, 0, end_x, end_y) != PARSE_SUCCEEDED)
        return;
    Referee_DrawingBufferPush(&graph);
}


/**
  * @brief      瞄准线绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupAimLine() {
    // draw_cnt: 4
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->aim_mode_last = draw->aim_mode;
    const uint32_t (*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}


/**
  * @brief      瞄准线绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateAimLine() {
    // draw_cnt: 4 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
//    if (draw->aim_mode_last == draw->aim_mode) return;
    draw->aim_mode_last = draw->aim_mode;
    const uint32_t (*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}

/*****************************************************文字绘制*********************************************/
const uint16_t AIM_MODE_TEXT[5]     = {0x501, 20, 2, 600, 840}; 
const uint16_t AIM_MODE_VALUE_TEXT[5]  = {0x501, 20, 2, 1000, 840};
const uint8_t AIM_MODE_LAYER        = 2;
const Draw_Color AIM_MODE_COLOR     = Draw_COLOR_GREEN;
const char *AIM_MODE_TEXT_STR       = "AIM_MODE:";
const char *NORMAL_AIM_TEXT_STR     = "NORMAL";
const char *ARMOR_AIM_TEXT_STR      = "ARMOR";
const char *BIG_BUFF_AIM_TEXT_STR   = "BIG_BUF";
const char *SMALL_BUFF_AIM_TEXT_STR = "SMALL_BUF";



/*********************************************文字绘制结束*****************************************************************/
/**
  * @brief      设置瞄准线模式
  * @param      mode: 瞄准线模式（0 ~ 2对应弹速 15,18,30 m/s）
  * @retval     无
  */
void Referee_SetAimMode(uint8_t mode) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (mode > 2) return;
    draw->aim_mode = mode;
}


/**
  * @brief      初始化各绘制功能
  * @param      无
  * @retval     无
  */
void Referee_Setup() {     
    static int last_time = -1000;
    int now = HAL_GetTick();
    if (now - last_time < 1000) return;
    last_time = now;    
    
	Draw_ClearAll();                    // cmd_cnt: 1, total_cmd_cnt: 1
	Referee_SetupAimLine();            // draw_cnt: 4

}

void Referee_Update() { 
    Draw_ClearAll()	;
    Referee_UpdateAimLine();           // draw_cnt: if bullet speed changed 4, else 0
    Referee_DrawingBufferFlush();       
}