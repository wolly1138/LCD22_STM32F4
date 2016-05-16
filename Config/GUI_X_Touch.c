#include "GUI.h"
#include "GUI_X.h"
#include "LCD22.h"

void GUI_TOUCH_X_ActivateX(void)   //不用配置
{
}
void GUI_TOUCH_X_ActivateY(void)   //不用配置
{
}
int GUI_TOUCH_X_MeasureX(void)
{
    //alt_u16 var = get_touch_data(TOUCH_CMD_X); //读取X轴的AD转换值 不是坐标值

    //return var;

	return curr_pot.x;
}
int GUI_TOUCH_X_MeasureY(void)
{
    //alt_u16 var = get_touch_data(TOUCH_CMD_Y); //读取Y轴的AD转换值

    //return var;

	return curr_pot.y;
}