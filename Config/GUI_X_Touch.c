#include "GUI.h"
#include "GUI_X.h"
#include "LCD22.h"

void GUI_TOUCH_X_ActivateX(void)   //��������
{
}
void GUI_TOUCH_X_ActivateY(void)   //��������
{
}
int GUI_TOUCH_X_MeasureX(void)
{
    //alt_u16 var = get_touch_data(TOUCH_CMD_X); //��ȡX���ADת��ֵ ��������ֵ

    //return var;

	return curr_pot.x;
}
int GUI_TOUCH_X_MeasureY(void)
{
    //alt_u16 var = get_touch_data(TOUCH_CMD_Y); //��ȡY���ADת��ֵ

    //return var;

	return curr_pot.y;
}