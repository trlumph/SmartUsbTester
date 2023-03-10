/*
 * draw.c
 *
 *  Created on: Dec 25, 2022
 *      Author: tymur
 */

#include "draw.h"
#include "fonts.h"
#include "ssd1306.h"
#include <stdio.h>

#define STEP 9
#define MENU_OFFSET 20

#define X_STEP 1
#define XN SSD1306_WIDTH/X_STEP
#define GRAPH_HEIGHT SSD1306_HEIGHT-MENU_OFFSET
#define GRAPH_WIDTH SSD1306_WIDTH
#define HEIGHT SSD1306_HEIGHT

uint8_t heights[XN];


void draw_init(){
	SSD1306_Init();
}

void draw_fill(int color){
	SSD1306_Fill(color);
}

void draw_clear(){
	SSD1306_Clear();
}

void draw_update_screen(){
	SSD1306_UpdateScreen();
}

void draw_exit_button(){
	SSD1306_DrawFilledRectangle(100 ,0, 10, 10 ,0);
	SSD1306_DrawRectangle(100 ,0, 10, 10 ,1);
}

void draw_exit_focus(){
	SSD1306_DrawFilledRectangle(100 ,0, 10, 10 ,1);
}

void draw_temperature(uint32_t t){
	SSD1306_GotoXY (6,0);
	char str[7];
	sprintf(str, "%lu C ", t);
	SSD1306_Puts(str, &Font_7x10, 1);
}

void draw_main_menu(page_t page) {
	SSD1306_Fill(0);
	switch (page) {
		case PAGE_1:
			draw_main_menu_page_1();
			break;
		case PAGE_2:
			draw_main_menu_page_2();
		default:
			break;
	}
}
void draw_main_menu_page_1() {
	  SSD1306_GotoXY (6,0);
	  SSD1306_Puts ("MENU", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1);
	  SSD1306_Puts ("QC", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1+1*STEP);
	  SSD1306_Puts ("POWER", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1+2*STEP);
	  SSD1306_Puts ("CURRENT CONTROL", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1+3*STEP);
	  SSD1306_Puts ("GRAPHS", &Font_7x10, 1);
}
void draw_main_menu_page_2() {
	  SSD1306_GotoXY (6,0);
	  SSD1306_Puts ("MENU", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1);
	  SSD1306_Puts ("MAX PARAMS", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1+1*STEP);
	  SSD1306_Puts ("RESISTANCE", &Font_7x10, 1);
	  SSD1306_GotoXY (6, MENU_OFFSET+1+2*STEP);
	  SSD1306_Puts ("CAPACITY", &Font_7x10, 1);
}

void draw_qc_menu(){
	SSD1306_GotoXY (6,0);
	SSD1306_Puts ("Quick Charge", &Font_7x10, 1);
	SSD1306_GotoXY (6, MENU_OFFSET-3);
	SSD1306_Puts ("Set 5V", &Font_7x10, 1);
	SSD1306_GotoXY (6, MENU_OFFSET-3+1*STEP);
	SSD1306_Puts ("Set 9V", &Font_7x10, 1);
	SSD1306_GotoXY (6, MENU_OFFSET-3+2*STEP);
	SSD1306_Puts ("Set 12V", &Font_7x10, 1);
	SSD1306_GotoXY (6, MENU_OFFSET-3+3*STEP);
	SSD1306_Puts ("QC 3.0 UP", &Font_7x10, 1);
	SSD1306_GotoXY (6, MENU_OFFSET-3+4*STEP);
    SSD1306_Puts ("QC 3.0 DOWN", &Font_7x10, 1);
}

void draw_qc_menu_deselect(uint16_t pos){
	SSD1306_DrawRectangle(6,MENU_OFFSET-4+(pos-1)*STEP, 124, 9 ,0);
}

void draw_qc_menu_focus(uint16_t pos){
	SSD1306_DrawRectangle(6,MENU_OFFSET-4+(pos-1)*STEP, 124, 9 ,1);
}

void draw_main_menu_selection(uint16_t move, uint16_t previous_move, page_t page){
	switch (page) {
		case PAGE_1:
			draw_main_menu_selection_page_1(move, previous_move);
			break;
		case PAGE_2:
			draw_main_menu_selection_page_2(move, previous_move);
			break;
		default:
			break;
	}

}
void draw_main_menu_selection_page_1(uint16_t move, uint16_t previous_move) {
	SSD1306_DrawRectangle(6,MENU_OFFSET+(previous_move)*STEP, 124, 9 ,0);
	SSD1306_DrawRectangle(6,MENU_OFFSET+(move)*STEP, 124, 9 ,1);

}
void draw_main_menu_selection_page_2(uint16_t move, uint16_t previous_move) {
	SSD1306_DrawRectangle(6,MENU_OFFSET+(previous_move-4)*STEP, 124, 9 ,0);
	SSD1306_DrawRectangle(6,MENU_OFFSET+(move-4)*STEP, 124, 9 ,1);

}

void draw_power_menu(uint32_t voltage, uint32_t amperage, uint32_t power){
    SSD1306_GotoXY (6,0);
    SSD1306_Puts("Power", &Font_7x10, 1);

    char str[14];
    sprintf(str, "%lu mV     ", voltage);
    SSD1306_GotoXY(6, MENU_OFFSET-3+2*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);

    char str2[14];
    sprintf(str2, "%lu mA     ", amperage);
    SSD1306_GotoXY(6, MENU_OFFSET-3+3*STEP);
    SSD1306_Puts(str2, &Font_7x10, 1);

    char str3[14];
    sprintf(str3, "%lu mW     ", power);
    SSD1306_GotoXY(6, MENU_OFFSET-3+4*STEP);
    SSD1306_Puts(str3, &Font_7x10, 1);
}

void draw_current_control_menu(int amperage_load, int amperage){
	// SSD1306_GotoXY (6,0);
	// SSD1306_Puts("Current Control", &Font_7x10, 1);
	char buf[8];
	snprintf(buf, 8, "%d    ", amperage_load);
	SSD1306_GotoXY (6, MENU_OFFSET+1);
	SSD1306_Puts (buf, &Font_7x10, 1);

	char str[14];
    sprintf(str, "%lu mA    ", amperage);
    SSD1306_GotoXY(6, MENU_OFFSET+1+2*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);
}

void draw_resistance_control_menu(int resistance_reference, int resistance_cable){
	SSD1306_GotoXY (6,0);
	SSD1306_Puts("Cable resist.", &Font_7x10, 1);

	char buf[20];
	sprintf(buf, "%d mOhm ref.  ", resistance_reference);
	SSD1306_GotoXY (6, MENU_OFFSET+1);
	SSD1306_Puts (buf, &Font_7x10, 1);

	char str[20];
    sprintf(str, "%d mOhm act.  ", resistance_cable);
    SSD1306_GotoXY(6, MENU_OFFSET+1+2*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);
}

void graph_builder(int value, int lower_bound, int upper_bound){
	//int prev_point = heights[0];
	for(int i = 1; i < XN; ++i){
		//SSD1306_DrawLine(X_STEP*(i-1),HEIGHT-prev_point, X_STEP*i, HEIGHT-heights[i], 1);
		SSD1306_DrawPixel(X_STEP*i, HEIGHT - heights[i-1] - 3, 0);
		SSD1306_DrawPixel(X_STEP*i, HEIGHT - heights[i] - 3, 1);
		//prev_point=heights[i];
	}
	// Put the new y-coordinate in the ring buffer with the rightmost y.
	int a = lower_bound;
	int b = upper_bound;
	// To map a value from [a, b] to [c, d]
	// f(t)=c+(d-c)*(t-a)/(b-a)
	// Where c = MENU_OFFSET, d = HEIGHT
	int mappedValue = (HEIGHT - MENU_OFFSET - 5)*(value-a)/(b-a);

	if (mappedValue > 255)
		mappedValue = 255;
	else if (mappedValue < 0)
		mappedValue = 0;

	uint8_t y1 = (uint8_t)(mappedValue);
	
	// Shift the rest of the y-coordinates to the left. Use memcpy to copy the memory.
	memcpy(heights, heights + 1, (XN - 1) * sizeof(uint8_t));
	heights[XN - 1] = y1;
}

void draw_graph_builder_menu(int lower_bound, int upper_bound, const graph_t *graphs, int curr_graph){

	SSD1306_GotoXY (6,1);
	char str[18];
	snprintf(str, 18, "%s [%d,%d]        ", graphs[curr_graph].description, lower_bound, upper_bound);
	SSD1306_Puts(str, &Font_7x10, 1);

	// Boundary selectors
	draw_graph_menu_upper_bound_button();
	draw_graph_menu_lower_bound_button();

	graph_builder(*(graphs[curr_graph].value), lower_bound, upper_bound);

	// Reset
	draw_graph_menu_reset_button();
}

void draw_graph_menu_upper_bound_button(){
	SSD1306_DrawCircle(XN-1, MENU_OFFSET, 3, 1);
}

void draw_graph_menu_lower_bound_button(){
	SSD1306_DrawCircle(XN-1, HEIGHT-5, 3, 1);
}

void draw_graph_menu_upper_bound_selected(){
	SSD1306_DrawFilledCircle(XN-1, MENU_OFFSET, 3, 1);
}

void draw_graph_menu_upper_bound_deselect(){
	SSD1306_DrawFilledCircle(XN-1, MENU_OFFSET, 3, 0);
}

void draw_graph_menu_upper_bound_focus(){
	SSD1306_DrawCircle(XN-1, MENU_OFFSET, 5, 1);
}

void draw_graph_menu_lower_bound_selected(){
	SSD1306_DrawFilledCircle(XN-1, HEIGHT-5, 3, 1);
}

void draw_graph_menu_lower_bound_deselect(){
	SSD1306_DrawFilledCircle(XN-1, HEIGHT-5, 3, 0);
}

void draw_graph_menu_lower_bound_focus(){
	SSD1306_DrawCircle(XN-1, HEIGHT-5, 5, 1);
}

void draw_graph_menu_reset_button(){
	SSD1306_GotoXY (6, HEIGHT-12);
	SSD1306_DrawFilledCircle(6+3, HEIGHT-9, 7, 0);
	SSD1306_Puts ("R", &Font_7x10, 1);
}

void draw_graph_menu_reset_focus(){
	SSD1306_DrawCircle(6+3, HEIGHT-9, 7, 1);
}

void draw_graph_menu_data_deselect(){
	SSD1306_DrawRectangle(4, 0, 18, 10 , 0);
}

void draw_graph_menu_data_selected(){
	SSD1306_DrawRectangle(4, 0, 18, 10 , 0);
}

void draw_graph_menu_data_focus(){
	SSD1306_DrawRectangle(4, 0, 18, 10 , 1);
}

void draw_graph_menu_exit_deselect(){
	SSD1306_DrawFilledRectangle(6, MENU_OFFSET-3, 6, 6 ,0);
}

void draw_graph_menu_exit_button(){
	SSD1306_DrawRectangle(6, MENU_OFFSET-3, 6, 6 ,1);
}

void draw_graph_menu_exit_focus(){
	SSD1306_DrawFilledRectangle(6, MENU_OFFSET-3, 6, 6 ,1);
}

void draw_graph_bounds_deselect(){
	SSD1306_DrawCircle(XN-1, MENU_OFFSET, 5, 0);
	SSD1306_DrawCircle(XN-1, HEIGHT-5, 5, 0);
}

void draw_graph_menu_clear_selection(){
	draw_graph_menu_data_deselect();
	draw_graph_menu_exit_deselect();
	draw_graph_bounds_deselect();
	SSD1306_DrawCircle(6+3, HEIGHT-9, 7, 0);
}

void draw_actual_value(uint32_t value){
	SSD1306_GotoXY (0,HEIGHT/2);
	char str[12];
	sprintf(str, " %lu  ", value);
	SSD1306_Puts(str, &Font_7x10, 1);
}

void draw_clear_actual_value(){
	SSD1306_GotoXY (0,HEIGHT/2);
	SSD1306_Puts("          ", &Font_7x10, 1);
}

void draw_qc_voltage(uint32_t voltage){
	SSD1306_GotoXY(80, MENU_OFFSET);
	char str[10];
	sprintf(str, "%lu    ", voltage);
	SSD1306_Puts(str, &Font_7x10, 1);
}
void draw_qc_support(qc_support_t type){
	SSD1306_GotoXY(80, MENU_OFFSET+1+1*STEP);
	switch(type){
		case QC2_PLUS:
			SSD1306_Puts("QC2.0+", &Font_7x10, 1);
			break;
		case QC_NOT_SUPPORTED:
			SSD1306_Puts("NO QC ", &Font_7x10, 1);
			break;
		case QC_UNKNOWN:
			SSD1306_Puts("QC ?  ", &Font_7x10, 1);
			break;
	}
}

void draw_show_check_qc(){
	SSD1306_GotoXY(80, MENU_OFFSET);
	SSD1306_Puts("WAIT ", &Font_7x10, 1);
}

void draw_hide_check_qc(){
	SSD1306_GotoXY(80, MENU_OFFSET);
	SSD1306_Puts("     ", &Font_7x10, 1);
}

void draw_max_params_menu(){
	int radius = 3;
	SSD1306_GotoXY(6,0);
    SSD1306_Puts("Max Params", &Font_7x10, 1);
	SSD1306_GotoXY(6+radius+3, MENU_OFFSET-5);
    SSD1306_Puts("Start", &Font_7x10, 1);
}
void draw_max_params_button(){
	int radius = 3;
	SSD1306_DrawFilledCircle(6, MENU_OFFSET-2, radius, 0);
	SSD1306_DrawCircle(6, MENU_OFFSET-2, radius, 1);
}
void draw_max_params_focus(){
	int radius = 3;
	SSD1306_DrawFilledCircle(6, MENU_OFFSET-2, radius, 1);
}

void draw_max_params_protection(){
	SSD1306_GotoXY(6, MENU_OFFSET-5+4*STEP);
    SSD1306_Puts("PROTECTION ON", &Font_7x10, 1);
}
void draw_max_params_protection_hide(){
	SSD1306_GotoXY(6, MENU_OFFSET-5+4*STEP);
    SSD1306_Puts("            ", &Font_7x10, 1);
}

void draw_max_params_results(uint32_t voltage, uint32_t amperage, uint32_t max_allowed_current){
	int radius = 3;
	SSD1306_GotoXY(6+radius+3, MENU_OFFSET-5);
    SSD1306_Puts("Rerun", &Font_7x10, 1);
	SSD1306_GotoXY(6, MENU_OFFSET-5+1*STEP);
    SSD1306_Puts("Max current:", &Font_7x10, 1);
	char str[15];
    sprintf(str, "%lu/%lu mA   ", amperage, max_allowed_current);
    SSD1306_GotoXY(6, MENU_OFFSET-5+2*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);
    char str2[14];
    sprintf(str2, "%lu mV     ", voltage);
    SSD1306_GotoXY(6, MENU_OFFSET-5+3*STEP);
    SSD1306_Puts(str2, &Font_7x10, 1);
}

void draw_capacity_menu(uint32_t voltage, uint32_t amperage, uint32_t mah, uint32_t mwh){
    char str[14];
    sprintf(str, "%lu mV     ", voltage);
    SSD1306_GotoXY(6, MENU_OFFSET-3+1*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);

    char str2[14];
    sprintf(str2, "%lu mA     ", amperage);
    SSD1306_GotoXY(6, MENU_OFFSET-3+2*STEP);
    SSD1306_Puts(str2, &Font_7x10, 1);

    char str3[14];
    sprintf(str3, "%lu mAh     ", mah);
    SSD1306_GotoXY(6, MENU_OFFSET-3+3*STEP);
    SSD1306_Puts(str3, &Font_7x10, 1);

    char str4[14];
	sprintf(str4, "%lu mWh     ", mwh);
	SSD1306_GotoXY(6, MENU_OFFSET-3+4*STEP);
	SSD1306_Puts(str4, &Font_7x10, 1);
}

void draw_done_capacity_measuring(uint32_t mAh, uint32_t mWh){
	SSD1306_GotoXY (6, MENU_OFFSET-3+1*STEP);
    SSD1306_Puts("Stopped  ", &Font_7x10, 1);

    SSD1306_GotoXY(6, MENU_OFFSET-3+2*STEP);
    SSD1306_Puts("             ", &Font_7x10, 1);

    char str[14];
    sprintf(str, "%lu mAh     ", mAh);
    SSD1306_GotoXY(6, MENU_OFFSET-3+3*STEP);
    SSD1306_Puts(str, &Font_7x10, 1);

    char str1[14];
	sprintf(str1, "%lu mWh     ", mWh);
	SSD1306_GotoXY(6, MENU_OFFSET-3+4*STEP);
	SSD1306_Puts(str1, &Font_7x10, 1);
}

void draw_capacity_header(){
	SSD1306_GotoXY (6,0);
    SSD1306_Puts("Capacity", &Font_7x10, 1);
}