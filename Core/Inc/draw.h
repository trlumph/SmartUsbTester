/*
 * draw.h
 *
 *  Created on: Dec 25, 2022
 *      Author: tymur
 */

#ifndef INC_DRAW_H_
#define INC_DRAW_H_

#include <stdint.h>
#include "qc.h"

typedef struct {
	const uint32_t *value;
	const char description[2];
	const int lower_bound;
	const int upper_bound;
} graph_t;
typedef enum page_t {
	PAGE_1,
	PAGE_2
} page_t;
void draw_init();
void draw_clear();
void draw_fill(int color);
void draw_update_screen();
void draw_exit_focus();
void draw_temperature(uint32_t t);
void draw_exit_button();
void draw_main_menu(page_t page);
void draw_main_menu_page_1();
void draw_main_menu_page_2();
void draw_qc_menu();
void draw_qc_menu_deselect(uint16_t pos);
void draw_qc_menu_focus(uint16_t pos);
void draw_main_menu_selection(uint16_t move, uint16_t previous_move, page_t page);
void draw_power_menu(uint32_t volts, uint32_t amperage, uint32_t power);
void draw_current_control_menu(int amperage_load, int amperage);
void graph_builder(int value, int lower_bound, int upper_bound);
void draw_graph_builder_menu(int lower_bound, int upper_bound, const graph_t *graphs, int curr_graph);
void draw_graph_menu_clear_selection();
void draw_main_menu_selection_page_1(uint16_t move, uint16_t previous_move);
void draw_main_menu_selection_page_2(uint16_t move, uint16_t previous_move);
void draw_graph_menu_upper_bound_button();
void draw_graph_menu_lower_bound_button();
void draw_graph_menu_upper_bound_selected();
void draw_graph_menu_upper_bound_deselect();
void draw_graph_menu_upper_bound_focus();
void draw_graph_menu_lower_bound_selected();
void draw_graph_menu_lower_bound_deselect();
void draw_graph_menu_lower_bound_focus();
void draw_graph_menu_reset_button();
void draw_graph_menu_reset_focus();
void draw_graph_menu_data_deselect();
void draw_graph_menu_data_selected();
void draw_graph_bounds_deselect();
void draw_graph_menu_data_focus();
void draw_graph_menu_exit_deselect();
void draw_graph_menu_exit_button();
void draw_graph_menu_exit_focus();
void draw_actual_value(uint32_t value);
void draw_clear_actual_value();

void draw_qc_support(qc_support_t type);
void draw_qc_voltage(uint32_t voltage);

void draw_show_check_qc();
void draw_hide_check_qc();
void draw_resistance_control_menu(int resistance_reference, int resistance_cable);

void draw_max_params_menu();
void draw_max_params_button();
void draw_max_params_focus();
void draw_max_params_results(uint32_t voltage, uint32_t amperage, uint32_t max_allowed_current);
void draw_max_params_protection();
void draw_max_params_protection_hide();

void draw_capacity_menu(uint32_t voltage, uint32_t amperage, uint32_t mah, uint32_t mwh);
void draw_done_capacity_measuring(uint32_t mAh, uint32_t mWh);
void draw_capacity_header();

#endif /* INC_DRAW_H_ */
