/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include "ina219.h"
#include "draw.h"
#include "fonts.h"
#include "qc.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
extern int adrs_219;
/* ntc vars */
uint32_t Ntc_Tmp = 0;
uint16_t Ntc_R;
/* R1 resistance */
#define NTC_UP_R 46600.0f
/* constants of Steinhart-Hart equation */
#define A 0.0007537836786f
#define B 0.0002356034273f
#define C 0.00000005496876007f

#define MAX_ALLOWED_TEMPERATURE 60
#define LOW_VOLTAGE 3200

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_ALLOWED_LOAD 3200
#define GRAPHS_N 4
#define MILLIVOLTAGE_LOWEST_BOUND 2000
#define MILLIVOLTAGE_HIGHEST_BOUND 15000
#define MILLIVOLTAGE_MINIMUM_ALLOWED 3900
#define COOLING_ENABLE_MILLIWATT 10000
#define COOLING_DISABLE_MILLIWATT 5000
#define COOLING_ENABLE_TEMPERATURE 30
#define COOLING_DISABLE_TEMPERATURE 24
#define MAX_UINT32 65532
#define REFERENCE_CABLE_RESISTANCE 228//81

typedef enum state_t {
    MAIN_MENU,
    QC,
    POWER,
    CURRENT_CONTROL,
    GRAPHS,
	TEST_MAX_PARAMS,
	TEST_RESISTANCE,
	TEST_CAPACITY


} state_t;

typedef enum qc_menu_t{
    EXIT_TO_MAIN_MENU_FROM_QC,
    SET_5V,
    SET_9V,
    SET_12V,
    CONTINUOUS_MODE_UP,
    CONTINUOUS_MODE_DOWN
} qc_menu_t;
typedef enum graph_toggle_t{
	EXIT_TO_MAIN_MENU_FROM_GRAPH,
    GRAPH_TOGGLE_UPPER_BOUND,
    GRAPH_TOGGLE_LOWER_BOUND,
    GRAPH_TOGGLE_RESET,
    GRAPH_TOGGLE_DATA
} graph_toggle_t;
typedef enum power_t{
	EXIT_TO_MAIN_MENU_FROM_POWER
} power_t;
typedef enum current_control_t{
	EXIT_TO_MAIN_MENU_FROM_CURRENT_CONTROL
} current_control_t;
typedef enum max_params_t{
	EXIT_TO_MAIN_MENU_FROM_MAX_PARAMS,
    START_MAX_PARAMS_TEST
} max_params_t;
typedef enum resistance_control_t{
	EXIT_TO_MAIN_MENU_FROM_RESISTANCE,
    SAVE_REFERENCE_RESISTANCE,
    GET_CABLE_RESISTANCE
} resistance_control_t;
typedef enum capacity_control_t{
	EXIT_TO_MAIN_MENU_FROM_CAPACITY
} capacity_control_t;

typedef struct OverFlow {
	int overflow_size;
} OverFlow;



volatile int btn_state = 1;
volatile int can_be_pressed = 1;
volatile uint32_t move = 0;
volatile uint32_t move_prev = 0;
volatile state_t state = MAIN_MENU;
volatile graph_toggle_t graph_toggle_state = EXIT_TO_MAIN_MENU_FROM_GRAPH;
volatile int is_drawn = 0;
volatile int device_available = 0;


uint32_t ina_curr = 0;
uint32_t ina_vol = 0;
float ina_vol_float = 0;
uint32_t ina_pwr = 0;
uint8_t ina_cnt = 0;
uint8_t channel = 0;
float mAh = 0;
float mWh = 0;
uint8_t start_measuring_capacity = 0;

uint8_t _cooling = 0;

uint32_t tim3_prev_cnt=0;
int amperage_load=0;
int active_load=0;
int start_load=0;

int graph_upper_bound=3200;
int graph_lower_bound=0;
page_t page = PAGE_1;

uint32_t board_temperature = 0;

qc_support_t has_qc_support = QC_UNKNOWN;
uint8_t qc_execure_check = 0;
int continuous_mode = 0;

int start_max_params_test = 0;

static const graph_t milliVoltage={
        &ina_vol,
        "mV",
        0,
        7500
};
static const graph_t milliAmperage={
        &ina_curr,
        "mA",
        0,
        5000
};
static const graph_t milliWattage= { 
        &ina_pwr,
        "mW",
        0,
        36000
};
static const graph_t temperature={
        &board_temperature,
        "TC",
        20,
        75
};

const graph_t graphs[GRAPHS_N] = {milliVoltage, milliAmperage, milliWattage, temperature};

int curr_graph = 0;

void reset_graph_bounds(){
    graph_upper_bound = graphs[curr_graph].upper_bound;
    graph_lower_bound = graphs[curr_graph].lower_bound;
}

int get_encoder_rotation(){
    unsigned int tim3_cnt = (TIM3->CNT)>>2;
    int diff;
    if (tim3_cnt > tim3_prev_cnt)
        diff = 1;
    else if (tim3_cnt == tim3_prev_cnt)
        diff = 0;
    else
        diff = -1;
    tim3_prev_cnt = tim3_cnt;
    return diff;
}
// Overflow methods
//move = (TIM3->CNT>>2)%1;
uint32_t getMove(uint32_t scale) {
	uint32_t count = ((TIM3->CNT));
	uint32_t move;
	if (count == MAX_UINT32) {
		(TIM3->CNT) = scale<<8;
		move = scale-1;
	}
	else {

		move = (count>>2)%scale;
	}
	return move;

}
// PID controller parameters
double kp = 0.05; // 0.05
double ki = 0.5; // 0.5
double kd = 0.1; // 0.1

// PID controller state
double integral = 0;
double previous_error = 0;

int pid_controller(uint32_t actual_voltage, uint32_t desired_voltage) {
    // Calculate error
    double error = desired_voltage + 0.0 - actual_voltage;
    // Update integral
    integral += error;
    // Calculate derivative
    double derivative = error - previous_error;
    // Save error for next iteration
    previous_error = error;
    // Calculate output
    double output = kp*error + ki*integral + kd*derivative;
    // Saturate output
    if (output < 0) {
        output = 0;
    } else if (output >= MAX_ALLOWED_LOAD) {
        output = MAX_ALLOWED_LOAD - 1;
    }
    // Convert output to uint32_t and return
    return output;
}

void measure_capacity(){
    mAh += ina_curr/3600.0;
	mWh += ina_vol*ina_curr/3600000.0;
}


void electrical_load(int diff){

    if (!active_load){
        amperage_load = 0;
        active_load = 1;
    }

    if(amperage_load + diff < 0){
        amperage_load = 0;
        diff = 0;
    }

    if(amperage_load + diff >= MAX_ALLOWED_LOAD){
        amperage_load = MAX_ALLOWED_LOAD;
        diff = 0;
    }

    amperage_load += diff;

    // 5v
    // 500 - 113 mA
    // 1000 - 220 mA
    // 2000 - 440 mA
    // 5000 - 1030 mA
    // 6000 - 1170 mA
    // 10000 - 1470 mA

    // !!!!!!! ATTENTION !!!!!!!!
    // find values for QC, using these values will cause damage to the device
    // !!!!!!! ATTENTION !!!!!!!!
    int adjustment = pid_controller(ina_curr, amperage_load);
    TIM2->CCR1 = adjustment;
    HAL_Delay(3);
}

void read_circut_parameters(){
    adrs_219 = 0x40;
    ina_curr = getCurrent_mA();
    ina_vol = getBusVoltage_V() * 1000;
    ina_vol_float = getBusVoltage_V();
    ina_pwr = getPower_mW();
}

void emergency_shutdown(){
    TIM2->CCR1 = 0;
    amperage_load = 0;
    active_load = 0;
    HAL_GPIO_WritePin(GPIOB, COOLER_Pin, GPIO_PIN_SET);
    Set_5V();
}

void start_cooling(){
    if (!_cooling){
        HAL_GPIO_WritePin(GPIOB, COOLER_Pin, GPIO_PIN_SET);
        _cooling = 1;
    }
}

void stop_cooling(){
    if(_cooling){
        HAL_GPIO_WritePin(GPIOB, COOLER_Pin, GPIO_PIN_RESET);
        _cooling = 0;
    }
}

volatile int last = -150;
void check_temperature(){
	const int curr = HAL_GetTick();
	if (curr-last<300){return;}
	last = curr;

	/* get adc value */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t Ntc_Temp_ADC = 4095.0-HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);

	/* calculate ntc resistance */
	Ntc_R = ((NTC_UP_R)/((4095.0/Ntc_Temp_ADC) - 1));
	/* calculate temperature */
	float Ntc_Ln = log(Ntc_R);
	Ntc_Tmp = (1.0/(A + B*Ntc_Ln + C*Ntc_Ln*Ntc_Ln*Ntc_Ln)) - 273.15;

    // draw_temperature(Ntc_Tmp);
    board_temperature = Ntc_Tmp;
}

int board_protection(){
    read_circut_parameters();
    check_temperature();
    int protection = 0;
    if(ina_vol < MILLIVOLTAGE_LOWEST_BOUND){
        device_available = 0;
        qc_execure_check = 0;
        has_qc_support = QC_UNKNOWN;
    }
    else if(!device_available){
        device_available = 1;
        qc_execure_check = 1;
        amperage_load = 0;
	}

    if (board_temperature >= MAX_ALLOWED_TEMPERATURE 
            || ina_vol > MILLIVOLTAGE_HIGHEST_BOUND) {
        emergency_shutdown();
        protection=1;
    }
    else if (board_temperature >= COOLING_ENABLE_TEMPERATURE 
            || ina_pwr >= COOLING_ENABLE_MILLIWATT) {
        start_cooling();
	}
    else if (board_temperature < COOLING_DISABLE_TEMPERATURE 
            && board_temperature < COOLING_DISABLE_MILLIWATT){
        stop_cooling();
    } 
    return protection;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Prevent unused argument(s) compilation warning */

    if(htim == &htim1){
        //HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
        can_be_pressed = 1;
        HAL_TIM_Base_Stop_IT(&htim1);
    }
}

int resistance_reference = 9999;
int resistance_cable = 9999;

int get_resistance(){
    // set voltage to 5 volts
    Set_5V();

    // set current to about 1000 mA
    TIM2->CCR1 = 500;
    HAL_Delay(1000);

    // read current
    board_protection();

    int resistance = (ina_curr/ina_vol_float)*10; //(ina_vol_float / (ina_curr / 1000.0))*1000;

    amperage_load = 0;
    TIM2->CCR1 = 0;

    return resistance;
}

int click = 0;
void on_button_clicked(){
    switch (state){
        case MAIN_MENU:
            state=move+1;
            draw_fill(0);
            switch(move+1)
            {
                case QC:
                    TIM2->CCR1 = 0;
                    amperage_load = 0;
                    break;
                case GRAPHS:
                    reset_graph_bounds();
                    break;
                case CURRENT_CONTROL:
                    reset_graph_bounds();
                    start_load = 1;
                    break;
                case TEST_CAPACITY:
                    start_measuring_capacity = 1;
                    break;
            }
            move=0;
            break;

        case QC:
			
            switch (move){
                case EXIT_TO_MAIN_MENU_FROM_QC:
                    state=MAIN_MENU;
                    draw_clear();
                    move=QC-1;
                    break;

                case SET_5V:
                    Set_5V();
                    break;

                case SET_9V:
                    Set_9V();
                    break;

                case SET_12V:
                    Set_12V();
                    break;

                case CONTINUOUS_MODE_UP:
                    continuous_mode = 1;
                    break;

                case CONTINUOUS_MODE_DOWN:
                    continuous_mode = -1;
                    break;
            }
            break;

        case POWER:
            switch (move){
                case EXIT_TO_MAIN_MENU_FROM_POWER:
                    state=MAIN_MENU;
                    draw_fill (0);
                    move=0;
                    break;
            }
            break;

        case CURRENT_CONTROL:
            switch (move){
                case EXIT_TO_MAIN_MENU_FROM_CURRENT_CONTROL:
                    state=MAIN_MENU;
                    draw_fill (0);
                    move=0;
                    active_load=0;
                    break;
            }
            break;

        case GRAPHS:
            if(graph_toggle_state == EXIT_TO_MAIN_MENU_FROM_GRAPH){
                if (move == EXIT_TO_MAIN_MENU_FROM_GRAPH){
                    state=move;
                    draw_fill (0);
                    move=0;
                    break;
                }

                draw_graph_menu_clear_selection();

                switch(move){
                    case GRAPH_TOGGLE_UPPER_BOUND:
                        draw_graph_menu_upper_bound_selected();
                        break;
                    case GRAPH_TOGGLE_LOWER_BOUND:
                        draw_graph_menu_lower_bound_selected();
                        break;
                    case GRAPH_TOGGLE_RESET:
                        reset_graph_bounds();
                        move=0;
                        break;
                    case GRAPH_TOGGLE_DATA:
                        reset_graph_bounds();
                        break;
                }

                graph_toggle_state = move;
            }
            else{
                graph_toggle_state = EXIT_TO_MAIN_MENU_FROM_GRAPH;
                draw_graph_menu_upper_bound_deselect();
                draw_graph_menu_lower_bound_deselect();
                draw_graph_menu_upper_bound_button();
                draw_graph_menu_lower_bound_button();
            }
            break;
        	case TEST_MAX_PARAMS:
        		switch (move){
					case EXIT_TO_MAIN_MENU_FROM_MAX_PARAMS:
						state=MAIN_MENU;
						draw_fill (0);
						move=4;
						break;
                    case START_MAX_PARAMS_TEST:
                        start_max_params_test = 1;
                        break;
					default:
						break;
				}
        		break;
			case TEST_RESISTANCE:
				switch (move){
					case EXIT_TO_MAIN_MENU_FROM_RESISTANCE:
						state=MAIN_MENU;
						draw_fill (0);
						move=4;
						break;
                    case SAVE_REFERENCE_RESISTANCE:
                    click = 1;
                        break;
                    case GET_CABLE_RESISTANCE:
                        // resistance_cable = get_resistance() - resistance_reference;
                        click = 2;
                        break;
				}
				break;
			case TEST_CAPACITY:
				switch (move){
						case EXIT_TO_MAIN_MENU_FROM_CAPACITY:
							state=MAIN_MENU;
							draw_fill (0);
							move=4;
							break;
					}
				break;
		default:
			break;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin==KEY_Pin) {
        if (can_be_pressed) {
            can_be_pressed = 0;
            btn_state = !btn_state;
            is_drawn = 0;

            on_button_clicked();

            HAL_TIM_Base_Start_IT(&htim1);
        }
    }
}

void setup(){
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    draw_init();
    draw_clear();
    adrs_219 = 0x40;
    setCalibration_32V_custom();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->CCR1 = 0;
}

void loop(){
    move_prev = move;
    board_protection();

    switch(state){

        case MAIN_MENU:
            move = getMove(7);
            if (4<=move&&move<7) {
				// draw page 2
            	if (move_prev < 4) {
                	page = PAGE_2;
                	is_drawn = 0;
            	}

			}
            else {
            	// draw page 1
            	if (4<=move_prev&&move_prev<7) {
                	page = PAGE_1;
                	is_drawn = 0;
            	}
            }
            if (!is_drawn) {
            	draw_main_menu(page);
            }
            draw_main_menu_selection(move, move_prev, page);
            break;

        case QC:
            move = getMove(6);
            if (continuous_mode != 0){
                ContinuousMode();

                for(int i = 0; i < 5; ++i){
                    if (continuous_mode == 1) IncVoltage();
                    else if (continuous_mode == -1) DecVoltage();
                    HAL_Delay(50);
                }
                continuous_mode = 0;
            }

            if (!is_drawn) {
                draw_qc_menu();
                is_drawn=1;
            }
          
		  	if(move_prev != EXIT_TO_MAIN_MENU_FROM_QC){
				draw_qc_menu_deselect(move_prev);
			}
			if (move == EXIT_TO_MAIN_MENU_FROM_QC){
				draw_exit_focus();
			}
            else{
				draw_qc_menu_focus(move);
				draw_exit_button();
			}

            draw_qc_voltage(ina_vol);
            draw_qc_support(has_qc_support);

            if (qc_execure_check){
                draw_show_check_qc();
                draw_update_screen();
                HAL_Delay(4000);
                has_qc_support = HasQCSupport();
                draw_hide_check_qc();
                qc_execure_check = 0;
            }
			
            break;

        case POWER:
            move = getMove(1);
            if (!is_drawn) {
                is_drawn=1;
            }else{
                HAL_Delay(500);
            }
            //read_circut_parameters();
            draw_power_menu(ina_vol, ina_curr, ina_pwr);
			if(move == EXIT_TO_MAIN_MENU_FROM_POWER){
				draw_exit_focus();
			}
            else{
				draw_exit_button();
			}
            break;

        case CURRENT_CONTROL:{
            int diff = 50 * get_encoder_rotation();
            if(!start_load){
                diff = 0;
                start_load = 1;
            }
            electrical_load(diff);
            curr_graph = 1;
            draw_current_control_menu(amperage_load, ina_curr);
            draw_graph_builder_menu(graph_lower_bound, graph_upper_bound, graphs, curr_graph);
            break;
        }
        case GRAPHS:
        	move = getMove(5);
            draw_graph_builder_menu(graph_lower_bound, graph_upper_bound, graphs, curr_graph);
            HAL_Delay(50);
            if(move_prev == GRAPH_TOGGLE_DATA && move != GRAPH_TOGGLE_DATA){
                draw_clear_actual_value();
            }
            int delta = 0;
            int multiplier = (graphs[curr_graph].upper_bound - graphs[curr_graph].lower_bound) / 50;
            switch (graph_toggle_state){
                case EXIT_TO_MAIN_MENU_FROM_GRAPH:

                    draw_graph_menu_clear_selection();

                    draw_graph_menu_exit_button();

                    switch(move){
                        case EXIT_TO_MAIN_MENU_FROM_GRAPH:
                            draw_graph_menu_exit_focus();
                            break;
                        case GRAPH_TOGGLE_UPPER_BOUND:
                            draw_graph_menu_upper_bound_focus();
                            break;
                        case GRAPH_TOGGLE_LOWER_BOUND:
                            draw_graph_menu_lower_bound_focus();
                            break;
                        case GRAPH_TOGGLE_RESET:
                            draw_graph_menu_reset_focus();
                            break;
                        case GRAPH_TOGGLE_DATA:
                            draw_actual_value(*graphs[curr_graph].value);
                            draw_graph_menu_data_focus();
                            break;
                    }
                    break;
                case GRAPH_TOGGLE_UPPER_BOUND:
                    delta = get_encoder_rotation() * multiplier;
                    graph_upper_bound += delta;
                    if (graph_upper_bound <= graph_lower_bound){
                        graph_upper_bound = graph_lower_bound + 1;
                    }
                    break;
                case GRAPH_TOGGLE_LOWER_BOUND:
                    delta = get_encoder_rotation() * multiplier;
                    graph_lower_bound += delta;
                    if (graph_upper_bound <= graph_lower_bound){
                        graph_lower_bound = graph_upper_bound - 1;
                    }
                    break;
                case GRAPH_TOGGLE_DATA:
                    delta = get_encoder_rotation();
                    int g = curr_graph + delta;
                    if (g < 0)
                        g = 0;
                    else if (g >= GRAPHS_N)
                        g = GRAPHS_N - 1;
                    curr_graph = g;
                    reset_graph_bounds();
                    break;
				case GRAPH_TOGGLE_RESET:
					break;
            }
            break;

        case TEST_MAX_PARAMS:
            move = getMove(2);
            if(!is_drawn){
                draw_max_params_menu(); 
                is_drawn=1;  
            }
            draw_exit_button();
            draw_max_params_button();

            switch(move){
                case EXIT_TO_MAIN_MENU_FROM_MAX_PARAMS:
                    draw_exit_focus();
                    break;
                case START_MAX_PARAMS_TEST:{
                    draw_max_params_focus();
                    if(!start_max_params_test) break;

                    draw_max_params_protection_hide();
                    start_max_params_test = 0;
                    uint32_t max_params_voltage = 0;
                    uint32_t max_params_amperage = 0;
                    int protection = 0;
                    for(int i = 0; i < MAX_ALLOWED_LOAD/100+1; ++i){
                        // Increment load with 100 mA
                        electrical_load(100);
                        protection = board_protection();
                        if (protection){
                            break;
                        }
                        max_params_voltage = ina_vol;
                        max_params_amperage = ina_curr;
                        // Bound the decresing voltage
                        if(ina_vol < MILLIVOLTAGE_MINIMUM_ALLOWED){
                            break;
                        }

                        HAL_Delay(100);
                    }
                    if(protection) draw_max_params_protection();
                    draw_max_params_results(max_params_voltage, max_params_amperage, MAX_ALLOWED_LOAD);  
                    // Return to 0 mA
                    amperage_load = 0;
                    // Repeat to stabilize
                    for(int j = 0; j < 10; ++j){
                        electrical_load(0);
                        HAL_Delay(100);
                    }
                }
                break;
            }
            break;
        case TEST_RESISTANCE:
            move = ((TIM3->CNT)>>2)%3;
            draw_exit_button();
            draw_graph_bounds_deselect();
            draw_graph_menu_upper_bound_button();
            draw_graph_menu_lower_bound_button();

            switch(click){
                case 0:
                    break;
                case 1:
                    resistance_reference = get_resistance() - REFERENCE_CABLE_RESISTANCE;
                    click = 0;
                    break;
                case 2:
                    resistance_cable = get_resistance() - resistance_reference;
                    click = 0;
                    break;
            }

            switch(move){
                case EXIT_TO_MAIN_MENU_FROM_RESISTANCE:
                    draw_exit_focus();
                    break;
                case SAVE_REFERENCE_RESISTANCE:
                    draw_graph_menu_upper_bound_focus();
                    break;
                case GET_CABLE_RESISTANCE:
                    draw_graph_menu_lower_bound_focus();
                    break;
            }

            

            draw_resistance_control_menu(resistance_reference, resistance_cable);

            break;
        case TEST_CAPACITY:
            move = getMove(1);
            if(move == EXIT_TO_MAIN_MENU_FROM_CAPACITY){
                draw_exit_focus();
            }
            else{
                draw_exit_button();
            }
            draw_capacity_header();
            if(start_measuring_capacity){
                start_measuring_capacity = 0;
                HAL_Delay(250);
            }
            if (ina_vol > LOW_VOLTAGE){
                measure_capacity();
                draw_capacity_menu(ina_vol, ina_curr, mAh, mWh);
                HAL_Delay(1000);
            }
            else{
                draw_done_capacity_measuring(mAh, mWh);
            }
            break;
        default:
            break;
    }


    draw_update_screen();
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  setup();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
    loop();
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DP_H_Pin|DP_L_Pin|DM_H_Pin|DM_L_Pin
                          |COOLER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DP_H_Pin DP_L_Pin DM_H_Pin DM_L_Pin
                           COOLER_Pin */
  GPIO_InitStruct.Pin = DP_H_Pin|DP_L_Pin|DM_H_Pin|DM_L_Pin
                          |COOLER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
