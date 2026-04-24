/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "Q_table.h"


/* ========================== TUNING PARAMETERS ========================== */
#define BASE_SPEED_RL   530
#define TURN_SPEED_RL   1000
#define MAX_SPEED_RL    1000
#define IR_THRESHOLD    2500

int last_action = 2;
int stop_occurrence = 0;
int black =0;
char detectedColor = 'W';

int box_count = 0;
bool searching_third_box = false;
bool junction_turn_done = false;
/* ========================== BOX + COLOR DEFINES ========================== */
#define SENSOR_OUT_PIN   GPIO_PIN_6
#define SENSOR_OUT_PORT  GPIOA

#define LED_ON_STATE     GPIO_PIN_RESET
#define LED_OFF_STATE    GPIO_PIN_SET

/* ========================== GLOBAL VARIABLES ========================== */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

uint16_t adc_buffer[5];
int stop_counter = 0;

int turn =0;
/* ================= WHITE LINE TURN CONTROL ================= */
bool has_turned_left = false;
uint32_t turnZoneStart = 0;
#define TURN_ZONE_TIMEOUT 50

/* ---- Box globals ---- */
uint32_t redFreq = 0, greenFreq = 0, blueFreq = 0;
bool isCarryingBox = false;

/* ========================== STATE ========================== */
uint32_t transition_start_time = 0;

typedef enum {
    MODE_WHITE_LINE = 0,
    MODE_BLACK_LINE = 1,
    MODE_FINISHED   = 2
} RobotMode;

RobotMode current_mode = MODE_WHITE_LINE;

/* ========================== PROTOTYPES ========================== */
void SystemClock_Config(void);
void Setup_Robot_Manually(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void Error_Handler(void);

uint8_t Get_State(bool find_white);
int Choose_Action(uint8_t state);
void Perform_Action(int action);
void Set_Motor_Speed(int left_pwm, int right_pwm);

void Follow_White_Line(void);
void Follow_Black_Line(void);

/* ---- Box Prototypes ---- */
void Process_Box_Detection(void);
uint32_t Get_Frequency(void);
void Set_LED_Color(char color);
void Activate_Magnet_Sequence(void);

/* ========================== MAIN ========================== */
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Box_Init();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_Delay(2000);

    MX_DMA_Init();
    MX_ADC1_Init();
    Setup_Robot_Manually();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 5);

    while (1)
    {
        /* -------- BOX PICKUP (ISOLATED) -------- */
        Process_Box_Detection();

        /* -------- LINE FOLLOW -------- */
        if (current_mode == MODE_WHITE_LINE)
            Follow_White_Line();
        else if (current_mode == MODE_BLACK_LINE)
            Follow_Black_Line();
        if (current_mode == MODE_FINISHED) {
            Set_Motor_Speed(0, 0);
            HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);

            // Blink sequence: 0.5 Hz (2s period) for 5 seconds
            // 5 seconds / 2 seconds per blink = 2.5 blinks
            for (int i = 0; i < 3; i++) {
                // LED ON (1 second)
                Set_LED_Color('W'); // White or choose 'R', 'G', 'B'
                HAL_Delay(1000);

                // If we've hit 5 seconds total, break before the final 1s OFF
                if (i == 2) break;

                // LED OFF (1 second)
                HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);
                HAL_Delay(1000);
            }

            // Ensure it stays off after the sequence
            HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);

            // Stay in an infinite loop to prevent the sequence from repeating
            while(1) {
                Set_Motor_Speed(0, 0);
            }
        }

        HAL_Delay(2);
    }
}

/* ========================== BOX LOGIC ========================== */
void Process_Box_Detection(void)
{
    static bool boxAlreadyDetected = false;

    if (HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) == GPIO_PIN_RESET)
    {
        if (!boxAlreadyDetected)
        {
            boxAlreadyDetected = true;
            box_count++;
            // Move forward for 1 second after detecting box
            Set_Motor_Speed(400, 400);
            HAL_Delay(150);

            // Stop robot
            Set_Motor_Speed(0, 0);
            HAL_Delay(300);

            // ----- COLOR DETECTION -----
            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
            HAL_Delay(50);
            redFreq = Get_Frequency();

            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            greenFreq = Get_Frequency();

            HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            blueFreq = Get_Frequency();

            if (redFreq < greenFreq && redFreq < blueFreq)
                Set_LED_Color('R');
            else if (greenFreq < redFreq && greenFreq < blueFreq)
                Set_LED_Color('G');
            else if (blueFreq < redFreq && blueFreq < greenFreq)
                Set_LED_Color('B');
            else
                Set_LED_Color('W');

            // -------- FIRST TWO BOXES --------
            if (box_count <= 2)
            {
                HAL_Delay(800);

                if (detectedColor == 'R')
                {
                    // ---- EARLY PICKUP ----
                    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_SET);
                    HAL_Delay(800);

                    // 180 degree turn
                    Set_Motor_Speed(-600, 600);
                    HAL_Delay(750);

                    // Disable junction logic
                    searching_third_box = false;
                    junction_turn_done = true;

                    isCarryingBox = true;

                    return;   // Exit function
                }

                // If not RED → just show and continue
                Set_LED_Color('W');

                if (box_count == 2)
                    searching_third_box = true;   // Keep your old logic
            }

            // -------- THIRD BOX --------
            else if (box_count == 3)
            {
                // Turn ON electromagnet
                HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_SET);
                HAL_Delay(800);

                // Stop briefly for stability
                Set_Motor_Speed(-500, 500);
                HAL_Delay(300);
            }
        }
    }
    else
    {
        boxAlreadyDetected = false;
    }
}

void Activate_Magnet_Sequence(void)
{
    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_SET);
    HAL_Delay(800);

    Set_Motor_Speed(-500, 500);
    HAL_Delay(850);

    Set_Motor_Speed(0, 0);
    HAL_Delay(200);
}

uint32_t Get_Frequency(void)
{
    uint32_t count = 0, timeout = 50000;

    while (HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN))
        if (--timeout == 0) return 5000;

    timeout = 50000;
    while (!HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN))
        if (--timeout == 0) return 5000;

    while (HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN)) {
        count++;
        if (count > 10000) break;
    }
    return count;
}

void Set_LED_Color(char color)
{
	detectedColor = color;
    HAL_GPIO_WritePin(GPIOC,
        RED_Pin | GREEN_Pin | BLUE_Pin,
        LED_OFF_STATE);

    if (color == 'R')
        HAL_GPIO_WritePin(GPIOC, RED_Pin, LED_ON_STATE);
    else if (color == 'G')
        HAL_GPIO_WritePin(GPIOC, GREEN_Pin, LED_ON_STATE);
    else if (color == 'B')
        HAL_GPIO_WritePin(GPIOC, BLUE_Pin, LED_ON_STATE);
    else
        HAL_GPIO_WritePin(GPIOC,
            RED_Pin | GREEN_Pin | BLUE_Pin,
            LED_ON_STATE);
}

/* ========================== LINE FOLLOW (UNCHANGED) ========================== */
void Follow_White_Line(void)
{
    uint8_t state = Get_State(true);

    // Detect junction when middle 3 sensors see white (01110 = 14)
    if (state == 14)   // 01110 → middle 3 sensors white
    {
        stop_counter++;

        if (stop_counter > 5)   // debounce
        {
            if (searching_third_box && !junction_turn_done)
            {

                Set_Motor_Speed(400, 400);
                HAL_Delay(500);


                Set_Motor_Speed(0, 0);
                HAL_Delay(150);


                Set_Motor_Speed(-500, 500);
                HAL_Delay(750);   // Tune if needed

                junction_turn_done = true;   // Lock so it runs only once
                stop_counter = 0;

                return;
            }

            stop_counter = 0;
        }
    }
    else
    {
        stop_counter = 0;
    }
    // Normal line following
    Perform_Action(Choose_Action(state));
}
void Follow_Black_Line(void) {
    uint8_t state = Get_State(false);
    uint32_t current_time = HAL_GetTick();
    if (detectedColor == 'G') {




    	if (stop_occurrence == 0) {
    	            int black_count = 0;
    	            for (int i = 0; i < 5; i++) {
    	                if (state & (1 << i)) black_count++;
    	            }

    	            if (black_count <= 5 && black_count >= 2) {
    	                stop_counter++;
    	                if (stop_counter > 10) {

    	                    Set_Motor_Speed(BASE_SPEED_RL, BASE_SPEED_RL);   // Stop briefly
    	                    HAL_Delay(300);

    	                    stop_occurrence = 1;
    	                    stop_counter = 0;
    	                }
    	                return;
    	            } else {
    	                stop_counter = 0;
    	            }
    	        }


        // --- 1. RIGHT TURN LOGIC (Occurrence 0) ---
    	else if (stop_occurrence == 1 && (state == 30 || state == 28)) {
            stop_counter++;

            if (stop_counter > 5) {

                Set_Motor_Speed(500, 415); // Right Turn
                HAL_Delay(1200);
                Set_Motor_Speed(400, -400); // Right Turn
                HAL_Delay(500);
                stop_occurrence = 2;
                stop_counter = 0;
                return;
            }
        }

        // --- 2. DETECT STOP LINE, TURN LEFT, THEN CONTINUE (Occurrence 1) ---
        else if (stop_occurrence == 2) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            if (black_count <= 4 && black_count >= 2) {
                stop_counter++;
                if (stop_counter > 10) {
                    Set_Motor_Speed(0, 0);   // Stop briefly
                    current_mode = MODE_FINISHED;
                    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);
					HAL_Delay(1000);
					isCarryingBox = false;
					Set_LED_Color('W');// <--- This kills the whole run
					return;


                    stop_occurrence = 3;
                    stop_counter = 0;
                }
                return;
            } else {
                stop_counter = 0;
            }
        }
        }
    else{


//     --- 1. RIGHT TURN LOGIC (Occurrence 0) ---

    if (stop_occurrence == 0) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            if (black_count <= 5 && black_count >= 2) {
                stop_counter++;
                if (stop_counter > 10) {

                    Set_Motor_Speed(BASE_SPEED_RL, BASE_SPEED_RL);   // Stop briefly
                    HAL_Delay(150);

                    stop_occurrence = 1;
                    stop_counter = 0;
                }
                return;
            } else {
                stop_counter = 0;
            }
        }

    else if (stop_occurrence == 1 && (state == 30 || state == 28)) {
            stop_counter++;
            if (stop_counter > 5) {
                Set_Motor_Speed(0, 420); // Right Turn
                HAL_Delay(500);
                stop_occurrence = 2;
                stop_counter = 0;
                return;
            }
        }


    // --- 2. DETECT STOP LINE, TURN LEFT, THEN CONTINUE (Occurrence 1) ---
    else if (stop_occurrence == 2) {
        int black_count = 0;
        for (int i = 0; i < 5; i++) {
            if (state & (1 << i)) black_count++;
        }

        if (black_count <= 4 && black_count >= 2) {
            stop_counter++;
            if (stop_counter > 10) {
                Set_Motor_Speed(0, 0);   // Stop briefly

                HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);
                HAL_Delay(1000);
                isCarryingBox = false;
                black=1;
                Set_LED_Color('W');
                Set_Motor_Speed(-500, 500); // Pivot Left
                HAL_Delay(900);

                stop_occurrence = 3;
                stop_counter = 0;
            }
            return;
        } else {
            stop_counter = 0;
        }
    }

    // --- 3. TURN RIGHT & START 4s TIMER (Occurrence 2) ---
    else if (stop_occurrence == 3) {
        int black_count = 0;
        for (int i = 0; i < 5; i++) {
            if (state & (1 << i)) black_count++;
        }

        if (black_count >= 3) {
            stop_counter++;
            if (stop_counter > 15) {
                // Perform the Right Turn
                Set_Motor_Speed(800, -100);
                HAL_Delay(475);

                // Record the time and move to the timed-follow state
                transition_start_time = HAL_GetTick();
                stop_occurrence = 4;
            }
            return;
        } else {
            stop_counter = 0;
        }
    }

    // --- 4. FOLLOW BLACK FOR 4 SECONDS THEN SWITCH TO WHITE (Occurrence 3) ---
    else if (stop_occurrence ==4) {
        // Check if 4 seconds (4000ms) have passed
        if (current_time - transition_start_time >= 1200) {
            current_mode = MODE_WHITE_LINE; // Switch back to White Line mode
            stop_occurrence = 0;           // Reset occurrences for next time if needed
            return;
        }
        // While within 4 seconds, just let the bottom Perform_Action handle following
    }
    }

    // Keep following the black line using standard logic
    Perform_Action(Choose_Action(state));
}
/* --------------------------------------------------------------------------
   ROBOT MOVEMENT
   -------------------------------------------------------------------------- */
void Perform_Action(int action)
{
	int fwd, turn, stop = 0;
	if (current_mode == MODE_BLACK_LINE) {
	        fwd  = BASE_SPEED_RL;
	        turn = TURN_SPEED_RL;
	    } else {
	        fwd  = BASE_SPEED_RL;
	        turn = TURN_SPEED_RL;
	    }

    int left = 0, right = 0;

    switch(action) {
        case 0: left = stop; right = turn; break;
        case 1: left = fwd -60; right = fwd + 100; break;
        case 2: left = fwd; right = fwd; break;
        case 3: left = (fwd + 100); right = fwd - 60; break;
        case 4: left = turn ; right = stop; break;
    }

    if (left < 0) left = 0;
    Set_Motor_Speed(left, right);
}

void Set_Motor_Speed(int left_pwm, int right_pwm)
{
    if (left_pwm > MAX_SPEED_RL) left_pwm = MAX_SPEED_RL;
    if (right_pwm > MAX_SPEED_RL) right_pwm = MAX_SPEED_RL;
    if (left_pwm < -MAX_SPEED_RL) left_pwm = -MAX_SPEED_RL;
    if (right_pwm < -MAX_SPEED_RL) right_pwm = -MAX_SPEED_RL;

    if (left_pwm >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -left_pwm);
    }

    if (right_pwm >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, right_pwm);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, -right_pwm);
    }
}

/* --------------------------------------------------------------------------
   SENSORS & LOGIC
   -------------------------------------------------------------------------- */
uint8_t Get_State(bool find_white) {
    uint8_t state = 0;
    for (int i = 0; i < 5; i++) {
        bool detection = false;
        if (find_white) {
            if (adc_buffer[i] > IR_THRESHOLD) detection = true;
        } else {
            if (adc_buffer[i] < IR_THRESHOLD) detection = true;
        }

        if (detection) {
            state |= (1 << (4 - i));
        }
    }
    return state;
}

int Choose_Action(uint8_t state)
{
    // --- 1. STOP OVERRIDE ---
    if (state == 15) return 5;

    // --- 2. ALL BLACK OVERRIDE ---
    if (state == 31) {
        last_action = 2;
        return 2;
    }

    // --- 3. MEMORY HANDLER ---
    if (state == 0) return last_action;

    // --- 4. HARDCODED OVERRIDES ---
    if (state == 28 || state == 24 || state == 16 || state == 30) {
        last_action = 0;
        return 0;
    }

    if (state == 7 || state == 3 || state == 1 || state == 2) {
        last_action = 4;
        return 4;
    }

    // --- 5. STANDARD PROPORTIONAL FOLLOWING ---
    int action = 2; // Default Forward
    if (state == 4)       action = 2;
    else if (state == 6)  action = 3;
    else if (state == 12) action = 1;

    last_action = action;
    return action;
}

/* --------------------------------------------------------------------------
   HARDWARE SETUP
   -------------------------------------------------------------------------- */
void MX_GPIO_Box_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* -------- Electromagnet -------- */
    GPIO_InitStruct.Pin = Electromagnet_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);

    /* -------- RGB LED (Common Anode) -------- */
    GPIO_InitStruct.Pin = RED_Pin | GREEN_Pin | BLUE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);

    /* -------- TCS3200 S0–S3 -------- */
    GPIO_InitStruct.Pin = S0_Pin | S1_Pin | S2_Pin | S3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Set color sensor scaling to 20% */
    HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);

    /* -------- TCS3200 OUT -------- */
    GPIO_InitStruct.Pin = SENSOR_OUT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SENSOR_OUT_PORT, &GPIO_InitStruct);
}

void Setup_Robot_Manually(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP;
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.Period = 999;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.Period = 999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();
}

void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  sConfig.Rank = ADC_REGULAR_RANK_1; sConfig.Channel = ADC_CHANNEL_10; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Rank = ADC_REGULAR_RANK_2; sConfig.Channel = ADC_CHANNEL_11; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Rank = ADC_REGULAR_RANK_3; sConfig.Channel = ADC_CHANNEL_12; HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Rank = ADC_REGULAR_RANK_4; sConfig.Channel = ADC_CHANNEL_8;  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
  sConfig.Rank = ADC_REGULAR_RANK_5; sConfig.Channel = ADC_CHANNEL_9;  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

void MX_DMA_Init(void) {
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_adc1.Instance = DMA1_Channel1;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_CIRCULAR;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void Error_Handler(void) { __disable_irq(); while (1) {} }
