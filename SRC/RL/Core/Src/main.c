/*
 * Team Id: 3450
 * Author List: Manjari M, Tanishk Choudhary, Tharaneeswarar MS	,Viswajit Arunkumar
 * Filename: main.c
 * Theme: CropDrop Bot (CB)
 *
 * Mechanical Design of Crop Drop BOT Done by Viswajit Arunkumar using Fusion 360
 *
 * Components Placement and Wire management Done by Viswajit Arunkumar, Tharaneeswarar MS, Manjari M, Tanishk Choudhary
 *
 * Q-table --> Done by Manjari M
 *
 * Functions:

 *main() --> Made by Tanishk Choudhary
 *
 *Process_Box_Detection() --> Made by Viswajit Arunkumar
 *Activate_Magnet_Sequence(char) --> Made by Viswajit Arunkumar
 *Get_Frequency(void) --> Made by Viswajit Arunkumar
 *Set_LED_Color(char)--> Made by Viswajit Arunkumar
 *
 *Follow_White_Line(void) --> Made by Tharaneeswarar Ms
 *Follow_Black_Line(void) --> Made by Tanishk Choudhary, Tharaneeswarar Ms
 *Perform_Action(int), --> Made by Tanishk Choudhary, Tharaneeswarar Ms
 *
 *Set_Motor_Speed(int,int) --> Made by Manjari M
 *Get_State(bool) --> Made by Manjari M
 *Choose_Action(uint8_t) --> Made by Manjari M
 *
 *MX_GPIO_Box_Init(void),Setup_Robot_Manually(void), SystemClock_Config(void),
 *MX_ADC1_Init(void), MX_DMA_Init(void), Error_Handler(void)
 *
 */

/* Global Variables:
 * last_action, stop_occurrence, black_turn_counter,
 * detected_color, finish_flag,
 * stop_counter, turn_counter,
 * red_frequency, green_frequency, blue_frequency,
 * is_carrying_box, transition_start_time,
 * current_mode, junction_turn_done, searching_third_box
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include "Q_table.h"

/* ========================== TUNING PARAMETERS ========================== */
#define BASE_SPEED_RL 570      // Base forward PWM speed
#define TURN_SPEED_RL 1000     // PWM speed for the outer wheel during hard turns
#define MAX_SPEED_RL 1100      // Absolute maximum PWM cap for motors
#define IR_THRESHOLD 2500      // ADC threshold to distinguish black vs white


/* ========================== BOX + COLOR DEFINES ========================== */
#define SENSOR_OUT_PIN GPIO_PIN_6
#define SENSOR_OUT_PORT GPIOA

#define LED_ON_STATE GPIO_PIN_RESET  // Assuming Common Anode RGB LED (Low = ON)
#define LED_OFF_STATE GPIO_PIN_SET   // Assuming Common Anode RGB LED (High = OFF)

/* ========================== GLOBAL VARIABLES ========================== */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* ---- State & Progression Globals ---- */
bool blue_priority_mode = false;     // Set true when searching specifically for a blue box
bool green_priority_mode = false;    // Set true when searching specifically for a green box
bool white_junction_stopped = false;

// Motor & Pathfinding States
int last_action = 2;        // Stores previous action (0=Left to 4=Right, 2=Straight). Used to recover if line is lost.
int stop_occurrence = 0;    // Tracks the current junction/step in the node-to-node sequence
int black = 0;              // Tracks the number of specific black line features/junctions crossed
int over = 0;               // Flag set to 1 when the final task sequence is completed
int turn = 0;               // Counter for 90-degree turns completed

bool junction_turn_done = false;
bool searching_third_box = false;

uint16_t adc_buffer[5];     // DMA buffer storing raw analog values from the 5 IR sensors
int stop_counter = 0;       // Debounce counter to confirm consistent sensor readings

/* ================= WHITE LINE TURN CONTROL ================= */
bool has_turned_left = false; // Prevents the robot from turning left multiple times at the same junction
uint32_t turnZoneStart = 0;
#define TURN_ZONE_TIMEOUT 50

/* ---- Box globals ---- */
uint32_t redFreq = 0, greenFreq = 0, blueFreq = 0; // Stores raw period counts from TCS3200 sensor
bool isCarryingBox = false;                        // True if electromagnet is active and holding a box
char detectedColor = 'W';

/* ========================== STATE ========================== */
uint32_t transition_start_time = 0; // Timestamp used for timed movements (e.g., driving blind for X ms)

typedef enum {
    MODE_WHITE_LINE = 0, // Following a white line on a black background
    MODE_BLACK_LINE = 1, // Following a black line on a white background
    MODE_FINISHED = 2    // Sequence complete, await final blink
} RobotMode;

RobotMode current_mode = MODE_WHITE_LINE;

/* ========================== PROTOTYPES ========================== */
void SystemClock_Config(void);
void Setup_Robot_Manually(void);
void MX_DMA_Init(void);
void MX_ADC1_Init(void);
void Error_Handler(void);
void MX_GPIO_Box_Init(void);

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
void Activate_Magnet_Sequence(char color);

/* ========================== MAIN ========================== */
int main(void) //Made by Tanishk Choudhary
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Box_Init();

    // Enable clocks and init GPIO for Motor Control / Sensors
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_Delay(2000); // Wait for robot to settle before moving

    MX_DMA_Init();
    MX_ADC1_Init();
    Setup_Robot_Manually();

    // Start continuous ADC reading via DMA into adc_buffer
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

        // Sequence Completion Handle
        if (current_mode == MODE_FINISHED) {
            Set_Motor_Speed(0, 0); // Halt motors
            HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET); // Drop any held payload

            // Blink sequence: 0.5 Hz (2s period) for 5 seconds
            // 5 seconds / 2 seconds per blink = 2.5 blinks
            for (int i = 0; i < 3; i++) {
                Set_LED_Color('W'); // White ON (1 second)
                HAL_Delay(1000);

                // If we've hit 5 seconds total, break before the final 1s OFF
                if (i == 2) break;

                // All LEDs OFF (1 second)
                HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);
                HAL_Delay(1000);
            }

            // Ensure it stays off after the sequence
            HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);

            // Infinite trap loop to freeze robot
            while(1) {
                Set_Motor_Speed(0, 0);
            }
        }

        HAL_Delay(2); // Small loop delay for stability
    }
}

/* ========================== BOX LOGIC ========================== */
void Process_Box_Detection(void) //Made By Viswajit Arunkumar
{
    // If not currently holding a box AND the mechanical limit switch/IR detects a box
    if (!isCarryingBox &&
        HAL_GPIO_ReadPin(Box_detect_GPIO_Port, Box_detect_Pin) == GPIO_PIN_RESET)
    {
        // Ram forward slightly to ensure box is flush with the sensor
        Set_Motor_Speed(350, 350);
        HAL_Delay(200);

        Set_Motor_Speed(0, 0);
        HAL_Delay(300); // Settle time before reading color

        // TCS3200 Color Reading Process
        // Lower frequency count = Higher intensity of that color

        // 1. Red Filter (S2 = LOW, S3 = LOW)
        HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_RESET);
        HAL_Delay(50);
        redFreq = Get_Frequency();

        // 2. Green Filter (S2 = HIGH, S3 = HIGH)
        HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        greenFreq = Get_Frequency();

        // 3. Blue Filter (S2 = LOW, S3 = HIGH)
        HAL_GPIO_WritePin(GPIOB, S2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, S3_Pin, GPIO_PIN_SET);
        HAL_Delay(50);
        blueFreq = Get_Frequency();

        /* -------- COLOR DECISION -------- */

        // RED → Lowest count is red. Always pickup Red first.
        if (redFreq < greenFreq && redFreq < blueFreq)
        {
            Set_LED_Color('R');
            Activate_Magnet_Sequence('R');
            isCarryingBox = true;
        }
        // GREEN → Lowest count is green.
        else if (greenFreq < redFreq && greenFreq < blueFreq)
        {
            Set_LED_Color('G');
            HAL_Delay(1000);

            if (green_priority_mode)
            {
                // We are looking for green, pick it up
                Activate_Magnet_Sequence('G');
                isCarryingBox = true;
                green_priority_mode = false;
            }
            else
            {
            	// Normal behaviour (ignore green box and push past it)
            	Set_LED_Color('W');
            	Set_Motor_Speed(500, 500);
            	HAL_Delay(400);
            }
        }
        // BLUE → Lowest count is blue.
        else if (blueFreq < redFreq && blueFreq < greenFreq)
        {
            Set_LED_Color('B');
            HAL_Delay(1000);

            if (blue_priority_mode)
            {
                // Pickup Blue (ONLY active after red cycle is completed)
                Activate_Magnet_Sequence('B');
                isCarryingBox = true;
                blue_priority_mode = false;   // Reset priority
            }
            else
            {
                // Normal behaviour (ignore blue box and push past it)
                Set_LED_Color('W');
                Set_Motor_Speed(500, 500);
                HAL_Delay(400);
            }
        }
    }
}

void Activate_Magnet_Sequence(char color) //Made By Viswajit Arunkumar
{
    // Engage Electromagnet
    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_SET);
    HAL_Delay(500);

    // Depending on the color picked up, execute a specific mechanical rotation
    if (color == 'B' || color == 'G')
    {
        // 180 degree U-Turn
        Set_Motor_Speed(-500, 500);
        HAL_Delay(960);
    }
    else if (color == 'R')
    {
        // 90 degree Left Turn
        Set_Motor_Speed(-500, 500);
        HAL_Delay(250);
    }

    Set_Motor_Speed(0, 0); // Stop moving after turn
    HAL_Delay(200);
}

/**
 * @brief Measures the pulse width of the TCS3200 sensor output.
 * Counts loop iterations while the pin is HIGH. A lower return value
 * means a higher frequency (brighter light/stronger color match).
 */
uint32_t Get_Frequency(void) //Made By Viswajit Arunkumar
{
    uint32_t count = 0, timeout = 50000;

    // Wait for pin to go LOW (synchronize)
    while (HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN))
        if (--timeout == 0) return 5000; // Return high value on timeout (no color seen)

    // Wait for pin to go HIGH
    timeout = 50000;
    while (!HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN))
        if (--timeout == 0) return 5000;

    // Count how long the pin stays HIGH
    while (HAL_GPIO_ReadPin(SENSOR_OUT_PORT, SENSOR_OUT_PIN)) {
        count++;
        if (count > 10000) break; // Cap the maximum count
    }
    return count;
}

void Set_LED_Color(char color) //Made By Viswajit Arunkumar
{
    detectedColor = color;

    // Turn all off initially (Common Anode -> HIGH = OFF)
    HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_OFF_STATE);

    if (color == 'R')
        HAL_GPIO_WritePin(GPIOC, RED_Pin, LED_ON_STATE);
    else if (color == 'G')
        HAL_GPIO_WritePin(GPIOC, GREEN_Pin, LED_ON_STATE);
    else if (color == 'B')
        HAL_GPIO_WritePin(GPIOC, BLUE_Pin, LED_ON_STATE);
    else // 'W' or unknown -> All ON for White
        HAL_GPIO_WritePin(GPIOC, RED_Pin | GREEN_Pin | BLUE_Pin, LED_ON_STATE);
}

/* ========================== LINE FOLLOW ========================== */

void Follow_White_Line(void) // Made by Tharaneeswarar MS
{
    // Get 5-bit digital mask for White Line (1 = White, 0 = Black)
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
                    HAL_Delay(250);


                    Set_Motor_Speed(0, 0);
                    HAL_Delay(150);


                    Set_Motor_Speed(-600, 400);
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

    static int all_black_counter = 0; // Local counter to debounce the black

    // Check for Final Stop Marker (over flag is set, and center 3 sensors see line: 0x0E = 0b01110)
    if (over == 1 && (state & 0x0E) == 0x0E) {
        Set_Motor_Speed(0, 0);
        current_mode = MODE_FINISHED; // Stop the robot permanently
        return;                       // Exit function
    }

    // ================= DEFAULT WHITE LINE LOGIC =================

    // --- 1. Transition Trigger (All White: 0x1F = 0b11111) ---
    // If all sensors see white, it means we reached the black line zone (inverted background).
    if (state == 0x1F) {
        stop_counter++;
        all_black_counter = 0;

        if (stop_counter > 5) { // Debounce 5 cycles
            current_mode = MODE_BLACK_LINE;
            stop_counter = 0;
            has_turned_left = false; // Reset for the next time we enter this mode
            return;
        }
    }
    // --- 2. All Black Logic (0x00 = 0b00000) ---
    // Runs ONLY ONCE after dropping the first box, hunting for the main track.
    else if (state == 0x00 && black <= 3 && black != 0 && turn <= 2) {
        stop_counter = 0;
        all_black_counter++;

        if (all_black_counter > 25) { // Requires 25 consecutive reads to confirm solid black
            Set_Motor_Speed(-500, 500); // Hard Left Pivot
            HAL_Delay(100);             // Tune this delay for exactly 90 degrees

            if (black == 1) {
                over = 1; // Mark system as ready to finish
            }

            has_turned_left = true; // Lockout flag so it only happens once
            turn += 1;
            all_black_counter = 0;
            return;
        }
    }
    // --- Reset counters if it's standard line following ---
    else {
        stop_counter = 0;
        all_black_counter = 0;
    }

    // --- 3. Standard Following ---
    // Pass the state mask to the pathing logic
    Perform_Action(Choose_Action(state));
}

void Follow_Black_Line(void) //Made by Tanishk Choudhary, Tharaneeswarar MS
{
    // Get 5-bit digital mask for Black Line (1 = Black, 0 = White)
    uint8_t state = Get_State(false);
    uint32_t current_time = HAL_GetTick();

    // ================= 1. GREEN BOX LOGIC =================
    if (detectedColor == 'G') {

            // --- 0. First Junction (Go Straight) ---
            if (stop_occurrence == 0) {
                int black_count = 0; // Count how many sensors see black
                for (int i = 0; i < 5; i++) {
                    if (state & (1 << i)) black_count++;
                }

                // If 2 to 5 sensors see black, we hit a junction
                if (black_count <= 5 && black_count >= 2) {
                    stop_counter++;
                    if (stop_counter > 10) {
                        Set_Motor_Speed(BASE_SPEED_RL, BASE_SPEED_RL);   // Go straight through it
                        HAL_Delay(300);

                        stop_occurrence = 1; // Move to next node logic
                        stop_counter = 0;
                    }
                    return;
                } else {
                    stop_counter = 0;
                }
            }

            // --- 1. Right Turn to Zone D ---
            // State 30 (0b11110) or 28 (0b11100) indicates heavy black on the right sensors
            else if (stop_occurrence == 1 && (state == 30 || state == 28)) {
                stop_counter++;

                if (stop_counter > 5) { // Debounce
                    // Custom Right Turn Profile
                    Set_Motor_Speed(500, 415);
                    HAL_Delay(1200);
                    Set_Motor_Speed(400, -200);
                    HAL_Delay(500);

                    stop_occurrence = 2; // Ready to drop box
                    stop_counter = 0;
                    return;
                }
            }

            // --- 2. Drop Green Box and U-Turn ---
            else if (stop_occurrence == 2)
            {
                int black_count = 0;
                for (int i = 0; i < 5; i++) {
                    if (state & (1 << i)) black_count++;
                }

                // If Center sensor (0x04 = 0b00100) sees black AND we are at a junction (2-4 sensors)
                if ((state & 0x04) && (black_count <= 4 && black_count >= 2)) {
                    stop_counter++;
                    if (stop_counter > 2) {

                        Set_Motor_Speed(0, 0); // STOP

                        // DROP GREEN
                        HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);
                        HAL_Delay(1000);
                        isCarryingBox = false;

                        Set_LED_Color('W');
                        detectedColor = 'G'; // FIX: Force state to stay Green to complete return trip!

                        // 180-Degree U-TURN
                        Set_Motor_Speed(-500, 500);
                        HAL_Delay(950);

                        stop_occurrence = 3;
                        stop_counter = 0;
                        return;
                    }
                } else {
                    stop_counter = 0;
                }
            }

            // --- 3. Left Turn at Center Black Box to go back to Start (A) ---
            else if (stop_occurrence == 3)
            {
                // Wait for right-side heavy junction (30=0b11110, 28=0b11100) or all black (31=0b11111)
                if (state == 30 || state == 28 || state == 31) {
                    stop_counter++;
                    if (stop_counter > 3) {

                        Set_Motor_Speed(0, 0);
                        HAL_Delay(150);

                        // Perform 90-Degree Left Turn
                        Set_Motor_Speed(-400, 400);
                        HAL_Delay(400);
                        black = 1;
                        turn = 0;

                        // Start timer for blind driving
                        transition_start_time = HAL_GetTick();

                        stop_occurrence = 4;
                        stop_counter = 0;
                        return;
                    }
                } else {
                    stop_counter = 0;
                }
            }

            // --- 4. Timed transition back to white line mode ---
            else if (stop_occurrence == 4)
            {
                if (state == 30 || state == 28 || state == 31) {
                     if (HAL_GetTick() - transition_start_time >= 800) { // Drive for 800ms
                        current_mode = MODE_WHITE_LINE;
                        stop_occurrence = 0;
                        stop_counter = 0;
                        return;
                    }
                } else {
                    stop_counter = 0;
                }
            }
        }

    // ================= 2. RED BOX LOGIC =================
    else if (detectedColor == 'R') {

        // --- 0. First Right Turn ---
        // State 15 (0b01111), 7 (0b00111), 31 (0b11111) -> Line is on the right side
        if (stop_occurrence == 0 && (state == 15 || state == 7 || state == 31)) {
            stop_counter++;
            if (stop_counter > 5) { // Debounce to confirm the line

                // Perform 90-Degree Right Turn
                Set_Motor_Speed(450, -400);
                HAL_Delay(350);

                stop_occurrence = 1;
                stop_counter = 0;
                return;
            }
        }

        // --- 1. Drop Logic for Red Box ---
        else if (stop_occurrence == 1) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            // Trigger if Center sensor (0x04) sees black AND it's a thick line (2-4 sensors total)
            if ((state & 0x04) && (black_count <= 4 && black_count >= 2)) {
                stop_counter++;
                if (stop_counter > 2) {
                    Set_Motor_Speed(0, 0);   // Stop briefly

                    // Drop the box
                    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);
                    HAL_Delay(1000);
                    isCarryingBox = false;
                    black = 2;
                    turn = 0;

                    Set_LED_Color('W');
                    detectedColor = 'R';     // Forces state machine to stay in RED for Occurrence 2

                    // Pivot Left (180-Degree U-Turn)
                    Set_Motor_Speed(-500, 500);
                    HAL_Delay(880);

                    stop_occurrence = 2;     // Move to Occurrence 2 to hunt for All Black
                    stop_counter = 0;
                    return;
                }
            } else {
                stop_counter = 0;
            }
        }

        // --- 2. LEFT TURN Logic for Red Box on All Black Junction ---
        // State 31 (0b11111) = All sensors see black
        else if (stop_occurrence == 2 && state == 31) {
            stop_counter++;
            if (stop_counter > 5) {

                // Perform Left Turn (Adjust speeds and delay for a 90-degree left turn)
                Set_Motor_Speed(-400, 400);
                HAL_Delay(350);

                // RECORD THE TIME to follow the little black line blindly
                transition_start_time = HAL_GetTick();

                stop_occurrence = 3;
                stop_counter = 0;
                return;
            }
        }

        // --- 3. TIMED TRANSITION: Follow little black line, then switch to White mode ---
        else if (stop_occurrence == 3) {
            // After 800ms of driving, switch modes
            if (current_time - transition_start_time >= 800) {
                current_mode = MODE_WHITE_LINE;
                stop_occurrence = 4;

                // Update priorities: We finished Red, Blue is next, Green waits.
                blue_priority_mode = true;
                green_priority_mode = false;
                last_action = 2; // Reset memory to straight
                return;
            }
        }
        else {
            stop_counter = 0;
        }
    }

    // ================= 3. DEFAULT/BLUE BOX LOGIC =================
    else {

        // --- 1. RIGHT TURN LOGIC (Occurrence 0) ---
        if (stop_occurrence == 0) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            // Detect Junction
            if (black_count <= 5 && black_count >= 2) {
                stop_counter++;
                if (stop_counter > 10) {
                    Set_Motor_Speed(BASE_SPEED_RL, BASE_SPEED_RL);   // Go straight through briefly
                    HAL_Delay(150);

                    stop_occurrence = 1;
                    stop_counter = 0;
                }
                return;
            } else {
                stop_counter = 0;
            }
        }

        // Wait for right turn indicator (30=0b11110, 28=0b11100)
        else if (stop_occurrence == 1 && (state == 30 || state == 28)) {
            stop_counter++;
            if (stop_counter > 5) {
                Set_Motor_Speed(0, 420); // Right Turn Pivot
                HAL_Delay(500);
                stop_occurrence = 2;
                stop_counter = 0;
                return;
            }
        }

        // --- 2. DETECT STOP LINE, DROP BOX, TURN LEFT, THEN CONTINUE (Occurrence 2) ---
        else if (stop_occurrence == 2) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            // Junction detected
            if (black_count <= 4 && black_count >= 2) {
                stop_counter++;
                if (stop_counter > 10) {
                    Set_Motor_Speed(0, 0);   // Stop

                    // Drop Blue Box
                    HAL_GPIO_WritePin(GPIOA, Electromagnet_Pin, GPIO_PIN_RESET);
                    HAL_Delay(1000);
                    isCarryingBox = false;
                    black = 3;
                    turn = 0;
                    Set_LED_Color('W');

                    // U-Turn Left
                    Set_Motor_Speed(-500, 500); // Pivot Left
                    HAL_Delay(970);

                    stop_occurrence = 3;
                    stop_counter = 0;
                }
                return;
            } else {
                stop_counter = 0;
            }
        }

        // --- 3. TURN RIGHT & START 1.2s TIMER (Occurrence 3) ---
        else if (stop_occurrence == 3) {
            int black_count = 0;
            for (int i = 0; i < 5; i++) {
                if (state & (1 << i)) black_count++;
            }

            // Thick line junction (3+ sensors)
            if (black_count >= 3) {
                stop_counter++;
                if (stop_counter > 15) {
                    // Perform 90-Degree Right Turn
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

        // --- 4. FOLLOW BLACK FOR 1.2 SECONDS THEN SWITCH TO WHITE (Occurrence 4) ---
        else if (stop_occurrence == 4) {
            // Check if 1.2 seconds (1200ms) have passed
            if (current_time - transition_start_time >= 1200) {
                current_mode = MODE_WHITE_LINE; // Switch back to White Line mode
                stop_occurrence = 0;            // Reset occurrences for next time

                blue_priority_mode = false;     // Blue cycle finished
                green_priority_mode = true;     // Now allow Green cycle to begin

                return;
            }
            // While within timer, just let the bottom Perform_Action handle following
        }
    }

    // Keep following the black line using standard proportional logic
    Perform_Action(Choose_Action(state));
}

/* --------------------------------------------------------------------------
   ROBOT MOVEMENT
   -------------------------------------------------------------------------- */
void Perform_Action(int action) //Made by Tanishk Choudhary, Tharaneeswarar MS
{
    int fwd, turn, stop = 0;

    // PWM profiles setup (Currently identical, but allows for independent tuning later)
    if (current_mode == MODE_BLACK_LINE) {
        fwd  = BASE_SPEED_RL;
        turn = TURN_SPEED_RL;
    } else {
        fwd  = BASE_SPEED_RL;
        turn = TURN_SPEED_RL;
    }

    int left = 0, right = 0;

    // Action execution based on Choose_Action return value
    switch(action) {
        case 0: left = stop; right = turn; break;               // Hard Left
        case 1: left = fwd - 60; right = fwd + 100; break;      // Soft Left
        case 2: left = fwd; right = fwd; break;                 // Straight
        case 3: left = (fwd + 100); right = fwd - 60; break;    // Soft Right
        case 4: left = turn ; right = stop; break;              // Hard Right
    }

    if (left < 0) left = 0; // Floor values to prevent integer wrap around
    Set_Motor_Speed(left, right);
}

void Set_Motor_Speed(int left_pwm, int right_pwm) //Made by Manjari M
{
    // Clamp values to maximum allowed speed
    if (left_pwm > MAX_SPEED_RL) left_pwm = MAX_SPEED_RL;
    if (right_pwm > MAX_SPEED_RL) right_pwm = MAX_SPEED_RL;
    if (left_pwm < -MAX_SPEED_RL) left_pwm = -MAX_SPEED_RL;
    if (right_pwm < -MAX_SPEED_RL) right_pwm = -MAX_SPEED_RL;

    // Left Motor Control (TIM2 CH2 is Fwd, TIM3 CH2 is Rev)
    if (left_pwm >= 0) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, left_pwm);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -left_pwm);
    }

    // Right Motor Control (TIM2 CH3 is Fwd, TIM2 CH4 is Rev)
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
/**
 * @brief Translates the raw ADC array into a 5-bit binary integer state.
 * MSB (bit 4) is the leftmost sensor. LSB (bit 0) is the rightmost sensor.
 * @param find_white If true, values > THRESHOLD become 1. If false, values < THRESHOLD become 1.
 */
uint8_t Get_State(bool find_white) //Made by Manjari M
{
    uint8_t state = 0;
    for (int i = 0; i < 5; i++) {
        bool detection = false;
        if (find_white) {
            if (adc_buffer[i] > IR_THRESHOLD) detection = true;
        } else {
            if (adc_buffer[i] < IR_THRESHOLD) detection = true;
        }

        // Shift bit into place based on sensor index (i=0 is bit 4, i=4 is bit 0)
        if (detection) {
            state |= (1 << (4 - i));
        }
    }
    return state;
}

/**
 * @brief Proportional Control Logic Mapper
 * Takes the 5-bit sensor state and maps it to a movement action index.
 */
int Choose_Action(uint8_t state) //Made by Manjari M
{
    // --- 1. STOP OVERRIDE ---
    // State 15 = 0b01111 -> Four rightmost sensors active. Stop action (5).
    if (state == 15) return 5;

    // --- 2. ALL BLACK OVERRIDE ---
    // State 31 = 0b11111 -> All sensors active (cross intersection). Go straight (2).
    if (state == 31) {
        last_action = 2;
        return 2;
    }

    // --- 3. MEMORY HANDLER ---
    // State 0 = 0b00000 -> Line lost. Repeat the last known turning action to find it.
    if (state == 0) return last_action;

    // --- 4. HARDCODED OVERRIDES ---
    // States indicating line is far to the LEFT. Return Action 0 (Hard Left).
    // 28=0b11100, 24=0b11000, 16=0b10000, 30=0b11110
    if (state == 28 || state == 24 || state == 16 || state == 30) {
        last_action = 0;
        return 0;
    }

    // States indicating line is far to the RIGHT. Return Action 4 (Hard Right).
    // 7=0b00111, 3=0b00011, 1=0b00001, 2=0b00010
    if (state == 7 || state == 3 || state == 1 || state == 2) {
        last_action = 4;
        return 4;
    }

    // --- 5. STANDARD PROPORTIONAL FOLLOWING ---
    int action = 2; // Default Forward

    // 4 = 0b00100 -> Perfectly centered. Go straight (2).
    if (state == 4)       action = 2;
    // 6 = 0b00110 -> Slightly right. Soft Right (3).
    else if (state == 6)  action = 3;
    // 12 = 0b01100 -> Slightly left. Soft Left (1).
    else if (state == 12) action = 1;

    last_action = action; // Save memory
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

    /* -------- TCS3200 Color Sensor S0–S3 Configuration -------- */
    GPIO_InitStruct.Pin = S0_Pin | S1_Pin | S2_Pin | S3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Set color sensor frequency scaling to 20% (S0=HIGH, S1=LOW) */
    HAL_GPIO_WritePin(GPIOB, S0_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, S1_Pin, GPIO_PIN_RESET);

    /* -------- TCS3200 OUT Pin -------- */
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

    // Remap pins for Timer outputs
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_FULLREMAP;
    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP;
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Init TIM2 for Left and Right Motors (Fwd/Rev)
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71; // 72MHz / 72 = 1MHz timer clock
    htim2.Init.Period = 999;   // 1kHz PWM frequency
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

    // Init TIM3 for additional motor phase
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 71;
    htim3.Init.Period = 999;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

    // Start PWM generation
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

  // Configure 5 ADC channels for the IR array
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
