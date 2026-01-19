/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdlib.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    GAME_IDLE,
    GAME_READY,
    GAME_WAIT_RANDOM,
    GAME_LED_ON_WAIT_PRESS,
    GAME_SHOW_RESULT,
    GAME_ERROR
} GameState_t;

typedef enum {
    PLAYER_1 = 0,
    PLAYER_2 = 1
} Player_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Pines del display 7 segmentos
#define SEG_A_PORT GPIOB
#define SEG_A_PIN  GPIO_PIN_0
#define SEG_B_PORT GPIOB
#define SEG_B_PIN  GPIO_PIN_1
#define SEG_C_PORT GPIOB
#define SEG_C_PIN  GPIO_PIN_2
#define SEG_D_PORT GPIOB
#define SEG_D_PIN  GPIO_PIN_3
#define SEG_E_PORT GPIOB
#define SEG_E_PIN  GPIO_PIN_4
#define SEG_F_PORT GPIOB
#define SEG_F_PIN  GPIO_PIN_5
#define SEG_G_PORT GPIOB
#define SEG_G_PIN  GPIO_PIN_6
#define SEG_DP_PORT GPIOB
#define SEG_DP_PIN  GPIO_PIN_7

#define DIG1_PORT GPIOA
#define DIG1_PIN  GPIO_PIN_8
#define DIG2_PORT GPIOA
#define DIG2_PIN  GPIO_PIN_9
#define DIG3_PORT GPIOA
#define DIG3_PIN  GPIO_PIN_10
#define DIG4_PORT GPIOA
#define DIG4_PIN  GPIO_PIN_15

// Pines de LEDs
#define LED1_PORT GPIOB
#define LED1_PIN  GPIO_PIN_8
#define LED2_PORT GPIOB
#define LED2_PIN  GPIO_PIN_9

// Constantes del juego
#define DEBOUNCE_TIME_MS 50
#define MIN_WAIT_TIME_MS 500
#define MAX_WAIT_TIME_MS 2000
#define MIN_REACTION_TIME_US 50000   // 50ms mínimo
#define MAX_REACTION_TIME_US 1000000 // 1s máximo base
#define SCORE_LIMIT 99
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// Variables del juego
GameState_t game_state = GAME_IDLE;
Player_t current_player = PLAYER_1;
uint8_t score_p1 = 0;
uint8_t score_p2 = 0;
uint32_t led_on_time = 0;
uint32_t button_press_time = 0;
uint32_t wait_start_time = 0;
uint32_t random_wait_time = 0;
bool game_running = false;
bool wrong_player_pressed = false;  // Flag para jugador incorrecto
int scoreP1 = 0;
int scoreP2 = 0;
// Variables para debounce
uint32_t last_button_time_p1 = 0;
uint32_t last_button_time_p2 = 0;

// Variables del display
uint8_t current_digit = 0;
uint8_t display_buffer[4] = {0, 0, 0, 0}; // P1: [0][1], P2: [2][3]

// Tabla de segmentos para dígitos 0-9 (cátodo común, 0=encendido)
const uint8_t digit_segments[10] = {
    0b00000011, // 0: a,b,c,d,e,f (invertido)
    0b10011111, // 1: b,c (invertido)
    0b00100101, // 2: a,b,d,e,g (invertido)
    0b00001101, // 3: a,b,c,d,g (invertido)
    0b10011001, // 4: b,c,f,g (invertido)
    0b01001001, // 5: a,c,d,f,g (invertido)
    0b01000001, // 6: a,c,d,e,f,g (invertido)
    0b00011111, // 7: a,b,c (invertido)
    0b00000001, // 8: a,b,c,d,e,f,g (invertido)
    0b00001001  // 9: a,b,c,d,f,g (invertido)
};

// Variables de dificultad
uint32_t max_reaction_time_us = MAX_REACTION_TIME_US;
uint32_t last_adc_value = 0;
bool showing_difficulty = false;
uint32_t last_difficulty_change_time = 0;
#define DIFFICULTY_DISPLAY_TIME_MS 2000
#define ADC_CHANGE_THRESHOLD 50  // Umbral para detectar cambios significativos
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// Funciones del display
void Display_Clear(void);
void Display_WriteSegments(uint8_t segments);
void Display_SelectDigit(uint8_t digit);
void Display_UpdateBuffer(void);
void Display_Refresh(void);

// Funciones del juego
void Game_Init(void);
void Game_StateMachine(void);
void Game_StartNewRound(void);
void Game_ProcessButtonPress(Player_t player);
void Game_UpdateScore(Player_t player, bool correct);
void Game_Reset(void);

// Funciones de utilidad
uint32_t GetRandomWaitTime(void);
void UpdateDifficulty(void);
void SetLED(Player_t player, bool state);
uint32_t GetMicroseconds(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ========== FUNCIONES DEL DISPLAY ==========
void Display_Clear(void) {
    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
    
    HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_SET);
}

void Display_WriteSegments(uint8_t segments) {
    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, (segments & 0x80) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, (segments & 0x40) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, (segments & 0x20) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, (segments & 0x10) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, (segments & 0x08) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, (segments & 0x04) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, (segments & 0x02) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, (segments & 0x01) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void Display_SelectDigit(uint8_t digit) {
    // Apagar todos los dígitos primero
    HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_RESET);
    
    // Encender el dígito seleccionado
    switch(digit) {
        case 0: HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_SET); break;
        case 3: HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_SET); break;
    }
}

void Display_UpdateBuffer(void) {
    if (showing_difficulty) {
        // Mostrar tiempo límite en milisegundos (ej: 0500 para 500ms)
        uint32_t time_limit_ms = max_reaction_time_us / 1000;
        
        display_buffer[0] = (time_limit_ms / 1000) % 10;  // Miles
        display_buffer[1] = (time_limit_ms / 100) % 10;   // Centenas 
        display_buffer[2] = (time_limit_ms / 10) % 10;    // Decenas
        display_buffer[3] = time_limit_ms % 10;           // Unidades
    } else {
        // Actualizar buffer con las puntuaciones (intercambiadas para coincidir con posición física)
        display_buffer[0] = score_p2 / 10;  // Decenas P2 (botón derecha)
        display_buffer[1] = score_p2 % 10;  // Unidades P2 (botón derecha)
        display_buffer[2] = score_p1 / 10;  // Decenas P1 (botón izquierda)
        display_buffer[3] = score_p1 % 10;  // Unidades P1 (botón izquierda)
    }
}

void Display_Refresh(void) {
    // PASO 1: Apagar TODOS los dígitos primero (HIGH = apagado en cátodo común)
    HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_SET);
    
    // PASO 2: Limpiar TODOS los segmentos (HIGH = apagado en cátodo común)
    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SEG_DP_PORT, SEG_DP_PIN, GPIO_PIN_SET);
    
    // PASO 3: Pequeña pausa para estabilizar
    for(volatile int i = 0; i < 10; i++);
    
    // PASO 4: Escribir segmentos del dígito actual
    Display_WriteSegments(digit_segments[display_buffer[current_digit]]);
    
    // PASO 5: Encender SOLO el dígito correspondiente (LOW = encendido en cátodo común)
    switch(current_digit) {
        case 0: HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_RESET); break;
        case 3: HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_RESET); break;
    }
    
    // PASO 6: Pasar al siguiente dígito
    current_digit = (current_digit + 1) % 4;
}

// ========== FUNCIONES DEL JUEGO ==========
void Game_Init(void) {
    game_state = GAME_IDLE;
    score_p1 = 0;
    score_p2 = 0;
    current_player = PLAYER_1;
    game_running = false;
    
    // Apagar LEDs
    SetLED(PLAYER_1, false);
    SetLED(PLAYER_2, false);
    
    // Actualizar display
    Display_UpdateBuffer();
    
    // Inicializar generador aleatorio con ADC
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    srand(adc_val + HAL_GetTick());
    
    // Inicializar valor del ADC para detección de cambios
    last_adc_value = adc_val;
}

void Game_StartNewRound(void) {
    // Resetear flag de jugador incorrecto
    wrong_player_pressed = false;
    
    // Seleccionar jugador aleatorio
    current_player = (rand() % 2) ? PLAYER_1 : PLAYER_2;
    
    // Generar tiempo de espera aleatorio
    random_wait_time = GetRandomWaitTime();
    wait_start_time = HAL_GetTick();
    
    game_state = GAME_WAIT_RANDOM;
}

void Game_ProcessButtonPress(Player_t player) {
    uint32_t press_time = GetMicroseconds();
    
    switch(game_state) {
        case GAME_IDLE:
            // Iniciar juego
            game_running = true;
            Game_StartNewRound();
            break;
            
        case GAME_WAIT_RANDOM:
            // Pulsación prematura
            if (player == current_player) {
                Game_UpdateScore(player, false);
                game_state = GAME_SHOW_RESULT;
            }
            break;
            
        case GAME_LED_ON_WAIT_PRESS:
            if (player == current_player) {
                // Jugador correcto pulsó
                uint32_t reaction_time = press_time - led_on_time;
                
                // Apagar LED
                SetLED(player, false);
                
                // Evaluar reacción
                if (reaction_time >= MIN_REACTION_TIME_US && reaction_time <= max_reaction_time_us) {
                    Game_UpdateScore(player, true);
                } else {
                    Game_UpdateScore(player, false);
                }
                
                game_state = GAME_SHOW_RESULT;
            } else {
                // Jugador incorrecto pulsó - restar punto pero no terminar ronda aún
                if (player == PLAYER_1 && score_p1 > 0) {
                    score_p1--;
                } else if (player == PLAYER_2 && score_p2 > 0) {
                    score_p2--;
                }
                
                // Marcar que el jugador incorrecto ya pulsó
                wrong_player_pressed = true;
                
                // Actualizar display si no se está mostrando dificultad
                if (!showing_difficulty) {
                    Display_UpdateBuffer();
                }
                
                // NO cambiar estado - permitir que el jugador correcto aún pueda acertar
            }
            break;
            
        default:
            break;
    }
}

void Game_UpdateScore(Player_t player, bool correct) {
    if (correct) {
        if (player == PLAYER_1 && score_p1 < SCORE_LIMIT) {
            score_p1++;
        } else if (player == PLAYER_2 && score_p2 < SCORE_LIMIT) {
            score_p2++;
        }
    }
    
    if (!showing_difficulty) {
        Display_UpdateBuffer();
    }
}
void Game_DisplayPlayerQuality(PlayerQuality_t quality) {

    switch (quality) {
        case PLAYER_QUALITY_POOR:
            Display_Print("POOR");
            break;

        case PLAYER_QUALITY_AVERAGE:
            Display_Print("AVERAGE");
            break;

        case PLAYER_QUALITY_GOOD:
            Display_Print("GOOD");
            break;

        case PLAYER_QUALITY_EXCELLENT:
            Display_Print("EXCELLENT");
            break;
    }
}

void Game_Reset(void) {
    score_p1 = 0;
    score_p2 = 0;
    game_running = false;
    game_state = GAME_IDLE;
    
    SetLED(PLAYER_1, false);
    SetLED(PLAYER_2, false);
    
    if (!showing_difficulty) {
        Display_UpdateBuffer();
    }
}

void Game_StateMachine(void) {
    static uint32_t result_start_time = 0;
    
    switch(game_state) {
        case GAME_IDLE:
            // Esperando inicio del juego
            break;
            
        case GAME_WAIT_RANDOM:
            if ((HAL_GetTick() - wait_start_time) >= random_wait_time) {
                // Encender LED y cambiar estado
                SetLED(current_player, true);
                led_on_time = GetMicroseconds();
                game_state = GAME_LED_ON_WAIT_PRESS;
            }
            break;
            
        case GAME_LED_ON_WAIT_PRESS:
            // Verificar timeout
            if ((GetMicroseconds() - led_on_time) > max_reaction_time_us) {
                SetLED(current_player, false);
                
                // Si el jugador incorrecto pulsó pero el correcto no, solo penalizar al correcto por timeout
                if (!wrong_player_pressed) {
                    Game_UpdateScore(current_player, false);
                } else {
                    // El jugador incorrecto ya fue penalizado, solo penalizar al correcto por timeout
                    Game_UpdateScore(current_player, false);
                }
                
                // Resetear flag
                wrong_player_pressed = false;
                game_state = GAME_SHOW_RESULT;
            }
            break;
            
        case GAME_SHOW_RESULT:
            if (result_start_time == 0) {
                result_start_time = HAL_GetTick();
            }
            
            // Mostrar resultado por 1 segundo
            if ((HAL_GetTick() - result_start_time) >= 1000) {
                result_start_time = 0;
                if (game_running) {
                    Game_StartNewRound();
                } else {
                    game_state = GAME_IDLE;
                }
            }
            break;
            
        default:
            game_state = GAME_IDLE;
            break;
    }
}

// ========== FUNCIONES DE UTILIDAD ==========
uint32_t GetRandomWaitTime(void) {
    return MIN_WAIT_TIME_MS + (rand() % (MAX_WAIT_TIME_MS - MIN_WAIT_TIME_MS));
}

void UpdateDifficulty(void) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        
        // Detectar cambios significativos en el potenciómetro
        if (abs((int32_t)adc_value - (int32_t)last_adc_value) > ADC_CHANGE_THRESHOLD) {
            last_adc_value = adc_value;
            last_difficulty_change_time = HAL_GetTick();
            showing_difficulty = true;
        }
        
        // Mapear ADC (0-4095) a tiempo máximo (250ms - 1.5s)
        // Usar aritmética de 64 bits para evitar overflow
        max_reaction_time_us = 250000 + ((uint64_t)adc_value * 1250000) / 4095;
    }
    HAL_ADC_Stop(&hadc1);
}

void SetLED(Player_t player, bool state) {
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;
    
    if (player == PLAYER_1) {
        HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, pin_state);
    } else {
        HAL_GPIO_WritePin(LED2_PORT, LED2_PIN, pin_state);
    }
}

uint32_t GetMicroseconds(void) {
    return __HAL_TIM_GET_COUNTER(&htim2);
}
uint8_t sevenSegDigits[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
while (1)
{
    Reflexes_Game();

    for (int i = 0; i < 100; i++)
    {
        Display_Show(1, scoreP1);
        Display_Show(2, scoreP2);
    }
}
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  // Inicializar el juego
  Game_Init();
  
  // Iniciar temporizadores
  HAL_TIM_Base_Start(&htim2);      // Timer para microsegundos
  HAL_TIM_Base_Start_IT(&htim3);   // Timer para multiplexado del display
  
  // Habilitar interrupciones EXTI
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Ejecutar máquina de estados del juego
    Game_StateMachine();
    
    // Actualizar dificultad cada 100ms
    static uint32_t last_difficulty_update = 0;
    if ((HAL_GetTick() - last_difficulty_update) >= 100) {
        UpdateDifficulty();
        last_difficulty_update = HAL_GetTick();
     
    }
    
    // Controlar visualización de dificultad
    if (showing_difficulty) {
        if ((HAL_GetTick() - last_difficulty_change_time) >= DIFFICULTY_DISPLAY_TIME_MS) {
            showing_difficulty = false;
        }
        Display_UpdateBuffer();  // Actualizar display cuando se muestre dificultad
    }
    
    // Reset del juego si se mantienen presionados ambos botones por 2 segundos
    static uint32_t reset_start = 0;
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET && 
        HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET) {
        if (reset_start == 0) {
            reset_start = HAL_GetTick();
        } else if ((HAL_GetTick() - reset_start) >= 2000) { // 2 segundos
            Game_Reset();
            reset_start = 0;
        }
    } else {
        // Reset del contador si no están presionados ambos
        reset_start = 0;
    }
    
    HAL_Delay(1); // Pequeña pausa para evitar saturar la CPU
  }
    Reflexes_TimeOut_LED();
    HAL_Delay(2000); // pausa entre rondas (opcional)

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 PB4 PB5 PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SevenSeg_Write(uint8_t value)
{
    HAL_GPIO_WritePin(SEG_A_PORT, SEG_A_PIN, (value & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_B_PORT, SEG_B_PIN, (value & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_C_PORT, SEG_C_PIN, (value & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_D_PORT, SEG_D_PIN, (value & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_E_PORT, SEG_E_PIN, (value & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_F_PORT, SEG_F_PIN, (value & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SEG_G_PORT, SEG_G_PIN, (value & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
// Callback para interrupciones EXTI (botones)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();
    
    // Debounce y procesamiento de botones
    if (GPIO_Pin == GPIO_PIN_0) { // Jugador 1
        if ((current_time - last_button_time_p1) > DEBOUNCE_TIME_MS) {
            last_button_time_p1 = current_time;
            Game_ProcessButtonPress(PLAYER_1);
        }
    }
    else if (GPIO_Pin == GPIO_PIN_1) { // Jugador 2
        if ((current_time - last_button_time_p2) > DEBOUNCE_TIME_MS) {
            last_button_time_p2 = current_time;
            Game_ProcessButtonPress(PLAYER_2);
        }
    }
}
void Display_Show(uint8_t player, int score)
{
    if (score < 0) score = 0;
    if (score > 99) score = 99;

    uint8_t decenas = score / 10;
    uint8_t unidades = score % 10;

    if (player == 1)
    {
        HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_RESET);
    }

    SevenSeg_Write(sevenSegDigits[decenas]);
    HAL_Delay(3);

    if (player == 1)
    {
        HAL_GPIO_WritePin(DIG1_PORT, DIG1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIG2_PORT, DIG2_PIN, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DIG3_PORT, DIG3_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIG4_PORT, DIG4_PIN, GPIO_PIN_SET);
    }

    SevenSeg_Write(sevenSegDigits[unidades]);
    HAL_Delay(3);
}
void Reflexes_Game(void)
{
    uint32_t startTime;

    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
    startTime = HAL_GetTick();

    while ((HAL_GetTick() - startTime) < 2000)
    {
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
        {
            scoreP1++;
            HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
            return;
        }

        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET)
        {
            scoreP2++;
            HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
            return;
        }
    }

    // Fallo
    for (int i = 0; i < 6; i++)
    {
        HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
        HAL_Delay(150);
    }

    scoreP1--;
    scoreP2--;
}
// Callback para TIM3 (multiplexado del display)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        Display_Refresh();
    }
}

// Handlers de interrupciones EXTI
void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

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
void Reflexes_TimeOut_LED(void)
{
    uint32_t startTime = HAL_GetTick();

    // Apagar LED al empezar (por seguridad)
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

    // Esperar hasta 3 segundos
    while ((HAL_GetTick() - startTime) < 3000)
    {
        // Si se pulsa el botón antes de 3 s, salir sin encender LED
        if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == GPIO_PIN_RESET)
        {
            return; // Botón pulsado a tiempo
        }
    }

    // Si llega aquí, NO se pulsó el botón en 3 s
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
}





