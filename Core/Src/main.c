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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t raw_data = 0;
uint32_t code = 0;
extern TIM_HandleTypeDef htim3; // Timer3 referansı

uint16_t sayac = 0;
// 0-9 arası rakamların segment kodları (A,B,C,D,E,F,G sırasıyla)
// Ortak Katot (Common Cathode) ekran için:
uint8_t segment_tablosu[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint32_t get_us(void) {
    return __HAL_TIM_GET_COUNTER(&htim3);
}


void Ekran_Goster(uint16_t sayi) {
    int basamaklar[4];
    basamaklar[0] = (sayi / 1000) % 10;
    basamaklar[1] = (sayi / 100) % 10;
    basamaklar[2] = (sayi / 10) % 10;
    basamaklar[3] = sayi % 10;

    GPIO_TypeDef* Digit_Ports[] = {DIGIT_1_GPIO_Port, DIGIT_2_GPIO_Port, DIGIT_3_GPIO_Port, DIGIT_4_GPIO_Port};
    uint16_t Digit_Pins[] = {DIGIT_1_Pin, DIGIT_2_Pin, DIGIT_3_Pin, DIGIT_4_Pin};

    for (int i = 0; i < 4; i++) {
        // Hepsini söndür
        HAL_GPIO_WritePin(GPIOB, DIGIT_1_Pin|DIGIT_2_Pin|DIGIT_3_Pin|DIGIT_4_Pin, GPIO_PIN_SET);

        // Segmentleri yak
        uint8_t mask = segment_tablosu[basamaklar[i]];
        HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET); // Önce temizle

        HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (mask & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (mask & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (mask & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (mask & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (mask & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (mask & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (mask & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // Digiti seç (Aktif et)
        HAL_GPIO_WritePin(Digit_Ports[i], Digit_Pins[i], GPIO_PIN_RESET);
        HAL_Delay(0.01);
        HAL_GPIO_WritePin(Digit_Ports[i], Digit_Pins[i], GPIO_PIN_SET);
    }
}

void Bekle_Ve_Tazele(uint32_t ms) {
    uint32_t start = HAL_GetTick();

    // En az bir kez ekranı tazele (4ms civarı sürer)
    Ekran_Goster(sayac);

    // Eğer istenen süre (ms) hala dolmadıysa taramaya devam et
    while (HAL_GetTick() - start < ms) {
        Ekran_Goster(sayac);
    }
}



void Step_Motor_Sur(int tam_tur_sayisi) {
    // 28BYJ-48 için bir tam tur yaklaşık 512 adımdır (Full step modunda)


	// --- BAŞA EKLE: Tuşa basıldığı an ses çıksın ---
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Buzzer Aç
	    uint32_t bip_baslangic = HAL_GetTick();
	    while(HAL_GetTick() - bip_baslangic < 100) { // 100ms boyunca
	        Ekran_Goster(sayac); // Ses çıkarken ekran sönmesin
	    }
	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Buzzer Kapat
	    // ----------------------------------------------


    int toplam_adim = tam_tur_sayisi * 512;

    for(int i = 0; i < toplam_adim; i++) {
        // Adım 1
        HAL_GPIO_WritePin(GPIOC, IN1_Pin, 1);
        HAL_GPIO_WritePin(GPIOC, IN2_Pin|IN3_Pin|IN4_Pin, 0);
        Bekle_Ve_Tazele(2);

        // Adım 2
        HAL_GPIO_WritePin(GPIOC, IN2_Pin, 1);
        HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN3_Pin|IN4_Pin, 0);
        Bekle_Ve_Tazele(2);

        // Adım 3
        HAL_GPIO_WritePin(GPIOC, IN3_Pin, 1);
        HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN4_Pin, 0);
        Bekle_Ve_Tazele(2);

        // Adım 4
        HAL_GPIO_WritePin(GPIOC, IN4_Pin, 1);
        HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin, 0);
        Bekle_Ve_Tazele(2);

//        // Her 10 adımda bir ekranı tazele ki sönük kalmasın
//        if(i % 10 == 0) Ekran_Goster(sayac);
    }
    // Enerjiyi kes (Isınmayı önle)
    HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, 0);
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3); // Timer3'ü başlat
  HAL_Delay(400);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, DIGIT_4_Pin, GPIO_PIN_RESET); // Sadece 4. haneyi yakmaya çalış
 // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

  // LED durumlarını tutmak için değişkenler
  uint8_t r = 0, g = 0, b = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE BEGIN WHILE */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      Ekran_Goster(sayac);

      // RGB LED'leri sür (Katot: 1=YAKAR, 0=SÖNDÜRÜR)
      HAL_GPIO_WritePin(GPIOB, RGB_RED_Pin, (r == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_GREEN_Pin, (g == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, RGB_BLUE_Pin, (b == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
      {
          __HAL_TIM_SET_COUNTER(&htim3, 0);
          while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && __HAL_TIM_GET_COUNTER(&htim3) < 15000);

          if (__HAL_TIM_GET_COUNTER(&htim3) > 2000)
          {
              uint32_t temp_data = 0;
              __HAL_TIM_SET_COUNTER(&htim3, 0);
              while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && __HAL_TIM_GET_COUNTER(&htim3) < 10000);

              for (int i = 0; i < 32; i++)
              {
                  __HAL_TIM_SET_COUNTER(&htim3, 0);
                  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && __HAL_TIM_GET_COUNTER(&htim3) < 5000);
                  __HAL_TIM_SET_COUNTER(&htim3, 0);
                  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && __HAL_TIM_GET_COUNTER(&htim3) < 5000);
                  if (__HAL_TIM_GET_COUNTER(&htim3) > 1000) temp_data |= (1UL << (31 - i));
              }
              code = temp_data;

              if (code != 0 && code != 0xFFFFFFFF)
              {
                  r = 0; g = 0; b = 0; // Renkleri sıfırla
                  uint8_t key = (uint8_t)(code & 0xFF); // Sadece son 8 bite odaklan

                  // --- GÜNCEL KONTROL MANTIGI ---
                  if (key == 0x66) { // 12. Tuş (Motor 3 Tur)
                          sayac++;

                           Step_Motor_Sur(3);
                                    }
                  else if (key == 0x85) { // 12. Tuş (Motor 3 Tur)
                      sayac++;
                      Step_Motor_Sur(3);
                  }
                  // RENK TUŞLARI
                  else if (key == 0x5D) { // 1, (Kırmızı)
                      r = 1; sayac += 1;
                  }
                  else if ( key == 0xFD) { //  5 (Mavi)
                      b = 1; sayac += 1;
                  }
                  else if (key == 0x1D) { // 3 (Yeşil)
                      g = 1; sayac += 1;
                  }
                  // DIGERLERI
                  else if (key == 0xCF) { sayac = 0; }     // 10 (Sıfırla)
                  else if (key == 0x4F) {
                	  HAL_Delay(100);
                	  sayac += 200;
                	  HAL_Delay(50);
                  }  // 9 (+200)
//                  else { sayac++; } // Hiçbiri tutmazsa genel artış

                  if (sayac > 9999) sayac %= 10000;

                  // BUZZER ONAY
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
                  uint32_t t_bip = HAL_GetTick();
                  while(HAL_GetTick() - t_bip < 100) Ekran_Goster(sayac);
                  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

                  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) Ekran_Goster(sayac);
              }
          }

  }
//    	    // TU�?LARI Ö�?RENME KODU:......................................................................
//
//	            Ekran_Goster(sayac);
//
//    	          if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
//    	          {
//    	              __HAL_TIM_SET_COUNTER(&htim3, 0);
//    	              // Leader Pulse: NEC standardı 9ms'dir.
//    	              while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && __HAL_TIM_GET_COUNTER(&htim3) < 15000);
//
//    	              uint32_t pulse = __HAL_TIM_GET_COUNTER(&htim3);
//
//    	              // Sinyal 4ms'den büyükse (Gürültü değilse) okumaya başla
//    	              if (pulse > 4000)
//    	              {
//    	                  // Başlangıç boşluğunu (4.5ms) geç
//    	                  __HAL_TIM_SET_COUNTER(&htim3, 0);
//    	                  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && __HAL_TIM_GET_COUNTER(&htim3) < 10000);
//
//    	                  uint32_t temp_data = 0;
//    	                  for (int i = 0; i < 32; i++)
//    	                  {
//    	                      // Bit başındaki kısa LOW darbesini (560us) bekle
//    	                      __HAL_TIM_SET_COUNTER(&htim3, 0);
//    	                      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && __HAL_TIM_GET_COUNTER(&htim3) < 5000);
//
//    	                      // Bit değerini belirleyen HIGH süresini ölç
//    	                      __HAL_TIM_SET_COUNTER(&htim3, 0);
//    	                      while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET && __HAL_TIM_GET_COUNTER(&htim3) < 5000);
//
//    	                      // Eğer HIGH süresi 1000us'den uzunsa "1", kısa ise "0"
//    	                      if (__HAL_TIM_GET_COUNTER(&htim3) > 1000) {
//    	                          temp_data |= (1UL << (31 - i));
//    	                      }
//    	                  }
//    	                  code = temp_data; // BREAKPOINT BURAYA
//
//    	                  if (code != 0 && code != 0xFFFFFFFF) {
//    	                      // Sadece gerçek kod gelince bip yap
//    	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//    	                      HAL_Delay(100);
//    	                      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//    	                  }
//
//    	                  // Elini tuştan çekene kadar bekle
//    	                  while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
//    	                      Ekran_Goster(sayac);
//    	                  }
//
//    	          }
//
//  }
//

//
//  }
//   İLK ÇALI�?AN HALİ..........................................................................
//    	// 1. Ekranı her zaman tazele
//    	  Ekran_Goster(sayac);
//
//    	  // 2. PA10'da herhangi bir hareket var mı?
//    	  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET)
//    	  {
//    	    // Küçük bir gecikme ile gürültü kontrolü yap (En az 500us sinyal sürsün)
//    	    uint32_t baslangic = HAL_GetTick();
//
//    	    // Tuşa basıldığında yapılacak işlemler
//    	    sayac++;
//    	    if (sayac > 9999) sayac = 0;
//
//    	    // Buzzer'ı öttür
//    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
//
//    	    // Bip sesi sırasında ekranı 100ms boyunca tazele
//    	    uint32_t bip_vakti = HAL_GetTick();
//    	    while(HAL_GetTick() - bip_vakti < 100) {
//    	        Ekran_Goster(sayac);
//    	    }
//    	    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
//
//    	    // Tuş basılı tutulduğu sürece bekle ve ekranı tazele (Sürekli artışı önler)
//    	    // Eğer sensör bozulduysa burada sonsuz döngüye girebilir, dikkat!
//    	    uint32_t timeout = HAL_GetTick();
//    	    while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && (HAL_GetTick() - timeout < 500)) {
//    	        Ekran_Goster(sayac);
//    	    }
//    	  }
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG_F_Pin|SEG_E_Pin|IN1_Pin|IN2_Pin
                          |IN3_Pin|BUZZER_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|LD2_Pin
                          |SEG_G_Pin|SEG_DP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_D_Pin|DIGIT_1_Pin|DIGIT_2_Pin|RGB_BLUE_Pin
                          |DIGIT_3_Pin|DIGIT_4_Pin|RGB_RED_Pin|RGB_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_F_Pin SEG_E_Pin IN1_Pin IN2_Pin
                           IN3_Pin BUZZER_Pin IN4_Pin */
  GPIO_InitStruct.Pin = SEG_F_Pin|SEG_E_Pin|IN1_Pin|IN2_Pin
                          |IN3_Pin|BUZZER_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A_Pin SEG_B_Pin SEG_C_Pin LD2_Pin
                           SEG_G_Pin SEG_DP_Pin */
  GPIO_InitStruct.Pin = SEG_A_Pin|SEG_B_Pin|SEG_C_Pin|LD2_Pin
                          |SEG_G_Pin|SEG_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_D_Pin DIGIT_1_Pin DIGIT_2_Pin RGB_BLUE_Pin
                           DIGIT_3_Pin DIGIT_4_Pin RGB_RED_Pin RGB_GREEN_Pin */
  GPIO_InitStruct.Pin = SEG_D_Pin|DIGIT_1_Pin|DIGIT_2_Pin|RGB_BLUE_Pin
                          |DIGIT_3_Pin|DIGIT_4_Pin|RGB_RED_Pin|RGB_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IR_IN_Pin */
  GPIO_InitStruct.Pin = IR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IR_IN_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
