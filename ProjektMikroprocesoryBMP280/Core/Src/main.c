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
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <bme280.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//zmienne do obsługi protokołu
#define USART_TXBUF_LEN 300
#define USART_RXBUF_LEN 300
#define bufor_len 1000
uint8_t USART_TxBUF[USART_TXBUF_LEN];
uint8_t USART_RxBUF[USART_RXBUF_LEN];

__IO int USART_TX_EMPTY=0; //wskaźnik dla bufora nadawczego
__IO int USART_TX_BUSY=0;
__IO int USART_RX_EMPTY=0; //wskaźnik dla bufora odbiorczego
__IO int USART_RX_BUSY=0;

extern float bufor[bufor_len]; //bufor czujnika*/
uint16_t buf_idx=0;

//do ramki
char nadawca[3];
char odbiorca[3];
char dlugosc_komendy[3];
char suma_kontrolna[2];
const char adres_stm[3] = "STM";
uint8_t dlugosc_ramki = 0;
int com_len;
int kontrolna_suma;
int znak_konca = 0;
int znak_poczatku = 0;

//do analizy ramki
int odczyt = 0;
int czas = 0;
int pom = 0;
int wilg = 0;

__IO uint32_t CNT;             /*!< TIM counter register,                        Address offset: 0x24 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int8_t i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	//HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout) blokujace dla porownania
	HAL_I2C_Mem_Read(&hi2c1, dev_id, reg_addr, 1, reg_data, len, 100);
	//tu jest na blokujaco
	//HAL_I2C_Mem_Read_IT(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size)
	// HAL_I2C_Mem_Read_IT(&hi2c1, dev_id, reg_addr, 1, reg_data, len); // przerwania
	return rslt;
}

int8_t i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	//HAL_I2C_Mem_Write_IT(&hi2c1, dev_id, reg_addr, 1, reg_data, len);
	//przerwania
	HAL_I2C_Mem_Write(&hi2c1, dev_id, reg_addr, 1, reg_data, len, 100);
	//blokujaco
	return rslt;
}

int8_t BME280_init(void)
{
	struct bme280_dev dev;
	int8_t rslt = BME280_OK;
	uint8_t dev_addr = BME280_I2C_ADDR_PRIM;
	int8_t init_done = BME280_E_DEV_NOT_FOUND;
	uint8_t settings_sel;

	dev.intf_ptr = &dev_addr;
	dev.intf = BME280_I2C_INTF;
	dev.read = i2c_read;
	dev.write = i2c_write;

	rslt = bme280_init(&dev);

	/* Recommended mode of operation: Indoor navigation */
	dev.settings.osr_h = BME280_OVERSAMPLING_1X;
	dev.settings.osr_p = BME280_OVERSAMPLING_16X;
	dev.settings.osr_t = BME280_OVERSAMPLING_2X;
	dev.settings.filter = BME280_FILTER_COEFF_16;
	dev.settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	if(rslt == BME280_OK)
	{
		settings_sel = BME280_OSR_PRESS_SEL;
		settings_sel = BME280_OSR_TEMP_SEL;
		settings_sel = BME280_OSR_HUM_SEL;
		settings_sel = BME280_STANDBY_SEL;
		settings_sel = BME280_FILTER_SEL;

		rslt = bme280_set_sensor_settings(settings_sel, &dev);

		if(rslt == BME280_OK)
		{
			rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, &dev);
			init_done = rslt;
		}
	}
	return rslt;
}

int8_t BME280_read_data(void)
{
	struct bme280_dev dev;
	struct bme280_data comp_data;
	int8_t init_done = BME280_E_DEV_NOT_FOUND;
	int8_t rslt = BME280_E_COMM_FAIL;
	if(init_done == BME280_OK)
	{
		USART_fsend(" Temperatura, Cisnienie, Wilgotnosc\r\n");
		/* Pobranie danych z czujnika i wyswietlenie */
		rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);
		print_sensor_data("pom[",&comp_data,"]");
	}
	return rslt;
}

int8_t BME280_read_data2(void)
{
	struct bme280_dev dev;
	struct bme280_data comp_data;
	int8_t init_done = BME280_E_DEV_NOT_FOUND;
	int8_t rslt = BME280_E_COMM_FAIL;
	if(init_done == BME280_OK)
	{
		USART_fsend(" Temperatura , Cisnienie\r\n");
		/* Pobranie danych z czujnika i wyswietlenie */
		rslt = bme280_get_sensor_data(BME280_PRESS, &comp_data, &dev);
		print_sensor_data("pom[",&comp_data,"]");
	}
	return rslt;
}

int8_t BME280_read_data3(void)
{
	struct bme280_dev dev;
	struct bme280_data comp_data;
	int8_t init_done = BME280_E_DEV_NOT_FOUND;
	int8_t rslt = BME280_E_COMM_FAIL;
	if(init_done == BME280_OK)
	{
		USART_fsend(" Wilgotnosc\r\n");
		/* Pobranie danych z czujnika i wyswietlenie */
		rslt = bme280_get_sensor_data(BME280_HUM, &comp_data, &dev);
		print_sensor_data("pom[",&comp_data,"]");
	}
	return rslt;
}

void print_sensor_data(struct bme280_data *comp_data)
{
	if(CNT< 500)
	{
#ifdef BME280_FLOAT_ENABLE
		USART_fsend("%0.2f C, %0.2f HPa, %0.2f %%\r\n",comp_data->temperature, (comp_data->pressure / 100), comp_data->humidity);
		bufor[CNT] =comp_data->temperature;
		CNT++; //spr wartosci
		if(CNT < 500)
		{
			bufor[CNT] =(comp_data->pressure/100);
		}else
		{
			CNT = 0;
			USART_fsend("Przekroczono zakres counter'a (cisnienie) \r \n");
			bufor[CNT] = (comp_data->pressure/100);
		}
		CNT++;
		if(CNT < 500)
		{
			bufor[CNT] =comp_data->humidity;
		}else
		{
			CNT = 0;
			USART_fsend("Przekroczono zakres counter'a (wilgotnosc)\r \n");
			bufor[CNT] = (comp_data->humidity);
		}
		CNT++;
#else

		USART_fsend("%ld, %ld, %ld\r\n",comp_data->temperature, comp_data->pressure, comp_data->humidity);

#endif

	}else
	{
		USART_fsend("Przekroczono zakres counter'a (temperatura) \r \n" );
		CNT = 0;//zapętlanie jak przekroczy,dane nadpisuje
	}
}

uint8_t USART_kbhit()
{
	if(USART_RX_EMPTY==USART_RX_BUSY)
	{
		return 0;
	}else
	{
		return 1;
	}
}//USART_kbhit

int16_t USART_getchar()
{
	int16_t tmp;
	if(USART_RX_EMPTY!=USART_RX_BUSY)
	{
		tmp=USART_RxBUF[USART_RX_BUSY];
		USART_RX_BUSY++;
		if(USART_RX_BUSY >= USART_RXBUF_LEN)USART_RX_BUSY=0;
		return tmp;
	}else return 0;
}

uint8_t USART_getline(char *buf)
{
	static uint8_t bf[300]; //bufor pomocniczy
	int i;

	// Pobieranie danych, jeżeli znajdują się w buforze
	while(USART_kbhit())
	{
		bf[dlugosc_ramki]=USART_getchar();

		// Wyświetlanie znaków w putty
		USART_fsend("%c",bf[dlugosc_ramki]);
		int len_ramki=0;

		if(bf[dlugosc_ramki]==60)
		{
			// Pomijanie znaku w zapisie danych do tablicy
			//sprawdzenie czy odnaleziono znak "<"
			dlugosc_ramki=0;
			znak_poczatku=1;
		}
		if(bf[dlugosc_ramki]==62 && znak_poczatku==1)
		{
			//jesli podalismy wiecej znakow niz minimalna dlugosc ramki
			if(dlugosc_ramki >= 12)
			{	// Zwracanie pobranych znaków do tablicy użytej jako argument funkcji
				for(i=0;i<=dlugosc_ramki;i++)
				{
					buf[i]=bf[i];//zapis do bufora
				}
				len_ramki=dlugosc_ramki;
				dlugosc_ramki-0;
				znak_poczatku=0;
				return len_ramki;// Zwracanie ilości odebranych znaków
			}
			dlugosc_ramki = 0;
			znak_poczatku = 0;
		}else
		{
			dlugosc_ramki++;

			if(dlugosc_ramki>=269)
			{
				dlugosc_ramki=0;
				znak_poczatku=0;
			}
		}
	}
		return 0;
}//USART_getline

void USART_fsend(char* format,...)
{
	char tmp_rs[128];
	int i;
	__IO int idx;
	va_list arglist;
	va_start(arglist,format);
	vsprintf(tmp_rs,format,arglist);
	va_end(arglist);
	idx=USART_TX_EMPTY;

	for(i=0;i<strlen(tmp_rs);i++)
	{
		USART_TxBUF[idx]=tmp_rs[i];
		idx++;
		if(idx>=USART_TXBUF_LEN)idx=0;
	}
	__disable_irq(); //wyłączenie przerwań
	if((USART_TX_EMPTY==USART_TX_BUSY)&&(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_TXE)==SET))
	{
		USART_TX_EMPTY=idx;
		uint8_t tmp=USART_TxBUF[USART_TX_BUSY];
		USART_TX_BUSY++;
		if(USART_TX_BUSY>=USART_TXBUF_LEN)USART_TX_BUSY=0;
		HAL_UART_Transmit_IT(&huart2,&tmp,1);
	}else
	{
		USART_TX_EMPTY=idx;
	}
	__enable_irq(); //włączenie przerwań
}
//nadawanie
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2)
	{
		if(USART_TX_EMPTY!=USART_TX_BUSY)
		{
			uint8_t tmp=USART_TxBUF[USART_TX_BUSY];
			USART_TX_BUSY++;
			if(USART_TX_BUSY>=USART_TXBUF_LEN)USART_TX_BUSY=0;
			HAL_UART_Transmit_IT(&huart2,&tmp,1);
		}
	}
}

//odbiór
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart==&huart2)
	{
		USART_RX_EMPTY++;
		if(USART_RX_EMPTY>=USART_RXBUF_LEN)
			{
				USART_RX_EMPTY=0;
			}
		HAL_UART_Receive_IT(&huart2,&USART_RxBUF[USART_RX_EMPTY],1);
	}
}

void pobierz_dane(char *buf)
{
	memcpy(nadawca, &buf[1], 3);
    memcpy(odbiorca, &buf[4], 3);
    memcpy(dlugosc_komendy, &buf[7], 3);
    for (int i = 0; i < 3; i++)
    {
    	if (!isdigit(dlugosc_komendy[i]))
    	{
    		USART_fsend("\n lecom \r \n");
    		return;
    	}
    }
    com_len = atoi(dlugosc_komendy);
    memcpy(suma_kontrolna, &buf[dlugosc_ramki-2],2);
    for (int i = 0; i < 2; i++)
    {
    	if (!isdigit(suma_kontrolna[i]))
    	{
    		USART_fsend("\n wsum \r \n");
    		return;
    	}
    }
    kontrolna_suma=atoi(suma_kontrolna);
    sprawdz_dane(buf);
}

void sprawdz_dane(char *buffer)
{
	if (strncmp(adres_stm, odbiorca, 3) == 0)
	{
		if (com_len <= 256 && com_len > 0)
		{
			int poprawna_ramka = 12 + com_len;

			if (dlugosc_ramki <= poprawna_ramka)
			{
				USART_fsend("\n lecom \r\n");
				return;
			}
			char cmd[com_len + 1];
			memcpy(cmd, &buffer[10], com_len);//wpisuje ciag znakow o dlugosci komenda dlugosc zaczynajac od 10 miejsca w buforze
			analizuj(cmd);
		}else if (com_len == 0)
		{
			int poprawna_ramka = 12 + com_len;
			if (dlugosc_ramki != poprawna_ramka) {
			USART_fsend("ledata\r\n");
			return;
		}else
		{
			USART_fsend("\n miscom \r\n");
			return;
		}
	}else
	{
		USART_fsend("\n lecom \r\n");
	}
}else
{
	USART_fsend("\n wframe \r\n");
}
}



void analizuj(char *cmd)
{
	int sum = 0;

	for(int i=0; i<com_len;i++)
	{
		sum=sum+cmd[i];
	}

	sum=sum%100;

	if (kontrolna_suma=sum)
    	{
			//interwał pomiarowy
    		if (cmd[0]=='s' && cmd[1]=='t' && cmd[2]=='a' && cmd[3]=='r' && cmd[4]=='t' && cmd[5]=='i' && cmd[6]=='n' && cmd[7]=='t' && com_len ==14)
    		{
    			if(cmd[8]=='[' && cmd[13]==']')
    			{
    				char interwal[3];
    				memcpy(interwal,&cmd[9],4);

    				for (int i=0; i<4;i++)
    				{
    					if(!isdigit(interwal[i]))
    				    {
    				    	USART_fsend("\n wcom \r\n %c", interwal[i]);
    				    	return;
    				    }
    				}
    				czas=atoi(interwal);
    				odczyt = 1;
    			}
    			if(cmd[13]==']')
    			{
    				USART_fsend("\n %d \r\n");
    			}else
    			{
    				USART_fsend("\n wcom \r\n");
    			}
    		}else if (cmd[0] == 'a' && cmd[1] == 'r' && cmd[2] == 'c' && cmd[3] == 'h' && cmd[4] == 'o' && cmd[5] == 'd' && com_len == 15)
    		{
    			if(cmd[6]=='[' && cmd[14]==']')
    			{
    				odczyt = 0;
    				char od[2];
    				int dokad_sprawdzac = 0;
    				int odczego_sprawdzac = 0;
    				memcpy(od, &cmd[7], 4);

    				for (int i = 0; i < 3; i++)
    				{
    					if (!isdigit(od[i]))
    					{
    						USART_fsend("wcom \r \n %c",od[i]);
    						return;
    					}
    					odczego_sprawdzac = atoi(od);//USART_fsend(" od czego : %hd \r \n ",odczego_sprawdzac);
    				}
    				char dokad[2];
    				memcpy(dokad, &cmd[12], 4);

    				for (int i = 0; i < 3; i++)
    				{
    					if (!isdigit(dokad[i]))
    					{
    						USART_fsend("wcom \r \n %c",dokad[i]);
    						return;
    					}
    					dokad_sprawdzac = atoi(dokad);// USART_fsend(" dokad_sprawdzac : %hd \r \n ",dokad_sprawdzac);
    				}
    				for(int i=odczego_sprawdzac ; i<=dokad_sprawdzac;i++)
    				{
    					if(bufor[i] == 0)
    					{
    						USART_fsend("misrec %d \r \n",i);
    						//return;
    					}else
    					{
    						USART_fsend("%d. %0.2f \r \n",i,bufor[i]);
    					}
    				}
    			}else
        		{
        			USART_fsend("\n wcom \r\n");
        		}
    		}else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'o' && cmd[3] == 'p' && com_len == 4)
    		{
    			// zatrzymanie czujnika(stopuje mierzenie temperatury,cisnienia i wilgotnosci)
    			odczyt = 0;
    			USART_fsend("Zatrzymano \r \n");
    		}else if (cmd[0] == 's' && cmd[1] == 't' && cmd[2] == 'a' && cmd[3] == 'r' && cmd[4] == 't' && com_len == 5)
    		{
    			//Wyswietlanie wszystkiego
    			odczyt = 1;
    			czas = 1000;
    		}else if (cmd[0] == 'o' && cmd[1] == 'f' && cmd[2] == 'f' && cmd[3] == 'p' && cmd[4] == 'o' && cmd[5] == 'm' && com_len == 6 )
    		{
    			//cisnienie i temperatura:
    			pom = 0;
    		}else if (cmd[0] == 'o' && cmd[1] == 'n' && cmd[2] == 'p' && cmd[3] == 'o' && cmd[4] == 'm' && com_len == 5 )
    		{
    			pom = 1;
    			czas = 1000;
    		}else if (cmd[0] == 'o' && cmd[1] == 'f' && cmd[2] == 'f' && cmd[3] == 'w' && cmd[4] == 'i' && cmd[5] == 'l' && cmd[6] == 'g' && com_len == 7 )
    		{
    			//wilgotnosc
    			wilg = 0;
    		}else if (cmd[0] == 'o' && cmd[1] == 'n' && cmd[2] == 'w' && cmd[3] == 'i' && cmd[4] == 'l' && cmd[5] == 'g' && com_len == 6)
    		{
    			wilg= 1;
    			czas = 1000;
    			//arch
    		}else if (cmd[0] == 'a' && cmd[1] == 'r' && cmd[2] == 'c' && cmd[3] == 'h' && com_len == 4)
    		{
    			odczyt = 0;
    			archiwalne();
    		}else
    		{
    			USART_fsend("\n wcom \r\n ");
    			return;
    		}
    	}else
    	{
    		USART_fsend("\n wsum \r\n");
    	}
}

void archiwalne ()
{
	//wyświetlanie danych archiwalnych przechowywanych przez czujnik
	int j = 0;
	USART_fsend("arch[\r\n");
	for( int i=0; i< bufor_len ; i=i+3 )
	{
		if(bufor[i]==0)
		{
			return;
		}else
		{
			USART_fsend("%d. %0.2f , %0.2f , %0.2f ]\r\n",j,bufor[i],bufor[i+1],bufor[i+2]);
			j++;
		}
	}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2,&USART_RxBUF[0],1);
      //uint32_t xxx=0;
    int len=0;
    char bx[500];
  while (1)
  	  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  	if ((len = USART_getline(bx)) > 0)
	  	{
	  		pobierz_dane(bx);
	  	}
	  	 if (__HAL_TIM_GET_COUNTER(&htim1) == (czas * 2))
	  	{
	  		if(odczyt == 1)
	  		{
	  			BME280_read_data();
	  			__HAL_TIM_SET_COUNTER(&htim1,0);
	  		}else if(wilg == 1)
	  		{
	  			BME280_read_data3();
	  			__HAL_TIM_SET_COUNTER(&htim1,0);
	  		}else if (pom == 1)
	  		{
	  			BME280_read_data2();
	  			__HAL_TIM_SET_COUNTER(&htim1,0);
	  		}
	  	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
}
  /* USER CODE END Error_Handler_Debug */


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
