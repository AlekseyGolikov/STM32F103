#include "NRF24.h"
//------------------------------------------------
extern SPI_HandleTypeDef hspi1;
//extern UART_HandleTypeDef huart1;

#define TX_ADR_WIDTH 5					// ширина адреса передатчика
#define TX_PLOAD_WIDTH 2        // ширина пакета данных
uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x78,0x78,0x78,0x78,0x78};    // массив с адресом передатчика
uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};   //
//------------------------------------------------
//функция задержки
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 9;
  /* Wait till done */
  while (micros--) ;
}
//--------------------------------------------------
//Команд чтения регистра NRF24
uint8_t NRF24_ReadReg(uint8_t addr)
{
  uint8_t dt=0, cmd;
  CS_ON;
  HAL_SPI_TransmitReceive(&hspi1,&addr,&dt,1,1000);
  if (addr!=STATUS)//если адрес равен адресу регистра STATUS то и возварщаем его состояние
  {
    cmd=0xFF;
    HAL_SPI_TransmitReceive(&hspi1,&cmd,&dt,1,1000);
  }
  CS_OFF;
  return dt;
}
//------------------------------------------------
//Команда записи в регистр NRF24
void NRF24_WriteReg(uint8_t addr, uint8_t dt)
{
  addr |= W_REGISTER;//включим бит записи в адрес
  CS_ON;    //включение NRF24 на прием команд по SPI
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//функция HAL_SPI_Transmit блокирует выполнение скрипта и отправляет адрес в шину
  HAL_SPI_Transmit(&hspi1,&dt,1,1000);//отправляет данные в шину
  CS_OFF;   
}
//-----------------------------------------------
//
void NRF24_ToggleFeatures(void)
{
  uint8_t dt[1] = {ACTIVATE};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  dt[0] = 0x73;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  CS_OFF;
}
//-----------------------------------------------
// Функция чтения из буфера
void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину
  HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//отправим данные в буфер
  CS_OFF;
}
//------------------------------------------------
// Функция записи в буфер
void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  addr |= W_REGISTER;//включим бит записи в адрес
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//отправим данные в буфер
  CS_OFF;
}
//------------------------------------------------
// Очистка буфера FIFO для приёма
void NRF24_FlushRX(void)
{
  uint8_t dt[1] = {FLUSH_RX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
// Очистка буфера FIFO для передачи
void NRF24_FlushTX(void)
{
  uint8_t dt[1] = {FLUSH_TX};
  CS_ON;
  HAL_SPI_Transmit(&hspi1,dt,1,1000);
  DelayMicro(1);
  CS_OFF;
}
//------------------------------------------------
// Функция включения режима приёма 
void NRF24L01_RX_Mode(void)
{
  uint8_t regval=0x00;
  regval = NRF24_ReadReg(CONFIG);
  //разбудим модуль и переведём его в режим приёмника, включив биты PWR_UP и PRIM_RX
  regval |= (1<<PWR_UP)|(1<<PRIM_RX);
  NRF24_WriteReg(CONFIG,regval);
  CE_SET;
  DelayMicro(150); //Задержка минимум 130 мкс
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//--------------------------------------------------
// Функция включения режима передачи 
void NRF24L01_TX_Mode(uint8_t *pBuf)
{
  NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
  CE_RESET;
  // Flush buffers
  NRF24_FlushRX();
  NRF24_FlushTX();
}
//--------------------------------------------------
void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)
{
  CE_RESET;
  CS_ON;
  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину
  DelayMicro(1);
  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//отправим данные в буфер
  CS_OFF;
  CE_SET;
}
//--------------------------------------------------
// Функция приёма данных
uint8_t* NRF24L01_Receive(void)
{
  uint8_t status=0x01;
  uint8_t *dt;

	status = NRF24_ReadReg(STATUS);

  DelayMicro(10);
  status = NRF24_ReadReg(STATUS);
  if(status & 0x40) //данные приняты
  {
    NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
    dt = RX_BUF;
    NRF24_WriteReg(STATUS, 0x40);
  }	
	return dt;
}
//--------------------------------------------------
// Инициализация модуля NRF24L01
void NRF24_ini(void)
{
		CE_RESET;            //установим контакт CE в низкий уровень (режим передатчика TX mode)
	DelayMicro(5000);    //применим задержку 2 мс
		NRF24_WriteReg(CONFIG, 0x0a);    
	DelayMicro(5000);    //Подождём ещё 2 милисекунд, чтобы передатчик включился.
		NRF24_WriteReg(EN_AA, 0x00);		
		NRF24_WriteReg(EN_RXADDR, 0x02);   
		NRF24_WriteReg(SETUP_AW, 0x03); 	 
		NRF24_WriteReg(SETUP_RETR, 0x5F); 
		NRF24_ToggleFeatures();
		NRF24_WriteReg(FEATURE, 0);    
		NRF24_WriteReg(DYNPD, 0);			 
		NRF24_WriteReg(STATUS, 0x70); //Reset flags for IRQ
		NRF24_WriteReg(RF_CH, 76); 
		NRF24_WriteReg(RF_SETUP, 0x00); //-18dB; 1Mbit/s
		NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
		NRF24_Write_Buf(RX_ADDR_P1, TX_ADDRESS, TX_ADR_WIDTH);
		NRF24_WriteReg(RX_PW_P1, TX_PLOAD_WIDTH); //Number of bytes in RX payload in data pipe 1
  NRF24L01_RX_Mode(); //пока уходим в режим приёмника
  //LED_ON;
				
}
//--------------------------------------------------
