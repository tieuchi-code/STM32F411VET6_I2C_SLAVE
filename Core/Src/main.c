#include "main.h"

#define GPIOA_BASE_ADDR	0x40020000
#define GPIOB_BASE_ADDR	0x40020400
#define GPIOC_BASE_ADDR	0x40020800
#define GPIOD_BASE_ADDR 0x40020C00
#define GPIOE_BASE_ADDR 0x40021000
#define GPIOH_BASE_ADDR 0x40021C00

#define USART1_BASE_ADDR 0x40011000
#define I2C1_BASE_ADDR 0x40005400
#define SPI1_BASE_ADDR 0x40013000
#define I2S1_BASE_ADDR 0x40013000

#define DMA1_BASE_ADDR 0x40026000

void I2C1_Slave_Init()	//PB6,PB7	SLAVE
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    // MODER: Alternate function (10)
    volatile uint32_t* MODER   = (uint32_t*)(GPIOB_BASE_ADDR + 0x00);
     *MODER &= ~((0b11 << (6 * 2)) | (0b11 << (7 * 2)));
     *MODER |=  ((0b10 << (6 * 2)) | (0b10 << (7 * 2)));

     // AFRL: AF4 for I2C
     volatile uint32_t* AFRL    = (uint32_t*)(GPIOB_BASE_ADDR + 0x20);
     *AFRL &= ~((0xF << (6 * 4)) | (0xF << (7 * 4)));
     *AFRL |=  ((0x4 << (6 * 4)) | (0x4 << (7 * 4)));

     // OTYPER: Open-drain
     volatile uint32_t* OTYPER  = (uint32_t*)(GPIOB_BASE_ADDR + 0x04);
     *OTYPER |= (1 << 6) | (1 << 7);

     // PUPDR: Pull-up
     volatile uint32_t* PUPDR   = (uint32_t*)(GPIOB_BASE_ADDR + 0x0C);
     *PUPDR &= ~((0b11 << (6 * 2)) | (0b11 << (7 * 2)));
     *PUPDR |=  ((0b01 << (6 * 2)) | (0b01 << (7 * 2)));

     // OSPEEDR: High speed
     volatile uint32_t* OSPEEDR = (uint32_t*)(GPIOB_BASE_ADDR + 0x08);
     *OSPEEDR &= ~((0b11 << (6 * 2)) | (0b11 << (7 * 2)));
     *OSPEEDR |= (0b11 << (6 * 2)) | (0b11 << (7 * 2));

     //Cấu hình I2C1
     volatile uint32_t* CR1   = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
     *CR1 &= ~(1 << 0);     // Disable I2C (PE = 0)

     *CR1 |= (1 << 10); // Bật ACK

     //Cấu hình địa chỉ Slave
     volatile uint32_t* OAR1   = (uint32_t*)(I2C1_BASE_ADDR + 0x08);
     *OAR1 |= (1 << 14);	//luôn giữ = 1
     *OAR1 &= ~(1 << 15);	// 7 bit address
     *OAR1 &= ~(0b1111111 << 1);	//Clear
     *OAR1 |= (0x42 << 1);			//Địa chỉ 0x42

     *CR1 |= (1 << 0);      // Enable I2C
}

uint8_t I2C1_Slave_Receive(void)
{
    volatile uint32_t* CR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x00);
    volatile uint32_t* SR1 = (uint32_t*)(I2C1_BASE_ADDR + 0x14);
    volatile uint32_t* SR2 = (uint32_t*)(I2C1_BASE_ADDR + 0x18);
    volatile uint32_t* DR  = (uint32_t*)(I2C1_BASE_ADDR + 0x10);

    // 1. Đợi Master
    while (!(*SR1 & (1 << 1)));  // ADDR = 1
    (void)*SR2;                  // clear cờ ADDR

    // 2. Đợi dữ liệu
    while (!(*SR1 & (1 << 6)));	//RxNE = 1
    uint8_t received_data = *DR;

    // Đợi Master gửi STOP
    while (!(*SR1 & (1 << 4)));   // STOPF = 1
    *CR1 |= (1 << 0);             // Clear STOPF bằng cách ghi lại PE

    return received_data;
}

int main()
{
	HAL_Init();
	I2C1_Slave_Init();
	I2C1_Slave_Receive();
	while(1)
	{

	}
}
