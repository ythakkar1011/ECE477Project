#include "stm32f0xx.h"

#include "lcd.h"

#include "ff.h"

#include "diskio.h"

#include "fifo.h"

#include "tty.h"

#include <string.h> // for memset()

#include <stdio.h> // for printf()



// Be sure to change this to your login...



// Prototypes for miscellaneous things in lcd.c

void nano_wait(unsigned int);



// Write your subroutines below



void small_delay()

{

    nano_wait(1000000);

}




//============================================================================

// Parameters for the wavetable size and expected synthesis rate.

//============================================================================

#define N 1000

#define RATE 20000

short int wavetable[N];



//============================================================================

// init_wavetable()    (Autotest #2)

// Write the pattern for one complete cycle of a sine wave into the

// wavetable[] array.

// Parameters: none

//============================================================================





//============================================================================

// Global variables used for four-channel synthesis.

//============================================================================

int volume = 2048;

int stepa = 0;

int stepb = 0;

int stepc = 0;

int stepd = 0; // not used

int offseta = 0;

int offsetb = 0;

int offsetc = 0;

int offsetd = 0; // not used



void enable_ports()

{

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~(0xffff);

    GPIOB->MODER |= 0x55;

    GPIOB->PUPDR &= ~(0xff00);

    GPIOB->PUPDR |= 0xaa00;

}



char offset;

char history[16];

char display[8];

char queue[2];

int  qin;

int  qout;



//============================================================================

// show_digit()    (Autotest #4)

// Output a single digit on the seven-segment LED array.

// Parameters: none

//============================================================================



//============================================================================

// set_row()    (Autotest #5)

// Set the row active on the keypad matrix.

// Parameters: none

//============================================================================

void set_row()

{

    int row = offset & 3;

    GPIOB->BSRR = 0xf0000 | (1<<row);

}



//============================================================================

// get_cols()    (Autotest #6)

// Read the column pins of the keypad matrix.

// Parameters: none

// Return value: The 4-bit value read from PC[7:4].

//============================================================================

int get_cols()

{

    return (GPIOB->IDR >> 4) & 0xf;

}



//============================================================================

// insert_queue()    (Autotest #7)

// Insert the key index number into the two-entry queue.

// Parameters: n: the key index number

//============================================================================

void insert_queue(int n)

{

    queue[qin] = n | 0x80;

    if (qin == 0)

    {

        qin = 1;

    }

    else

    {

        qin = 0;

    }

}



//============================================================================

// update_hist()    (Autotest #8)

// Check the columns for a row of the keypad and update history values.

// If a history entry is updated to 0x01, insert it into the queue.

// Parameters: none

//============================================================================

void update_hist(int cols)

{

    int row = offset & 3;

    for(int i=0; i < 4; i++){

        history[4*row+i] = (history[4*row+i]<<1) + ((cols>>i)&1);

        if (history[4*row+i] == 0x1)

        {

            insert_queue(4*row+i);

        }

    }

}



//============================================================================

// Timer 7 ISR()    (Autotest #9)

// The Timer 7 ISR

// Parameters: none

// (Write the entire subroutine below.)

//============================================================================

void TIM7_IRQHandler()

{

    TIM7->SR &= ~TIM_SR_UIF;

    //show_digit();

    int cols = get_cols();

    update_hist(cols);

    offset = (offset + 1) & 0x7; // count 0 ... 7 and repeat

    set_row();

}



//============================================================================

// setup_tim7()    (Autotest #10)

// Configure timer 7.

// Parameters: none

//============================================================================

void setup_tim7()

{

    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM6EN);

    TIM7->PSC = 24000 - 1;

    TIM7->ARR = 2 - 1;

    TIM7->DIER |= TIM_DIER_UIE;

    TIM7->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM7_IRQn);

}



//============================================================================

// getkey()    (Autotest #11)

// Wait for an entry in the queue.  Translate it to ASCII.  Return it.

// Parameters: none

// Return value: The ASCII value of the button pressed.

//============================================================================

int getkey()

{

    while(queue[qout] == 0)

    {

        asm volatile("wfi");

    }



    char copy = queue[qout];

    queue[qout] = 0;

    qout = !qout;

    copy = copy & 0x7f;





    if (copy == 0)

        return '1';

    if (copy == 1)

        return '2';

    if (copy == 2)

        return '3';

    if (copy == 3)

        return 'A';

    if (copy == 4)

        return '4';

    if (copy == 5)

        return '5';

    if (copy == 6)

        return '6';

    if (copy == 7)

        return 'B';

    if (copy == 8)

        return '7';

    if (copy == 9)

        return '8';

    if (copy == 10)

        return '9';

    if (copy == 11)

        return 'C';

    if (copy == 12)

        return '*';

    if (copy == 13)

        return '0';

    if (copy == 14)

        return '#';

    if (copy == 15)

        return 'D';

}



//============================================================================

// This is a partial ASCII font for 7-segment displays.

// See how it is used below.

//============================================================================

const char font[] = {

        [' '] = 0x00,

        ['0'] = 0x3f,

        ['1'] = 0x06,

        ['2'] = 0x5b,

        ['3'] = 0x4f,

        ['4'] = 0x66,

        ['5'] = 0x6d,

        ['6'] = 0x7d,

        ['7'] = 0x07,

        ['8'] = 0x7f,

        ['9'] = 0x67,

        ['A'] = 0x77,

        ['B'] = 0x7c,

        ['C'] = 0x39,

        ['D'] = 0x5e,

        ['*'] = 0x49,

        ['#'] = 0x76,

        ['.'] = 0x80,

        ['?'] = 0x53,

        ['b'] = 0x7c,

        ['r'] = 0x50,

        ['g'] = 0x6f,

        ['i'] = 0x10,

        ['n'] = 0x54,

        ['u'] = 0x1c,

};



// Shift a new character into the display.



// Read an entire floating-point number.


void setup_USART5(void)

{

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIODEN;

    GPIOC->MODER &= ~(0x3000000);

    GPIOC->MODER |= 0x2000000;

    GPIOC->AFR[1] |= 0x20000;

    GPIOD->MODER &= ~(0x30);

    GPIOD->MODER |= 0x20;

    GPIOD->AFR[0] |= 0x200;

    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    USART5->CR1 &= ~(USART_CR1_UE);

    USART5->CR1 &= ~USART_CR1_M;

    USART5->CR2 &= ~USART_CR2_STOP;

    USART5->CR1 &= ~USART_CR1_PCE;

    USART5->CR1 &= ~USART_CR1_OVER8;

    USART5->BRR = 0x1A1;

    USART5->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    USART5->CR1 |= USART_CR1_UE;

    while ((USART5->ISR & USART_ISR_TEACK) == 0);

    while ((USART5->ISR & USART_ISR_REACK) == 0);

}

void setup_USART3(){

//Enable the RCC clocks to GPIOC and GPIOD.

    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

//RCC->AHBENR |= RCC_AHBENR_GPIODEN;

//configure pin PC5 to be routed to USART3_RX.

//configure pin PC4 to be routed to USART3_TX.

    GPIOC->MODER &= ~0xf00000;

    GPIOC->MODER |= 0xa00000;

    GPIOC->AFR[1] |= 0x1100;



    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;



    USART3->CR1 &= ~USART_CR1_UE;

    USART3->CR1 &= ~USART_CR1_M;

    USART3->CR2 &= ~USART_CR2_STOP;

    USART3->CR1 &= ~USART_CR1_PCE;

    USART3->CR1 &= ~USART_CR1_OVER8;

    USART3->BRR = 0x34;

    USART3->CR1 |= (USART_CR1_TE | USART_CR1_RE);

    USART3->CR1 |= USART_CR1_UE;

    USART3->CR1  |= USART_CR1_RXNEIE;    // UART3 Receive Interrupt Enable.
         // Enable interrupt fromUSART1(NVIC level)
    NVIC->ISER[0] = (1<<29);

    //while ((USART3->ISR & USART_ISR_TEACK) == 0);

    //while ((USART3->ISR & USART_ISR_REACK) == 0);

}


void USART5_IRQHandler()

{

    USART_TypeDef *u = USART5;

    if(u->ISR & USART_ISR_ORE)

    {

        u->ICR |= USART_ICR_ORECF;

    }


    int x = USART5->RDR;

    if(fifo_full(&input_fifo) == 1)

    {

        return;

    }


}

/*

#define BufferSize 2048

uint8_t USART3_Buffer_Rx[BufferSize];

volatile uint32_t Rx3_Counter = 0;

void USART3_IRQHandler()
{


    printf("RDR: %d", USART3->RDR);

    if(USART3->ISR & USART_ISR_RXNE){
        printf("RDR: %d", USART3->RDR);

        USART3_Buffer_Rx[Rx3_Counter] = USART3->RDR;

        ++Rx3_Counter;

        if(Rx3_Counter >= BufferSize) Rx3_Counter = 0;


    }

}
*/


char USART3_Buffer_Rx[6] = "";

int counter = 0;


void USART3_IRQHandler()

{

    //printf("Enter in the USART3 IRQ\n");

    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET){
        //printf("%c",USART_ReceiveData(USART3));

        char ch = USART_ReceiveData(USART3);
        strncat(USART3_Buffer_Rx, &ch, 1);
        ++counter;

    }

    if (counter == 3){
        //printf("%s", USART3_Buffer_Rx);

        FIL rssiFile;
        f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);
        f_printf(&rssiFile, "%s", USART3_Buffer_Rx);
        f_close(&rssiFile);
        //f_printf(&rssiFile, "%s", USART3_Buffer_Rx);

        LCD_DrawString(70,150, BLACK, BLACK, "Deselect Button", 16, 0);

        LCD_DrawString(50,150, WHITE, BLACK, "RSSI Value: ", 16, 0);

        LCD_DrawString(140,150, WHITE, BLACK, USART3_Buffer_Rx , 16, 0);

        USART3_Buffer_Rx[0] = '\0';

        counter = 0;
    }

}

void advance_fattime(void);

void setup_spi2()
{
    /*RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(0xfc0c);
    GPIOA->MODER |= 0xa804;
    GPIOA->AFR[0] |= 0x00000000;
    GPIOA->PUPDR &= ~(0x3000);
    GPIOA->PUPDR |= 0x1000;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    SPI1->CR1 |= (SPI_CR1_BR);
    SPI1->CR1 &= ~(SPI_CR1_SPE);
    SPI1->CR1 &= ~(SPI_CR1_BIDIMODE);
    SPI1->CR1 &= ~(SPI_CR1_BIDIOE);
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_NSSP | SPI_CR2_FRXTH;
    //SPI1->CR2 |= SPI_CR2_FRXTH;
    SPI1->CR1 |= SPI_CR1_SPE;*/

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER12);
    GPIOA->MODER |= GPIO_MODER_MODER12_0;
    GPIOA->PUPDR &= ~(0x3000);
    GPIOA->PUPDR |= 0x1000;
    GPIOB->MODER &= ~ (GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
    GPIOB->MODER |= (GPIO_MODER_MODER15_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER13_1);
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFR15 |GPIO_AFRH_AFR14 | GPIO_AFRH_AFR13);
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    SPI2->CR1 |= (SPI_CR1_BR);
    SPI2->CR1 &= ~(SPI_CR1_SPE);
    SPI2->CR1 &= ~(SPI_CR1_BIDIMODE);
    SPI2->CR1 &= ~(SPI_CR1_BIDIOE);
    SPI2->CR1 |= SPI_CR1_MSTR;
    SPI2->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_NSSP | SPI_CR2_FRXTH;
    SPI2->CR1 |= SPI_CR1_SPE;
}

void spi_high_speed()
{
    /*SPI1->CR1 &= ~SPI_CR1_SPE;
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= (SPI_CR1_BR_1);
    SPI1->CR1 |= SPI_CR1_SPE;*/

    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 &= ~(SPI_CR1_BR);
    SPI2->CR1 |= (SPI_CR1_BR_1);
    SPI2->CR1 |= SPI_CR1_SPE;

}


void setup_spi1()

{

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER7 | GPIO_MODER_MODER3);

    GPIOA->MODER |= (GPIO_MODER_MODER5_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER3_0);

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFR5 | GPIO_AFRL_AFR7);

    GPIOB->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);

    GPIOB->MODER |= (GPIO_MODER_MODER10_0 | GPIO_MODER_MODER11_0);

    GPIOB->ODR |= (GPIO_ODR_10 | GPIO_ODR_11);

    GPIOA->ODR |= GPIO_ODR_3;

    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR2 = (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0);

    SPI1->CR1 |= (SPI_CR1_MSTR);

    SPI1->CR1 &= ~(SPI_CR1_BR);

    SPI1->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

    SPI1->CR1 |= SPI_CR1_SPE;

}


BYTE buffer[4096];
void file_write(){
    FATFS fs_storage;
    FATFS *fs = &fs_storage;


    //Give work area to storage drive
    //f_mount(fs, "", 1);
    //f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_ALWAYS);
    //f_close(&rssiFile);
    //f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);

}



void enable_tty_interrupt()

{

    //USART3->CR1 |= USART_CR1_RXNEIE;

    USART5->CR1 |= USART_CR1_RXNEIE;

    NVIC->ISER[0] = (1<<29);

}



int better_putchar(int x)

{

    if (x == 10)

    {

        while ((USART5->ISR & USART_ISR_TXE) == 0);

        USART5->TDR = 13;

        //USART5->TDR = 10;

    }



    while ((USART5->ISR & USART_ISR_TXE) == 0);

    USART5->TDR = x;

    return x;

}



int __io_putchar(int ch)

{

    return better_putchar(ch);

}



int __io_getchar(void)

{

    return line_buffer_getchar();

}





void internal_clock();

void demo();

void autotest();



extern const Picture *image;


// Write your subroutines below...
void setup_i2c()
{
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    GPIOB->MODER &= ~(0xf0000);
    GPIOB->MODER |= 0xa0000;
    GPIOB->AFR[1] |= 0x11;

    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->CR1 &= ~I2C_CR1_ANFOFF;
    I2C1->CR1 &= ~I2C_CR1_ERRIE;
    I2C1->CR1 &= ~I2C_CR1_NOSTRETCH;

    I2C1->TIMINGR &= ~I2C_TIMINGR_PRESC;
    I2C1->TIMINGR |= 0 << 28;
    I2C1->TIMINGR |= 3 << 20;
    I2C1->TIMINGR |= 1 << 16;
    I2C1->TIMINGR |= 3 << 8;
    I2C1->TIMINGR |= 9 << 0;

    I2C1->OAR1 &= ~I2C_OAR1_OA1EN;
    I2C1->OAR2 &= ~I2C_OAR2_OA2EN;

    I2C1->CR2 &= ~I2C_CR2_ADD10;
    I2C1->CR2 |= I2C_CR2_AUTOEND;
    I2C1->CR1 |= I2C_CR1_PE;
}

void i2c_waitidle(void)
{
    while((I2C1->ISR & I2C_ISR_BUSY) == I2C_ISR_BUSY);
}

void i2c_start(uint32_t devaddr, uint8_t size, uint8_t dir)
{
    uint32_t tmpreg = I2C1->CR2;
    tmpreg &= ~(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP);
    if (dir == 1)
        tmpreg |= I2C_CR2_RD_WRN;
    else
        tmpreg &= ~I2C_CR2_RD_WRN;
    tmpreg |= ((devaddr<<1) & I2C_CR2_SADD) | ((size << 16) & I2C_CR2_NBYTES);
    tmpreg |= I2C_CR2_START;
    I2C1->CR2 = tmpreg;
}

void i2c_stop()
{
    if (I2C1->ISR & I2C_ISR_STOPF)
        return;
    I2C1->CR2 |= I2C_CR2_STOP;
    while((I2C1->ISR & I2C_ISR_STOPF) == 0);
    I2C1->ICR |= I2C_ICR_STOPCF;
}

int8_t i2c_senddata(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0) return -1;
    uint8_t *udata = (uint8_t*)pdata;
    i2c_waitidle();
    i2c_start(devaddr, size, 0);

    for(i=0; i<size; i++)
    {
        int count = 0;
        while((I2C1->ISR & I2C_ISR_TXIS) == 0)
        {
            count += 1;
            if (count > 1000000) return -1;
            //if (i2c_checknack()) {i2c_clearnack(); i2c_stop(); return -1;}
        }
        I2C1->TXDR = udata[i] & I2C_TXDR_TXDATA;
    }

    while ((I2C1->ISR & I2C_ISR_TC) == 0 && (I2C1->ISR & I2C_ISR_NACKF) == 0);

    if((I2C1->ISR & I2C_ISR_NACKF) != 0)
        return -1;
    i2c_stop();
    return 0;
}

int8_t i2c_recvdata(uint8_t devaddr, void *pdata, uint8_t size)
{
    int i;
    if (size <= 0 || pdata == 0) return -1;

    uint8_t *udata = (uint8_t*)pdata;
    i2c_waitidle();
    i2c_start(devaddr, size, 1);
    for (i=0; i < size; i++)
    {
        while((I2C1->ISR & I2C_ISR_RXNE) == 0);
        udata[i] = I2C1->RXDR & I2C_RXDR_RXDATA;
    }
    while((I2C1->ISR & I2C_ISR_TC) == 0);

    i2c_stop();

}

uint16_t i2c_fuel_gauge_init()
{
    uint8_t hold1 = 0x04;
    uint8_t hold2 = 0x05;
    int size1 = sizeof(hold1);
    int size2 = sizeof(hold2);
    uint8_t voltage1;
    uint8_t voltage2;
    int sizeV1 = sizeof(voltage1);
    int sizeV2 = sizeof(voltage2);
    i2c_senddata(0x55, &hold1, size1);
    i2c_senddata(0x55, &hold2, size2);
    i2c_recvdata(0x55, &voltage1, sizeV1);
    i2c_recvdata(0x55, &voltage2, sizeV2);
    uint16_t voltage = voltage1<<8 | voltage2;
    return voltage;
}

uint16_t i2c_fuel_gauge_percent()
{
    uint8_t hold1 = 0x1c;
    uint8_t hold2 = 0x1d;
    int size1 = sizeof(hold1);
    int size2 = sizeof(hold2);
    uint8_t voltage1;
    uint8_t voltage2;
    int sizeV1 = sizeof(voltage1);
    int sizeV2 = sizeof(voltage2);
    i2c_senddata(0x55, &hold1, size1);
    i2c_senddata(0x55, &hold2, size2);
    i2c_recvdata(0x55, &voltage1, sizeV1);
    i2c_recvdata(0x55, &voltage2, sizeV2);
    uint16_t voltage = voltage1<<8 | voltage2;
    return voltage;
}

int main(void)

{

    //Test

    setup_USART5();

    setup_USART3();

    setbuf(stdin, 0);

    setbuf(stdout, 0);

    setbuf(stderr, 0);


    //internal_clock();

    //demo();

    //autotest();

    enable_ports();

    GPIOC->MODER &= ~(0xc000000);

    GPIOC->MODER |= 0x4000000;


    //GPIOC->MODER &= ~(0x3000);

    //GPIOC->MODER |= 0x1000;

    //GPIOC->BSRR = (1U << 6);


    setup_tim7();

    setup_spi1();

    setup_spi2();

    spi_high_speed();

    uint16_t voltage;
    setup_i2c();


    //file_write();

    FATFS fs_storage;
    FATFS *fs = &fs_storage;
    FIL rssiFile;

    //Give work area to storage drive
    f_mount(fs, "", 1);
    f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_ALWAYS);
    //f_printf(&rssiFile, "RSSI Value = -52!!!\n");
    f_close(&rssiFile);
    f_unlink("RSSI.txt");


    LCD_Init(0,0,0);

    LCD_Clear(BLACK);

    LCD_DrawString(10,100,  WHITE, BLACK, "Press Initialization Button", 16, 0);

    char first = getkey();

    while(first != 'B'){
        first = getkey();
    }

    LCD_Clear(BLACK);

    //LCD_DrawLine(10,20,100,200, WHITE);

    //LCD_DrawRectangle(10,20,100,200, GREEN);

    //LCD_DrawFillRectangle(120,20,220,200, RED);

    //LCD_Circle(50, 260, 50, 1, BLUE);

    //LCD_DrawFillTriangle(130,130, 130,200, 190,160, YELLOW);

    //LCD_DrawChar(150,155, BLACK, WHITE, 'X', 16, 1);

    //LCD_DrawString(70,100,  WHITE, BLACK, "Please Select", 16, 0);

    //LCD_DrawString(80,120,  WHITE, BLACK, "Your Wi-Fi", 16, 1);

    //LCD_DrawString(90,140, WHITE, BLACK, "Network", 16, 0);

    //LCD_DrawPicture(110,220,(const Picture *)&image);

    int x = 0;

    char writex = "0";

    int y = 0;

    char writey = "0";

    int counterForSerial = 0;

    for(;;) {

        char key = getkey();

        if (key == 'A'){

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Transfer Button", 16, 0);

            counterForSerial++;

            if (counterForSerial == 5){
                break;
            }

        }

        else if (key == 'B'){

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Initialization Button", 16, 0);

            counterForSerial = 0;

        }

        else if (key == '2'){

            voltage = i2c_fuel_gauge_init();//i2c_fuel_gauge_percent();

            float printVoltage = ((float) voltage)/1000;

            char batteryBuffer[20] = "";

            sprintf(batteryBuffer, "Voltage: %.3lf V", printVoltage);

            LCD_DrawString(50 ,230,  WHITE, BLACK, batteryBuffer, 16, 0);

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Up Button", 16, 0);

            y++;

            counterForSerial = 0;

            GPIOC->BSRR = (1U << 13);

            f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);
            f_printf(&rssiFile, "\n%d,%d,", x, y);//f_printf(&rssiFile, "\nX: %d, Y: %d RSSI Value:", x, y);
            f_close(&rssiFile);

                //nano_wait(100000000);

        }

        else if (key == '3'){

            voltage = i2c_fuel_gauge_init();//i2c_fuel_gauge_percent();

            float printVoltage = ((float) voltage)/1000;

            char batteryBuffer[20] = "";

            sprintf(batteryBuffer, "Voltage: %.3lf V", printVoltage);

            LCD_DrawString(50 ,230,  WHITE, BLACK, batteryBuffer, 16, 0);

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Right Button", 16, 0);

            x++;

            counterForSerial = 0;

            GPIOC->BSRR = (1U << 13);

            f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);
            f_printf(&rssiFile, "\n%d,%d,", x, y);//f_printf(&rssiFile, "\nX: %d, Y: %d RSSI Value:", x, y);
            f_close(&rssiFile);

                //nano_wait(100000000);

        }

        else if (key == '5'){
            voltage = i2c_fuel_gauge_init();//i2c_fuel_gauge_percent();

            float printVoltage = ((float) voltage)/1000;

            char batteryBuffer[20] = "";

            sprintf(batteryBuffer, "Voltage: %.3lf V", printVoltage);

            LCD_DrawString(50 ,230,  WHITE, BLACK, batteryBuffer, 16, 0);

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Left Button", 16, 0);

            x--;

            counterForSerial = 0;

            GPIOC->BSRR = (1U << 13);

            f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);
            f_printf(&rssiFile, "\n%d,%d,", x, y);//f_printf(&rssiFile, "\nX: %d, Y: %d RSSI Value:", x, y);
            f_close(&rssiFile);

                //nano_wait(100000000);

        }

        else if (key == '6'){
            voltage = i2c_fuel_gauge_init();//i2c_fuel_gauge_percent();

            float printVoltage = ((float) voltage)/1000;

            char batteryBuffer[20] = "";

            sprintf(batteryBuffer, "Voltage: %.3lf V", printVoltage);

            LCD_DrawString(50 ,230,  WHITE, BLACK, batteryBuffer, 16, 0);

            LCD_DrawString(70,100,  BLACK, BLACK, "Initialization Button", 16, 0);

            LCD_DrawString(70,100,  WHITE, BLACK, "Down Button", 16, 0);

            y--;

            counterForSerial = 0;

            GPIOC->BSRR = (1U << 13);

            f_open(&rssiFile, "RSSI.txt", FA_WRITE | FA_OPEN_APPEND);
            f_printf(&rssiFile, "\n%d,%d,", x, y);//f_printf(&rssiFile, "\nX: %d, Y: %d RSSI Value:", x, y);
            f_close(&rssiFile);

                //nano_wait(100000000);

        }

        //printf("\nX: %d, Y: %d\nRSSI Value: ", x, y);



        GPIOC->BSRR = (1U << 29);

    }

    char line[100];
    f_open(&rssiFile, "RSSI.txt", FA_READ);
    while (f_gets(line, sizeof(line), &rssiFile)){
        printf(line);
    }

    f_close(&rssiFile);

    f_unlink("RSSI.txt");

}
