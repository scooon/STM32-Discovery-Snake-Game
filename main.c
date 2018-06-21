// Snake Game 
// v1.0.0
// Copyright 2018 - Maciej Bednarek


#include "main_base.h"

// Kordynata początkowa
int x = 120, y=160;

// Zmienne dotyczące pomiarów z ADC, przekształceń odczytów z joysticka oraz początkowych punktów losujących
int pomiar1,pomiar2,pomiar3,pomiar4,y1,y2,x1,x2,n,losx,losy;

int flaga=0, liczniczek=0;

// Tablice przechowujące położenie kartezjańskie węża w pikselach
int tabx[50];
int taby[50];

// Tablice przechowujące informacje o aktualnych koordynatach jabłka w układzie kartezjańskim, wyrażone w pikselach
int jabkox[50];
int jabkoy[50];

// Generowanie losowości położenia jabłka
static uint32_t miState;
int32_t los( void )
{
    miState ^= (miState << 13);
    miState ^= (miState >> 17);
    miState ^= (miState << 15);

    return (miState * 1332534557) & 0x7FFFFFFF;
}

void initseed( uint32_t seed )
{
    // Dla zera nie będzie działał, dlatego
    if (seed == 0)
        seed = 0x55aaff00;

    miState = seed;
}


// Zmienne dotyczące przerwania i timeTicka
int timeTick = 0;
__attribute__((aligned(0x100))) unsigned long vectors[256];


// Funkcja sysTicka
void SysTickIRQ () {
if(timeTick>10){
	timeTick = 0;
}

timeTick++;
}


// Funkcja odpowiadająca za reset gry po fail evencie
void resetgry()
{
	// Czyścimy tablicę do wartości powyżej rozdziałki ekranu, dzięki detekcji out of range, eliminujemy problem świecącego piksela w punkcie [0,0]
	for (int q = 0; q < 50; q++) {
		tabx[q]=5000;
		taby[q]=5000;
	}

	// Zerujemy długość węża oraz koordynaty x,y
	n=0;
	x=120;
	y=160;

	// Odświeżamy ekran
	Clear_And_Reload_Screen();
}


// Funkcja odpowiadająca za generowanie punktu zaczepienia jabłka
void japko()
{
	losx = los();
	losy = los();
	losx = losx%240;
	losy = losy%320;

}


int main(void) {

	// Init STM32 oraz LCD, zegara, HALa i peryferiów
		HAL_Init();
		SystemClock_Config();
		BSP_SDRAM_Init();
		LCD_Config();

	// Generujemy seeda	
		initseed(0);

	// Losujemy Jabłko
		japko();
	
	// Init Joystick

		RCC->APB2ENR|=RCC_APB2ENR_TIM1EN;
		GPIOA->AFR[1]|= 1; // 1 - Tim1_CH1
		GPIOA->AFR[1]|= 1 << 4; // 1 - Tim1_CH2
		GPIOA->MODER |= GPIO_Mode_AF << 16;
		GPIOA->MODER |= GPIO_Mode_AF << 18;
		GPIOA->PUPDR |= GPIO_PuPd_UP << 16;
		GPIOA->PUPDR |= GPIO_PuPd_UP << 18;

		GPIOA->MODER |=GPIO_Mode_AN<<(5*2);
		GPIOA->MODER |=GPIO_Mode_AN<<(7*2);
		GPIOC->MODER |=GPIO_Mode_AN<<(3*2);
		GPIOF->MODER |=GPIO_Mode_AN<<(6*2);
		RCC->APB2ENR|=RCC_APB2ENR_ADC1EN;
		RCC->APB2ENR|=RCC_APB2ENR_ADC3EN;
		ADC1->CR2=ADC_CR2_ADON;
		ADC3->CR2=ADC_CR2_ADON;
		ADC1->JSQR=2<<4*5 | 13<<3*5 | 5<<2*5 | 7<<1*5;
		ADC3->JSQR=4<<3*5;
		ADC1->SMPR1=7<<(13-10)*3;
		ADC1->SMPR2=7<<5*3 | 7<<7*3;
		ADC1->CR1|=ADC_CR1_SCAN|ADC_CR1_JEOCIE;
		ADC1->CR2|=ADC_CR2_JSWSTART;
		ADC3->CR2|=ADC_CR2_JSWSTART;
		NVIC->ISER[0]=0x40000;
		GPIOC->MODER |= GPIO_Mode_IN << (11*2);
		GPIOC->PUPDR |=GPIO_PuPd_UP<< (11*2);

		TIM1->PSC=0;
		TIM1->CNT=16;
		TIM1->SMCR=1;
		TIM1->CCMR1=0x3131;
		TIM1->CR1=1;

		__disable_irq();

		SysTick->LOAD=10000;
		SysTick->VAL=10000;
		unsigned long* addr=
		(unsigned long*) (SCB->VTOR);
		for (int a=0;a<256;a++)
		vectors[a]=addr[a];
		SCB->VTOR=(unsigned long)vectors;
		unsigned long* addrs= (unsigned long*)(SCB->VTOR+0x0000003C);
		*addrs=(unsigned long)SysTickIRQ;
		__enable_irq();
		SysTick->CTRL=7;
	

	// Setup GPIO
		RCC->AHB1ENR |= RCC_AHB1Periph_GPIOG;
		GPIOG->MODER |= GPIO_Mode_OUT << 28;// 1 - output
		GPIOG->OTYPER|= GPIO_OType_PP << 14;//0 - push-pull (def.)
		GPIOG->PUPDR |= GPIO_PuPd_NOPULL << 28;//0 - (default)
		RCC->AHB1ENR |= RCC_AHB1Periph_GPIOA;
		GPIOA->MODER |= GPIO_Mode_IN; // 0 - input (default)
		GPIOA->PUPDR |= GPIO_PuPd_NOPULL ; // 0 - (default)

	
		
	// Init gry, kierunek początkowy, długość węża, reset gry
		flaga=0;
		n=0;
		resetgry();
	
	while (1) {

		// Wykrywanie kolizji węża
		for (int q = 0; q < 50; q++) {
			if(tabx[q]==x && taby[q]==y){
				resetgry();
			}
		}
		
		// Generowanie nowego jabłka po zebraniu
		for (int v = 0; v < 3; v++) {
			if(jabkox[v]==x && jabkoy[v]==y){
				japko();
			}
		}

		
		// Wykrywanie kolizji ze ścianami oraz reset gry
		if(x==240){
			resetgry();
		}
		if(x==0){
			resetgry();
		}
		if(y==320){
			resetgry();
		}
		if(y==0){
			resetgry();
		}

		// Dodanie kolejnego kroku węża do jego śladu
		taby[n]=y;
		tabx[n]=x;

		
		// Warunkowanie kątów prostych podczas zmian kierunku przy użyciu kontrolera oraz podążania w zadanym kierunku
		if(flaga==0){
			y++;
		}else if(flaga==1){
			y--;
		}else if(flaga==2){
			y++;
		}else if(flaga==3){
			x--;
		}else if(flaga==4){
			x++;
		}
		
		// Warunek precyzujący czułość joga na zmianę kierunków
			
			// Dla Y
			if(y1<20){
				flaga=1;
			}else if(y1>300){
				flaga=2;
			}
			
			// Dla X
			if(x1<20){
				flaga=3;
			}else if(x1>200){
				flaga=4;
			}


		// Odświeżenie i czyszczenie ekranu przed wypełnieniem nowymi danymi
		Clear_And_Reload_Screen();
		
		// Pętla rysująca węża po zadanych współprzędnych
		for (int l = 0; l < 50; l++) {
			// Warunek wykrywania pustej tablicy po resecie
			if(l!=5000){
					unsigned short int *pixel = (unsigned short int*) (LCD_BUF + tabx[l] + taby[l] * 240);
					*pixel++ =   	((31 - ((x >> 4) & 0x1f)) << 6)
									| (((((240 - x) * 32 / 240)
											+ x / 240) & 0x3f) << 6)
									| ((((int)y >> 4) & 0x1f) <<6);

			}
		}

		//Pętla rysująca jabłko
		for (int yy = 0; yy < 3; yy++) {
			losy[yy]=(losy + yy);
			unsigned short int *pixel = (unsigned short int*) (LCD_BUF + losx + (losy + yy)) * 240);
			for (int xx = 0; xx < 3; xx++) {
					losx[xx]=(losx + xx);
					*pixel++ =   	((31 - ((losx >> 4) & 0x1f)) << 6)
									| (((xx + (((240 - losx)) * (32 - yy) / 240)
											+ losx * yy / 240) & 0x3f) << 6)
									| ((((losy + yy) >> 4) & 0x1f) <<6);



			}
		}
	
	


		// Kreowanie długości węża oraz skracanie po przemieszczeniu
		if(n==49){
			n=0;
		}else{
			n++;
		}

	}

}

void ADC_IRQHandler(void){
	// Pomiary z ADC joga
	pomiar1=ADC1->JDR1;
	pomiar2=ADC1->JDR2;
	pomiar3=ADC1->JDR3;
	pomiar4=ADC3->JDR1;
	
	// Interpretacja pomiarów i przypisanie do zmiennych
	y2=pomiar4/13;
	x2=pomiar3/18;
	y1=pomiar2/13;
	x1=pomiar1/18;
	
	ADC1->SR&=~ADC_SR_JEOC;
	ADC1->CR2|=ADC_CR2_JSWSTART;
	ADC3->CR2|=ADC_CR2_JSWSTART;
}


