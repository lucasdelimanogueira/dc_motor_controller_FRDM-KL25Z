#include <MKL25Z4.h>

//botao em PORTA PTA1
#define BUTTON 1

//encoder em PORTD PTD X
#define ENCODER 2


class Motor{
	private:
		float velocidade;
		int sentido;
		
	public:
		void motorInit(){
			
			//CONFIGURANDO PINOS SENTIDO MOTOR///////
			
			//ativar clock no PORTB
			SIM_SCGC5 |= (1<<10);
			
			//configurar pinos sentido motor como GPIO
			PORTB_PCR8 |= (1<<8);
			PORTB_PCR11 |= (1<<8);
			
			
			//configurar pinos sentido motor como saída (pinos PTB8,9,10,11)
			PTB->PDDR |= (1<<8);
			PTB->PDDR |= (1<<11);
			
		  ///////////////////////////////////////////
			
			
			
			
			//CONFIGURANDO PINO PWM////////
			
			
			//ativar clock no modulo do PWM TPM1 (bit 25)
			SIM_SCGC6 |= (1<<25);
			
			//configurar pino PWM (MUX = -> ALT3 (PTB0 TPM1 CH0)
			PORTB_PCR0 |= (0<<0)|(1<<9)|(1<<8);
			
			//configurando pino como saída
			PTB->PDDR |= (1<<0);
			
			//configurar qual fonte do clock para o TPM (no caso, MCGFLLCLK)
			SIM_SOPT2 |= (1<<24);
			
			//configurações do modulo PWM
			TPM1_SC |= (1<<5);
			TPM1_SC |= (0<<4)|(1<<3);
			TPM1_SC |= (0<<2)|(1<<1)|(1<<0);	

			//PWM de 12 bit
			TPM1_MOD = 0x0FFF;
			
			TPM1_C0SC |= (1<<5)|(0<<4)|(1<<3)|(0<<2);
			
			/////////////////////////////
			
			Motor::setSentido(1);	
		}
	
		
		void brakeMotor(){
			PTB->PCOR = (1<<8);
			PTB->PCOR = (1<<11);
		}
	
		void setSentido(int s){
			Motor::sentido = s;
			
			if(Motor::sentido == 1){
				PTB->PSOR = (1<<8);
				PTB->PCOR = (1<<11);			
			}
			else{
				PTB->PCOR = (1<<8);
				PTB->PSOR = (1<<11);
			}
		}
		
		int getSentido(){
			return Motor::sentido;
		}
	
};


class Encoder{
	private:
		float setPoint, velMeasured;
	
	public:
		void encoderInit(){
			//ativar clock no PORTD
			SIM_SCGC5 |= (1<<12);
			
			//configurar port como GPIO
			PORTD_PCR0 |= (1<<8);
			
			//habilitando resistor de pull up
			PORTD_PCR0 |= (1<<1);
	
			//habilitando interrupção do pino do encoder
			PORTD_PCR0 |= (0<<16)|(1<<17)|(0<<18)|(1<<19);		
			NVIC_EnableIRQ(PORTD_IRQn);				
		
			Encoder::setVelMeasured(0, 1);
		}
		void setVelMeasured(int p, int t){
			Encoder::velMeasured = ((4*1000*p)/t); //pulsos por segundo	
		}
		float getVelMeasured(){
			return Encoder::velMeasured;
		}	
};

class Button{
	private:
		
	public:
		void buttonInit(){
			//ativar clock no PORTA (9)
			SIM_SCGC5 |= (1<<9);
			
			//configurar port como GPIO (8) (MUX)
			PORTA_PCR4 = (1<<8);
			PORTA_PCR5 = (1<<8);

			//habilitando interrupções nos botoes (IRQC)
			PORTA_PCR4 |= (0<<16)|(1<<17)|(0<<18)|(1<<19);
			PORTA_PCR5 |= (0<<16)|(1<<17)|(0<<18)|(1<<19);
			NVIC_EnableIRQ(PORTA_IRQn);	

			//habilitando resistor pull up (1) (PE)
			PORTA_PCR4 |= (1<<1);
			PORTA_PCR5 |= (1<<1);
		}
		
};

class Potentiometer{
	private:
		int setPoint;
	
	public:
		void potInit(){
			//ativar clock do AD Converter (ADC0) (27)
			SIM_SCGC6 |= (1<<27);
			
			//ativar clock no PORTC (11)
			SIM_SCGC5 |= (1<<11);
			
			//configura PTC0 como Analog
			PORTC_PCR0 = 0;
			
			//para o pino PTC0, não é preciso configurar o ALT, mas lembrar de configurar
			//caso mude de pino
			
			//por segunrança, apaga configurações prévias do ADC
			ADC0_CFG1 = 0;
			
			//12 bits (MODE(1)), clock principal (ADICLK(0)) dividido por 8 ADIV(3)
			ADC0_CFG1 = ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0) | ADC_CFG1_ADIV(1);
			
			//desabilitar modulo (31) para configurar
			ADC0_SC1A |= ADC_SC1_ADCH(31); 
			
			//conversão contínua, com amostra de 32 para média
			ADC0_SC3 = ADC_SC3_ADCO(1) | ADC_SC3_AVGE(1) | ADC_SC3_AVGS(11);
			
			//Canal 14 (que é o que o PTC0 conectado como default (ALT0)
			ADC0_SC1A = ADC_SC1_ADCH(14);	

			Potentiometer::setPoint = 0;
		}
		
		void setSetpoint(){
			//leitura do sinal analógico
			if(ADC0_SC1A & ADC_SC1_COCO_MASK){
				Potentiometer::setPoint = ADC0_RA;
			}
		}
		
		int getSetpoint(){
			return Potentiometer::setPoint;
		}

};



			