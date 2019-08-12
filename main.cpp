#include <C:\Users\mecatronica\Documents\Lucas de Lima Sistemas Digitais 2\Projeto 2_Motor_DC\Motor.h>
#include <String.h>
#include <string>
#include <stdlib.h>



int countMS = 0; //contador milissegundos
int now, lastCount, lastCount2;
int pulsos = 0;
int state = 1;
int change;
int vel = 0;
int SP = 0;
float P,I,D,PID;
float kp = 1.0;
float ki = 0.000000001;
float kd = 0.1;
float lastVel = 0;
int tempo = 0;
float erro = 0;
float deltaTime = 0;
int testSet = 0;
int testsentido = 0;


//**************Comunicação UART**********************//

#define BAUD 57600 // Velocidade de transmissão da comunicação serial
#define OSR 8            // Número de leituras por bit (OverSampling Rate)
 
int clockUART;     // variável para guardar o valor do clock que chega na UART
char msg[100]; // string a ser enviada
int msgLength = 10; // comprimento da string
int msgPointer = 0; // ponteiro da string

void static uartInit(){
			int clockDIV; // variável auxiliar para definir o divisor do clock da UART
     
    // Acionamento dos clocks dos periféricos
  SIM_SOPT2 = SIM_SOPT2_UART0SRC(1); // define o clock como sendo o MCGFLLCLK
  SIM_SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK; // clock MCGFLLCLK
  SIM_SCGC4 |= SIM_SCGC4_UART0_MASK; // liga a UART0
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK; // liga a PORTA, que tem os pinos da UART
     
    // Mapeamento dos pinos utilizados
    PORTA_PCR1 |= PORT_PCR_MUX(2); // UART0 RX
    PORTA_PCR2 |= PORT_PCR_MUX(2); // UART0 TX
     
    // Configuração da UART0
    UART0_C2 &= ~ (UART0_C2_TE_MASK| UART0_C2_RE_MASK); // desabilita os pinos enquanto configura. Várias configurações só são permitidas com os pinos desabilitados
     
  // Habilita interrupções da UART0
    NVIC_EnableIRQ(UART0_IRQn);
   
    // Definição do divisor de clock.
    clockUART=DEFAULT_SYSTEM_CLOCK;//getMCGClock(); // calcula o valor do clock de entrado da UART
    //clockUART = 48000000;
    clockDIV = clockUART/( BAUD * OSR); // calcula o quanto tem que ser o divisor
    UART0_BDH = (clockDIV/256)%32; // Obtêm a parte mais significativa do divisor
  UART0_BDL = clockDIV%256; // Obtêm a parte menos significativa do divisor
  UART0_C4 = OSR-1; // define quantas vezes cada bit será lido
     
    //Configuração da transmissão
  UART0_C1 = 0; // padrão: define 8 bits de dados, 1 de parada, sem paridade
  UART0_C3 = 0x00; // tudo como dantes no quartel de abrantes
  //UART0_MA1 = 0x00;
  //UART0_MA1 = 0x00;
  UART0_S1 |= 0x1F; // Limpa os status antes de começar
  UART0_S2 |= 0xC0; //altera configurações do idle e break character !?
    //UART0_C2 |= (UART0_C2_RIE_MASK | UART0_C2_TCIE_MASK); // Habilita as interrupções de transmissão completada e recebeu dados
  UART0_C2 |= (UART0_C2_RIE_MASK ); // Habilita a interrupção de recepção
  UART0_C2 |= (UART0_C2_TE_MASK| UART0_C2_RE_MASK); // habilita os pinos de TX e RX
}

//Envia a string preparada. Basicamente habilita a interrupção transmissão finalizada e a interrupção faz o trabalho
void uartSendString(int state, int vel, int setPoint){
	
		vel = vel/20;
		setPoint = setPoint/20;
		//convertendo inteiros para char
		char stateC = state + 48;
	
		char VelC[5];
		int resto = 0;
		for(int i=3;i>=0;i--){
		resto = vel%10 + 48;
		VelC[i] = resto;
		vel = vel/10;
		}
		VelC[4] = 0;
		
		char setPointC[5];
		int resto2 = 0;
		for(int i=3;i>=0;i--){
		resto2 = setPoint%10 + 48;
		setPointC[i] = resto2;
		setPoint = setPoint/10;
		}
		setPointC[4] = 0;

		//formatando mensagem para o padrao e armazenando valores na string msg
		msg[0] = stateC;
	  msg[1] = 0;
		strcat(msg,"	");
		strcat(msg,VelC);
		strcat(msg,"	");
		strcat(msg,setPointC);
		strcat(msg,"\n");
		

	
		msgLength = strlen(msg); //armazena em msgLength o tamanho da string msg
    msgPointer = 0; //Inicializa o ponteiro para o inicio da string
    UART0_D = msg[msgPointer++]; // Coloca a primeira letra da string para imprimir
    UART0_C2 |= ( UART0_C2_TCIE_MASK); // Habilita as interrupção de transmissão completada
}
 
// Variáveis que serão mexdas nas interrupções
static volatile char c; // o char recebido
static volatile int flagCommReceived,flagCommSend; // flags para a recepção e envio


//************************************************************************//







extern "C" {

	// função da interrupção da UART0
	void UART0_IRQHandler(){
    if (UART0_S1&UART_S1_RDRF_MASK){ // o RDRF indica que chegou algum dado
        c= UART0_D;                                      // então lê o que chegou
        flagCommReceived = 1;                    // e marca que recebeu
    }
    if ((UART0_S1&UART_S1_TDRE_MASK)){ // o TDRE indica que já foi enviado o dado
        if (msgPointer<msgLength){ // se ainda tem coisa para mandar,
            UART0_D = msg[msgPointer++]; // manda a próxima letra
        }else{ // se já mandou tudo,
    UART0_C2 &= ~(UART0_C2_TCIE_MASK); // desabilita a interrupções de transmissão completada
   
        }
    }
}
	
	
	//a cada milissegundo essa interrupção é chamada e acrescenta countMS
	void SysTick_Handler(void) {
		countMS++;
	}
	
	void PORTA_IRQHandler(void){
		//interrupção botoes
		
		//se R change = 1
		if((PORTA_PCR4&(1<<24)) == (1<<24)){
			change = 1;
			//desliga flag interrupção
			PORTA_PCR4 |= (1<<24);
		}
		
		//se S change = 2
		else if((PORTA_PCR5&(1<<24)) == (1<<24)){
			change = 2;
			//desliga flag interrupção
			PORTA_PCR5 |= (1<<24);
		}
		

		
		else {
			change = 0;
		}
	}
	
	//a cada pulso do encoder essa interrupção é chamada e acrescenta pulsos
	void PORTD_IRQHandler(void){
		if((PORTD_PCR0&(1<<24)) == (1<<24)){
			pulsos++;
			//desliga flag interrupção
			PORTD_PCR0 |= (1<<24);
		}
	}
}


int main(){
	//configurando clock
	SystemCoreClockUpdate();
	//configurando sysTick para ser chamado a cada SystemCoreClock/1000-1 (1 milissegundo)
	SysTick_Config(SystemCoreClock/1000-1);
	Motor motor1;
	Encoder encoder1;
	Button button1;
	Potentiometer pot1;
	pot1.potInit();
	
	motor1.motorInit();
	encoder1.encoderInit();
	button1.buttonInit();
	
	uartInit();

	
	lastCount = countMS;
	while(true){

		pot1.setSetpoint();
		testSet = pot1.getSetpoint();
		testsentido = motor1.getSentido();
		now = countMS;
			if(now-lastCount >= 1000){
			  tempo = now-lastCount;
				encoder1.setVelMeasured(pulsos, tempo);
				pulsos = 0;
				lastCount = countMS;	
				uartSendString(state, vel, SP);
			}			

			
			
		//maquina de estados
		

		//se botao R change = 1
		//se botao S change = 2
		
		//PARADO/CW
		switch(state){
			
			//PARADO/CW
			case 1:	
				
				//caso botao R
				if(change == 1){
					state=2;
					change=0;
				}
				
				//caso botao S
				else if(change == 2){
					state = 3;
					change = 0;
				}
			break;
			
			//OFF/CCW
			case 2:
				
				//caso botao R
				if(change == 1){
					state=1;
					change=0;
				}
				
				//caso botao S
				else if(change == 2){
					state = 4;
					change = 0;
				}
			break;
			
			//ACC/CW
			case 3:
				//define sentido de giro do motor
				motor1.setSentido(1);	
				change = 0;
				state = 5;
			break;
			
			//ACC/CCW
			case 4:	
				//define sentido de giro do motor
				motor1.setSentido(2);
				change = 0;
				state = 6;	
			break;
			
			//ON/CW
			case 5:
			
				//CONTROLE DO PWM COM PID
				lastVel = encoder1.getVelMeasured();
				pot1.setSetpoint();
				erro = pot1.getSetpoint() - encoder1.getVelMeasured();
				deltaTime = (countMS - lastCount2);
				P = erro*kp;
				I += erro*ki*deltaTime;
				D = ((lastVel - encoder1.getVelMeasured())*kd)/deltaTime;
				PID = P+I+D;
				if(PID>4095){
					TPM1_C0V = 4095;
				}
				else if(PID<0){
					TPM1_C0V = 0;
				}
				else{
					TPM1_C0V = PID;
				}
				
				//caso botao R
				if(change == 1){
					state=8;
					change=0;
				}
				
				//caso botao S
				else if(change == 2){
					state = 9;
					change = 0;
				}
			break;
			
			//ON/CCW
			case 6:

				//CONTROLE DO PWM COM PID
				lastVel = encoder1.getVelMeasured();
				pot1.setSetpoint();
				erro = pot1.getSetpoint() - encoder1.getVelMeasured();
				deltaTime = (countMS - lastCount2);
				P = erro*kp;
				I += erro*ki*deltaTime;
				D = ((lastVel - encoder1.getVelMeasured())*kd)/deltaTime;
				PID = P+I+D;
				if(PID>4095){
					TPM1_C0V = 4095;
				}
				else if(PID<0){
					TPM1_C0V = 0;
				}
				else{
					TPM1_C0V = PID;
				}
				
				//caso botao R
				if(change == 1){
					state=7;
					change=0;
				}
				
				//caso botao S
				else if(change == 2){
					state = 10;
					change = 0;
				}
			break;
			
			//REV/CCW
			case 7:
				//muda sentido rotação e cai em estado 3
				motor1.setSentido(1);
				state = 3;
				change = 0;
			break;
			
			//REV/CW
			case 8:
				//muda sentido rotação e cai em estado 4
				motor1.setSentido(2);
				state = 4;
				change = 0;
			break;
			
			//BRAKE/CW
			case 9:
				//desacelera e cai em estado 1
				change = 0;
				motor1.brakeMotor();
				if(encoder1.getVelMeasured() == 0){
				state = 1;
				}
			break;
			
			//BRAKE/CCW
			case 10:
				//desacelera e cai em estado 2
				change = 0;
				motor1.brakeMotor();
				if(encoder1.getVelMeasured() == 0){
				state = 2;
				}
			break;

		}
		vel = encoder1.getVelMeasured();
		SP = pot1.getSetpoint();
	}
}


