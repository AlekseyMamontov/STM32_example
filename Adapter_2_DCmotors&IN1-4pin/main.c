
#include "main.h"

#define n_Sync_Object 1

#define Led_Status 4
#define Led_Error  2

#define PWM_prescaler TIM2->PSC
#define PWM_arr TIM2->ARR

#define IN1_DRV TIM2->CCR1
#define IN2_DRV TIM2->CCR2

#define IN1_DRV2 TIM2->CCR3
#define IN2_DRV2 TIM2->CCR4

#define PIN1 0x10 // PA4
#define PIN2 0x20 // PA5
#define PIN3 0x40 // PA6
#define PIN4 0x80 // PA7


struct Adapter_DCmotor DC_motor;

// CAN object
struct CAN_frame  RAM_can_frames[MAX_BUFFER_CAN],Data_frame;
struct CAN_buffer Dcan_buffer;
// CanOpen
CAN_FIFOMailBox_TypeDef rx_mailbox;
CAN_TxMailBox_TypeDef 	tx_mailbox,tx_mailbox2;

uint32_t id_rxPDO1,id_rxPDO2,
		 id_txPDO1,id_rxSDO,
		 id_txSDO,heartbroken;

uint8_t  NMT_ID = 0,NMT_status = NMT_status_Operational;
//uint32_t *sync_data,*Sync_obj[n_Sync_Object]={NULL};


uint8_t  send_txPDO1 = 0,send_txSDO = 0;

uint8_t  pin_ms[4]={0,0,0,0},
		 gpio_ms[4]={20,20,20,20},
		 send_gpio = 0;

uint16_t gpio_old =0,
		 gpio_new = 0,
		 pin_status = 0;


int main(void)
{

	init_can_buffer(&Dcan_buffer,RAM_can_frames,MAX_BUFFER_CAN);
	init_controller_STM32F042();

	   tx_mailbox.TIR = (heartbroken<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	   tx_mailbox.TDTR = 1;// n на_отправку
	   tx_mailbox.TDLR = 0;// d0-d3
	   tx_mailbox.TDHR = 0;// d4-d7

	   CAN_transmit(&tx_mailbox);

  struct CAN_frame can_frame;
  uint8_t send_data,data0,data1;
  uint16_t data16;
  send_txPDO1 = 0;

  gpio_old = gpio_new = GPIOA->IDR&0xf0;
  send_gpio ++;

  while (1){

	  if(!read_can_buffer(&Dcan_buffer,&can_frame)){

		if(can_frame.dlc){

		  data0 = (can_frame.msg[0]&0xff);
		  DC_motor.in_data = data0&3;

		  switch (can_frame.dlc){

		  case 1:

			  IN1_DRV = (data0&1)?DC_motor.in1_drv_pwm:0;
			  IN2_DRV = (data0&2)?DC_motor.in2_drv_pwm:0;
			  IN1_DRV2 = (data0&4)?DC_motor.in1_drv2_pwm:0;
			  IN2_DRV2 = (data0&8)?DC_motor.in2_drv2_pwm:0;

			  break;

		  case 2:

			  data1 = (can_frame.msg[0]&0xff00)>>8;
			  if(data0&1)DC_motor.in1_drv_pwm =  data1;
			  if(data0&2)DC_motor.in2_drv_pwm =  data1;
			  if(data0&4)DC_motor.in1_drv2_pwm = data1;
			  if(data0&8)DC_motor.in2_drv2_pwm = data1;

			  IN1_DRV = (data0&1)?DC_motor.in1_drv_pwm:0;
			  IN2_DRV = (data0&2)?DC_motor.in2_drv_pwm:0;
			  IN1_DRV2 =(data0&4)?DC_motor.in1_drv2_pwm:0;
			  IN2_DRV2 =(data0&8)?DC_motor.in2_drv2_pwm:0;

			  break;

		  default:

			  if(!can_frame.dlc) break;
		  	  data16 = (can_frame.msg[0]&0xffff00)>>8;
		  	  if(data0&1)DC_motor.in1_drv_pwm = data16;
		  	  if(data0&2)DC_motor.in2_drv_pwm = data16;
			  if(data0&4)DC_motor.in1_drv2_pwm = data16;
			  if(data0&8)DC_motor.in2_drv2_pwm = data16;
		  	  IN1_DRV = (data0&1)?DC_motor.in1_drv_pwm:0;
		  	  IN2_DRV = (data0&2)?DC_motor.in2_drv_pwm:0;
			  IN1_DRV2 =(data0&4)?DC_motor.in1_drv2_pwm:0;
			  IN2_DRV2 =(data0&8)?DC_motor.in2_drv2_pwm:0;
			  break;

		  };

		};
	  };

	  if(send_gpio){

		  tx_mailbox2.TIR = ((txPDO2+ DC_motor.can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
		  tx_mailbox2.TDTR = 1;
		  tx_mailbox2.TDLR = gpio_old;
		  tx_mailbox2.TDHR = 0;

			  if (CAN_transmit(&tx_mailbox2) == 0) send_gpio --;
	  }

	  SDO_object(&DC_motor);

	  if(send_txPDO1){

	  		   tx_mailbox.TIR = ((txPDO1 + DC_motor.can_id)<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения


	  		   	   send_data = (IN1_DRV == 0)?0:1;
	  		   	   send_data |= (IN2_DRV == 0)?0:2;
	  		   	   send_data = (IN1_DRV2 == 0)?0:4;
	  		   	   send_data |= (IN2_DRV2 == 0)?0:8;

	  			   tx_mailbox.TDTR = 3;
	  			   tx_mailbox.TDLR = send_data|
	  								   (DC_motor.in1_drv_pwm << 8)
	  								   |(DC_motor.in2_drv_pwm << 16);
	  			   tx_mailbox.TDHR = 0;


	  		   if (CAN_transmit(&tx_mailbox) == 0) send_txPDO1 = 0;

	  };
  };
}



void TIM3_IRQHandler(void){TIM3->SR &= ~TIM_SR_UIF;}
void TIM2_IRQHandler(void){TIM2->SR &= ~TIM_SR_UIF;}
void TIM14_IRQHandler(){TIM14->SR &= ~TIM_SR_UIF;}

void CEC_CAN_IRQHandler(void){

	uint32_t id,dlc;
	uint8_t data0,cmd;

			if((CAN->RF0R & 0b0011) == 0) goto exit; //no message
			// индефикатор = тип сообщения ? extd : std
			id = CAN->sFIFOMailBox[0].RIR&0b0100?CAN->sFIFOMailBox[0].RIR >> 3:CAN->sFIFOMailBox[0].RIR >> 21;
			dlc = CAN->sFIFOMailBox[0].RDTR &0b01111;// получение из регистра длины кадра
			if((CAN->sFIFOMailBox[0].RIR & 0b010) == 0 && dlc != 0){// тип сообщения 0 - data / 1- rtr

				//dlc = dlc>8? 8:dlc;
				data0 = CAN->sFIFOMailBox[0].RDLR&0xff;

				switch (id){

					 case 0:
					  if(dlc < 2) break;
					  cmd = data0;
					  data0  = (CAN->sFIFOMailBox[0].RDLR&0xff00) >>8;

					  if(data0 == 0 || data0 == NMT_ID ){

						  switch(cmd){
					  	  	  case NMT_start : NMT_status = NMT_status_Operational;break;
					  	  	  case NMT_pre_operational: NMT_status = NMT_status_Pre_Operational;break;
					  	  	  case NMT_stop: NMT_status = NMT_status_Stopped;break;
					  	  	  case NMT_reset: NVIC_SystemReset();break;
					  	  	  default:break;}

					  };
					 break;

					default:

						if(id == id_rxPDO1){
						if(NMT_status != NMT_status_Operational) break;
						if(!dlc) break;

							Data_frame.id = id;
							Data_frame.dlc = dlc;
							Data_frame.msg[0] = CAN->sFIFOMailBox[0].RDLR;
							Data_frame.msg[1] = CAN->sFIFOMailBox[0].RDHR;
							write_can_buffer (&Dcan_buffer,&Data_frame);

						}else if(id == id_rxPDO2){

								if(NMT_status != NMT_status_Operational) break;
								if(!dlc) break;
								if(!data0)break;
								send_txPDO1 =  1;
						};

				    break;
				};
			};
	exit:	SET_BIT(CAN->RF0R, CAN_RF0R_RFOM0);//CAN->RF0R |= 0b0100000;  сообщение прочитано.
};

void Set_default_data(struct Adapter_DCmotor *ram){

	ram->num = 0;
	ram->in_data = 0;
	ram->in1_drv_pwm = 200;
	ram->in2_drv_pwm = 200;
	ram->in1_drv2_pwm = 200;
	ram->in2_drv2_pwm = 200;
	ram->can_id = 10;
	ram->can_speed = 0;// 500kb
	ram->timer_psc = 120-1;
	ram->timer_arr = 200-1;

};

void Load_default_data(struct Adapter_DCmotor *ram_addr){

	volatile  uint16_t* ram_addr16  = (volatile uint16_t*)ram_addr;
	volatile  uint16_t* flash_addr = (volatile uint16_t*)ADDR_SAVE_FLASH;
	uint16_t size16 = sizeof(struct Adapter_DCmotor)/2;

				if(*flash_addr == 0xFFFF){ // Default

					Set_default_data(ram_addr);

				}else{

					for(uint8_t i=0;i< size16;i++){

				    	*ram_addr16++ = *flash_addr++;

					};
				};
};
void Save_flash_block(struct Adapter_DCmotor *ram_addr){

	volatile  uint16_t* flash_addr16,*ram_addr16;
	flash_addr16 = (volatile uint16_t*)ADDR_SAVE_FLASH;
    ram_addr16  =  (volatile uint16_t*) ram_addr;
    uint16_t size16 = sizeof(struct Adapter_DCmotor)/2;


		// key
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
		//clear
		while (FLASH->SR & FLASH_SR_BSY){};
		if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;
		FLASH->CR |= FLASH_CR_PER;
		FLASH->AR =  ADDR_SAVE_FLASH; //addr segment
		FLASH->CR |= FLASH_CR_STRT;
		while (!(FLASH->SR & FLASH_SR_EOP));//check operation end
		FLASH->SR = FLASH_SR_EOP;
		FLASH->CR &= ~FLASH_CR_PER;
		//save
		while (FLASH->SR & FLASH_SR_BSY){};
		if (FLASH->SR & FLASH_SR_EOP) FLASH->SR = FLASH_SR_EOP;
		FLASH->CR |= FLASH_CR_PG;

		(ram_addr->num) ++;

		for(uint16_t i=0;i< size16;i++){

		  *flash_addr16++ = *ram_addr16++;
		  while (!(FLASH->SR & FLASH_SR_EOP)){};
		  FLASH->SR = FLASH_SR_EOP;

		};


		FLASH->CR &= ~(FLASH_CR_PG);
		FLASH->CR |= FLASH_CR_LOCK;


};

void SDO_object(struct Adapter_DCmotor *DCmotor){

	uint32_t id,dlc,cmd,index,subindex,data;
	uint16_t data16;
    uint32_t error = 0;

	//no message?
	if((CAN->RF1R & 0b0011) == 0) return;

	// индефикатор = тип сообщения ? extd : std
	id = CAN->sFIFOMailBox[1].RIR&0b0100?CAN->sFIFOMailBox[1].RIR >> 3:CAN->sFIFOMailBox[1].RIR >> 21;
	dlc = CAN->sFIFOMailBox[1].RDTR &0b01111;// получение из регистра длины кадра
	if((CAN->sFIFOMailBox[1].RIR & 0b010) != 0 && dlc < 4)goto sdo_exit;// тип сообщения 0 - data / 1- rtr
	if( id != id_rxSDO || NMT_status ==  NMT_status_Stopped ) goto sdo_exit;


		cmd = 		CAN->sFIFOMailBox[1].RDLR&0xFF;
		index =    (CAN->sFIFOMailBox[1].RDLR&0x00FFFF00) >> 8;
		subindex = (CAN->sFIFOMailBox[1].RDLR&0xFF000000) >> 24;
		data = 		CAN->sFIFOMailBox[1].RDHR;

		error = ERROR_NO_ACCESS;

		switch(index){

///// id_product
			case 0x1000:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};
				if(cmd !=0x40){error =ERROR_NO_SAVE;break;};

					 cmd = 0x4B;
					 dlc = 6;
					 error = 0;
					 data = 0x00030192;// dc402

			break;

///// name_product
			case 0x1008:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};
				if(cmd !=0x40){error =ERROR_NO_SAVE;break;};

					  cmd = 0x43;
					  dlc = 8;
					  error = 0;
					  data = 0x326D4344; // DCm1

			break;

/// save to flash
			case 0x1010:
	///read sub 0
		    	if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
		    	if(subindex == 0){
		    	if(cmd !=0x40){error =ERROR_NO_SAVE;break;};
		    		cmd = 0x4F;
		    		dlc = 5;
		    		error = 0;
		    		data = DCmotor->num;// N save
		    	break;}
	///save sub1
				if(dlc < 8)break;
				if(cmd !=0x23) break;
				if(subindex != 1)break;
				if(data != 0x65766173)break;
				NVIC_DisableIRQ(CEC_CAN_IRQn);
				Save_flash_block(DCmotor);
				NVIC_EnableIRQ(CEC_CAN_IRQn);
					cmd = 0x60;
					dlc = 4;
					error = 0;
					data = 0;

					break;

			case 0x1011: // load default

				if(dlc < 8)break;
				if(cmd !=0x23) break;
				if(subindex != 1)break;
				if(data != 0x64616f6c)break;

				NVIC_DisableIRQ(CEC_CAN_IRQn);
				data16 = DCmotor->num;
				Set_default_data(DCmotor);
				DCmotor->num = data16;
				Save_flash_block(DCmotor);
				NVIC_EnableIRQ(CEC_CAN_IRQn);
				// hard reset
				//NVIC_SystemReset();

				//soft reset)
/*
				 init_CAN_module();

				 PWM_prescaler = DC_motor.timer_psc;
				 PWM_arr = DC_motor.timer_arr;
				 IN1_DRV = (DC_motor.in_data&1)?DC_motor.in1_drv_pwm:0;// TIM2->CCR1 = ?;
				 IN2_DRV = (DC_motor.in_data&2)?DC_motor.in2_drv_pwm:0;// TIM2->CCR2 = ?;
				 LEFT_DRV_led  = IN1_DRV;//TIM2->CCR3 = ?;
				 RIGHT_DRV_led = IN2_DRV;
*/
					cmd = 0x60;
					dlc = 4;
					error = 0;
					data = 0;

					break;

				// Block Data


////// IN_1_2 || IN2_1_2
				case 0x2000:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

				  if(cmd == 0x40){  //read

					  cmd = 0x4F;
					  dlc = 5;
					  data=(DCmotor->in_data)&0x0f;
					  error = 0;


				}else
				  if(cmd == 0x2F){   //save

					  if(dlc < 5){error = ERROR_sLEN_OBJECT;break;};

					  if(data&0x0f)DCmotor->in_data= data&0x0f;

					  if (NMT_status == NMT_status_Operational){
					  IN1_DRV = (data&1)?DCmotor->in1_drv_pwm:0;
					  IN2_DRV = (data&2)?DCmotor->in2_drv_pwm:0;
					  IN1_DRV2 = (data&4)?DCmotor->in1_drv2_pwm:0;
					  IN2_DRV2 = (data&8)?DCmotor->in2_drv2_pwm:0;
					  }

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

				}
				break;

////// IN_PWM1
				case 0x2001:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

				  if(cmd == 0x40){  //read

					  cmd = 0x4B;
					  dlc = 0x06;
					  error=0;
					  data= DCmotor->in1_drv_pwm;

				}else

				  if(cmd ==0x2B){   //save

					  if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

					  DCmotor->in1_drv_pwm = data&0xffff;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

				}
				break;
///// IN_PWM2
				case 0x2002:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

				  if(cmd == 0x40){  //read

					  cmd = 0x4B;
					  dlc = 0x06;
					  error=0;
					  data= DCmotor->in2_drv_pwm;

				}else

				  if(cmd == 0x2B){   //save

					  if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

					  DCmotor->in2_drv_pwm = data&0xffff;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

				}
				break;

////// IN2_PWM1
				case 0x2003:

					if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
					if(subindex != 0){error =ERROR_SUB_INDEX;break;};

					if(cmd == 0x40){  //read

						cmd = 0x4B;
						dlc = 0x06;
						error=0;
						data= DCmotor->in1_drv2_pwm;

					}else

					if(cmd ==0x2B){   //save

						if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

						DCmotor->in1_drv2_pwm = data&0xffff;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

						}
				break;

///// IN2_PWM2
				case 0x2004:

					if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
					if(subindex != 0){error =ERROR_SUB_INDEX;break;};

					if(cmd == 0x40){  //read

						cmd = 0x4B;
						dlc = 0x06;
						error=0;
						data= DCmotor->in2_drv2_pwm;

					}else

					if(cmd == 0x2B){   //save

					if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

						 DCmotor->in2_drv2_pwm = data&0xffff;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

					}
				break;

///// ID_CAN
				case 0x2005:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

				  if(cmd == 0x40){  //read

					  cmd = 0x4F;
					  dlc = 0x05;
					  error=0;
					  data=(DCmotor->can_id)&0x7F;


				}else
				  if(cmd == 0x2F){   //save

					  if(dlc < 5){error = ERROR_sLEN_OBJECT;break;};

					  DCmotor->can_id = data&0x7F;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

				}
				break;

//// CAN_SPEEd
				case 0x2006:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

				  if(cmd == 0x40){  //read

					  cmd = 0x4F;
					  dlc = 0x05;
					  error=0;
					  data=(DCmotor->can_speed)&0x07;


				}else
				  if(cmd ==0x2F){   //save

					  if(dlc < 5){error = ERROR_sLEN_OBJECT;break;};

					  DCmotor->can_id = data&0x07;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

				}
				break;

//// TIMER_PSC
				case 0x2007:

				if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
				if(subindex != 0){error =ERROR_SUB_INDEX;break;};

					if(cmd == 0x40){  //read

						cmd = 0x4B;
						dlc = 0x06;
						error=0;
						data= DCmotor->timer_psc;

				}else
					if(cmd ==0x2B){   //save

					if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

					DCmotor->timer_psc = data&0xffff;

						cmd = 0x60;
						dlc = 4;
						error = 0;
						data = 0;

					}
			   break;

//// TIMER_ARR
			   case 0x2008:

			   if(dlc < 4){error = ERROR_sLEN_OBJECT;break;};
			   if(subindex != 0){error =ERROR_SUB_INDEX;break;};

			   	   	if(cmd == 0x40){  //read

			   		   cmd = 0x4B;
			   		   dlc = 0x06;
			   		   error=0;
			   		   data= DCmotor->timer_arr;

			   	}else
			   		if(cmd ==0x2B){   //save

			   			if(dlc < 6){error = ERROR_sLEN_OBJECT;break;};

			   			DCmotor->timer_arr = data&0xffff;

			   				cmd = 0x60;
			   				dlc = 4;
			   				error = 0;
			   				data = 0;

			   	}
			    break;

		default:
			break;
	};

	   if(error){cmd = 0x80;dlc = 8;data = error_msg[error];}

	   tx_mailbox.TIR = (id_txSDO<<21)| 0x01; // addr,std0,data0,TXRQ - отправка сообщения
	   tx_mailbox.TDTR = dlc;// n на_отправку
	   tx_mailbox.TDLR = (subindex<<24)|index<<8|cmd;// d0-d3
	   tx_mailbox.TDHR = data;// d4-d7
	   CAN_transmit(&tx_mailbox);

sdo_exit:SET_BIT(CAN->RF1R, CAN_RF1R_RFOM1);
};

// ------- GPIO -------

void EXTI4_15_IRQHandler(void){

	uint16_t  gpio_exti= (EXTI->PR&0xf0);
	pin_status |= gpio_exti;

	if (pin_status & PIN1)pin_ms[0] = gpio_ms[0];
	if (pin_status & PIN2)pin_ms[1] = gpio_ms[1];
	if (pin_status & PIN3)pin_ms[2] = gpio_ms[2];
	if (pin_status & PIN4)pin_ms[3] = gpio_ms[3];

	EXTI->PR |= gpio_exti;
};
void SysTick_Handler(void){

	uint16_t gpio;

	//if(send_gpio) send_gpio--;

	if(pin_status){

		gpio = GPIOA->IDR&0xf0;

		// pin1
		if (pin_status&PIN1){
			if(--pin_ms[0] == 0){
				pin_status &= ~PIN1;
				if(gpio&PIN1){gpio_new |= PIN1;}else{gpio_new &=(~PIN1);}
			};
		};
		// pin2
		if (pin_status&PIN2){
			if(--pin_ms[1] == 0){
				pin_status &= ~PIN2;
				if(gpio&PIN2){gpio_new |= PIN2;}else{gpio_new &=(~PIN2);}
			};
		};

		// pin3
		if (pin_status&PIN3){
			if(--pin_ms[2] == 0){
				pin_status &= ~PIN3;
				if(gpio&PIN3){gpio_new |= PIN3;}else{gpio_new &=(~PIN3);}
			};
		};
		// pin4
		if (pin_status&PIN4){
			if(--pin_ms[3] == 0){
				pin_status &= ~PIN4;
				if(gpio&PIN4){gpio_new |= PIN4;}else{gpio_new &=(~PIN4);}
			};
		};

	if(gpio_old != gpio_new){gpio_old = gpio_new;send_gpio++;}

	};

};











void init_controller_STM32F042(void){

	/*registr ACR  str.69
	Bit 4 PRFTBE: Prefetch buffer enable
	0: Prefetch is disabled /
	1: Prefetch is enabled
    Bit 3 Reserved, must be kept at reset value.
    Bits 2:0 LATENCY[2:0]: Latency
    These bits represent the ratio of the SYSCLK (system clock) period to the flash access time.
    000: Zero wait state, if SYSCLK ≤ 24 MHz
    001: One wait state, if 24 MHz < SYSCLK ≤ 48 MHz
	 * */
	FLASH->ACR |= FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY; // включаем буфер и частота 48мгц

	NVIC_SetPriority(RCC_CRS_IRQn,0);
	NVIC_EnableIRQ(RCC_CRS_IRQn);

	RCC->CR |= RCC_CR_HSEON;  			//включаем генератор HSE
	while(!(RCC->CR & RCC_CR_HSERDY)){};//ожидание готовности HSE
    RCC->CR |= RCC_CR_CSSON; 			//следить за кварцем

	RCC->CFGR2 |= RCC_CFGR2_PREDIV_DIV1;  // Prediv предделитель для PLL
	//завести HSE на PLL и 8 Мгц x6 = 48 Мгц
	RCC->CFGR |=(RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL6);

	RCC->CR |= RCC_CR_PLLON;  			//включаем PLL
	while(!(RCC->CR & RCC_CR_PLLRDY));  // ожидание готовности PLL
	RCC->CFGR |= RCC_CFGR_SW_PLL;  		// переводим тактирование от PLL

	//MODIFY_REG(RCC->CFGR,RCC_CFGR_HPRE,RCC_CFGR_HPRE_DIV1);//AHB Prescaler /1
	//MODIFY_REG(RCC->CFGR,RCC_CFGR_PPRE,RCC_CFGR_PPRE_DIV1);//APB1 Prescaler /1,

	RCC->APB1ENR |= RCC_APB1ENR_PWREN;	  // управление питанием


	/*
	 SYSCFG configuration register 1 (SYSCFG_CFGR1
		 PA11_PA12_RMP: PA11 and PA12 remapping bit for small packages (28 and 20 pins).
		Available on STM32F04x devices only.
			This bit is set and cleared by software.
			It controls the mapping of either PA9/10 or PA11/12 pin pair on small pin
			count packages.
			0: No remap (pin pair PA9/10 mapped on the pins)
			1: Remap (pin pair PA11/12 mapped instead of PA9/10)

	 */

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // Включение SYSCFG
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP; // CAN




  /******************** GPIO ************************************
	 *
	 GPIO port mode register (GPIOx_MODER)
		Address offset:0x00
		Reset value: 0x2800 0000 for port A
		Reset value: 0x0000 0000 for other ports
			00: Input mode (reset state)
			01: General purpose output mode
			10: Alternate function mode
			11: Analog mode
	 GPIO port output type register (GPIOx_OTYPER)
	 Address offset: 0x04
	 Reset value: 0x0000 0000
			0: Output push-pull (reset state)
			1: Output open-drain

	GPIO port output speed register (GPIOx_OSPEEDR)
		Address offset: 0x08
		Reset value: 0x0C00 0000 (for port A)
		Reset value: 0x0000 0000 (for other ports)
			x0: Low speed
			01: Medium speed
			11: High speed

	GPIO port pull-up/pull-down register (GPIOx_PUPDR)
		Address offset: 0x0C
		Reset value: 0x2400 0000 (for port A)
		Reset value: 0x0000 0000 (for other ports)
			00: No pull-up, pull-down
			01: Pull-up
			10: Pull-down
			11: Reserved
	GPIO port input data register (GPIOx_IDR)
		Address offset: 0x10
		Reset value: 0x0000 XXXX
	GPIO port output data register (GPIOx_ODR)
		Address offset: 0x14
		Reset value: 0x0000 0000
	GPIO port bit set/reset register (GPIOx_BSRR) Reset value: 0x0000 0000
	GPIO alternate function low register (GPIOx_AFRL) Reset value: 0x0000 0000
	GPIO port configuration lock register (GPIOx_LCKR) Reset value: 0x0000 0000
	*/





     /* GPIOA
	 0,1,2,3 TIM2 ch1-ch4 PWM output
	 13-14 (swlck/swdio);
	 11,12 CAN
	 in0-in3  PA4,PA5,PA6,PA7

	 */

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;// включить порт GPIOA
	//---------------  151413121110 9 8 7 6 5 4 3 2 1 0
	GPIOA->MODER  |= 0b00000010100000000000000010101010; // General purpose output mode
	GPIOA->OSPEEDR|= 0b00000000000000000000000111111111; //0b11  high speed
	//GPIOA->PUPDR  |= 0b00000000000000000000000000000000; //8,9,10 addrCan,5 none
	//GPIOA->OTYPER |= 0b0000000000000000;// open drain led_status

	/*
	 GPIOB

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;// enable GPIOВ
	//---------------  	151413121110 9 8 7 6 5 4 3 2 1 0
    GPIOB->MODER   |= 0b00000000000000000000000000011010;// General purpose in/out mode
    GPIOB->OSPEEDR |= 0b00000000000000000000000000011111;// 11 high, 01 medium speed
    GPIOB->PUPDR   |= 0b00000000000010101010101010000000;
    GPIOA->OTYPER  |= 0b0000000000000000;// open drain led_status

    */


/*---------------  PWM  TIM3 ---------------

	Bits 6:4 OC1M: Output compare 1 mode
	These bits define the behavior of the output reference signal OC1REF from which OC1 and
	OC1N are derived. OC1REF is active high whereas OC1 and OC1N active level depends on
	CC1P and CC1NP bits.
	000: Frozen - The comparison between the output compare register TIMx_CCR1 and the
	counter TIMx_CNT has no effect on the outputs.(this mode is used to generate a timing base).
	001: Set channel 1 to active level on match. OC1REF signal is forced high when the counter
	TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
	010: Set channel 1 to inactive level on match. OC1REF signal is forced low when the
	counter TIMx_CNT matches the capture/compare register 1 (TIMx_CCR1).
	011: Toggle - OC1REF toggles when TIMx_CNT=TIMx_CCR1.
	100: Force inactive level - OC1REF is forced low.
	101: Force active level - OC1REF is forced high.
	110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1
	else inactive. In downcounting, channel 1 is inactive (OC1REF=‘0) as long as
	TIMx_CNT>TIMx_CCR1 else active (OC1REF=1).
	111: PWM mode 2 - In upcounting, channel 1 is inactive as long as TIMx_CNT<TIMx_CCR1
	else active. In downcounting, channel 1 is active as long as TIMx_CNT>TIMx_CCR1 else
	inactive.

	 *
	 *
	 *
	 *
	 * */

	Load_default_data(&DC_motor);
	NMT_ID = DC_motor.can_id;


	/*	   RCC->AHBENR |= RCC_APB1ENR_TIM2EN;
		   TIM2->PSC = 120 - 1;
		   TIM2->ARR = 200-1;

		   GPIOA->MODER |=  GPIO_MODER_MODER2_1;//Alternate function mode
		   GPIOA->AFR[0] |= 0x200;  // 3 pin AF2 tim2_ch3

		   TIM2->CR1 |= 0x80;// Preload_en
		   TIM2->CCMR2 |= 0b110 << 4;//TIM2_ch3_PWM_1
		   TIM2->CCER |= TIM_CCER_CC3E|TIM_CCER_CC3P;
		   TIM2->CCR3 = 00; //50%
		   TIM2->CR1 |= TIM_CR1_CEN;// on timer2
	*/
			// TIM2

			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

			TIM2->PSC = DC_motor.timer_psc;// 120 - 1; // 2 kГц
			TIM2->ARR = DC_motor.timer_arr;// 200-1;

			GPIOA->AFR[0] |= 0x2222;//AF#2 // TIM_CH1-TIM_CH4 pin A0-A3

			TIM2->CR1 |= 0x80; // Preload_en
			TIM2->CCMR1 |= 0b110 << 4 | 0b110 << 12;//
			TIM2->CCMR2 |= 0b110 << 4 | 0b110 << 12;//Bits 6:4 OC1M PWM 1
			TIM2->CCER |= TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|TIM_CCER_CC4E;
		  //TIM2->CCER |= TIM_CCER_CC1P|TIM_CCER_CC2P|TIM_CCER_CC3P|TIM_CCER_CC4P;// output ~low _

			  IN1_DRV = (DC_motor.in_data&1)?DC_motor.in1_drv_pwm:0;// TIM2->CCR1 = ?;
			  IN2_DRV = (DC_motor.in_data&2)?DC_motor.in2_drv_pwm:0;// TIM2->CCR2 = ?;


			  IN1_DRV2 = (DC_motor.in_data&1)?DC_motor.in1_drv2_pwm:0;// TIM2->CCR3 = ?;
			  IN2_DRV2 = (DC_motor.in_data&2)?DC_motor.in2_drv2_pwm:0;// TIM2->CCR4 = ?;

		  // Включение таймера 3

			TIM2->CR1 |= TIM_CR1_CEN;// on timer2

			//NVIC_EnableIRQ(TIM2_IRQn);




/***************************** EXTIxx  *********************************
 *
 * SYSCFG_EXTICR1 exti0-3
 * SYSCFG_EXTICR2 exti4-7
 * SYSCFG_EXTICR3 exti8-11
 * SYSCFG_EXTICR4 ext12-15
  Bits 31:16 Reserved, must be kept at reset value.
  Bits 15:0 EXTIx[3:0]: EXTI x configuration bits (x = 0 to 3)
	These bits are written by software to select the source input for the EXTIx external interrupt.
		x000: PA[x] pin
		x001: PB[x] pin
		x010: PC[x] pin
		x011: PD[x] pin
		x100: PE[x] pin
		x101: PF[x] pin
			other configurations: reserved

   SYSCFG_ITLINE5
   	   Bit 1 EXTI1: EXTI line 1 interrupt request pending
   	   Bit 0 EXTI0: EXTI line 0 interrupt request pending
   SYSCFG_ITLINE6
   	   Bit 1 EXTI3: EXTI line 3 interrupt request pending
   	   Bit 0 EXTI2: EXTI line 2 interrupt request pending
   SYSCFG_ITLINE7
		Bit 11 EXTI15: EXTI line 15 interrupt request pending
   	   	Bit 10 EXTI14: EXTI line 14 interrupt request pending
		Bit 9 EXTI13: EXTI line 13 interrupt request pending
		Bit 8 EXTI12: EXTI line 12 interrupt request pending
		Bit 7 EXTI11: EXTI line 11 interrupt request pending
		Bit 6 EXTI10: EXTI line 10 interrupt request pending
		Bit 5 EXTI9: EXTI line 9 interrupt request pending
		Bit 4 EXTI8: EXTI line 8 interrupt request pending
		Bit 3 EXTI7: EXTI line 7 interrupt request pending
		Bit 2 EXTI6: EXTI line 6 interrupt request pending
		Bit 1 EXTI5: EXTI line 5 interrupt request pending
		Bit 0 EXTI4: EXTI line 4 interrupt request pending

	Interrupt mask register (EXTI_IMR)

	    bit 0 -15 exti0-15
		Address offset: 0x00
		Reset value: 0x0FF4 0000 (STM32F03x devices)
		0x7FF4 0000 (STM32F04x devices)
		0x0F94 0000 (STM32F05x devices)
		0x7F84 0000 (STM32F07x and STM32F09x devices)

		Bits 31:0 IMx: Interrupt Mask on line x (x = 31 to 0)
		0: Interrupt request from Line x is masked
		1: Interrupt request from Line x is not masked

		Event mask register (EXTI_EMR)
			Address offset: 0x04
			Reset value: 0x0000 0000

		Bits 31:0 EMx: Event mask on line x (x = 31 to 0)
			0: Event request from Line x is masked
			1: Event request from Line x is not masked

		Rising trigger selection register (EXTI_RTSR)  _|
		Falling trigger selection register (EXTI_FTSR) |_

			Address offset: 0x08
			Reset value: 0x0000 0000
			Bits 17:0 RTx: Rising trigger event configuration bit of line x (x = 17 to 0)
				0: Rising trigger disabled (for Event and Interrupt) for input line
				1: Rising trigger enabled (for Event and Interrupt) for input line.

		Software interrupt event register (EXTI_SWIER)
			Address offset: 0x10
			Reset value: 0x0000 0000
			Bits 17:0 SWIx: Software interrupt on line x (x = 17 to 0)
							If the interrupt is enabled on this line in the EXTI_IMR, writing a ‘1’ to this bit when it is at ‘0’
							sets the corresponding pending bit in EXTI_PR resulting in an interrupt request generation.
							This bit is cleared by clearing the corresponding bit of EXTI_PR (by writing a ‘1’ to the bit).
		Pending register (EXTI_PR)
			Address offset: 0x14
			Reset value: undefined
			PIFx: Pending bit on line x (x = 17 to 0)
				0: no trigger request occurred
				1: selected trigger request occurred
				This bit is set when the selected edge event arrives on the external interrupt line. This bit is
				cleared by writing a 1 to the bit.

 */

		EXTI->IMR = 0xf0; //pin A4-A7 exti
		EXTI->FTSR =0xf0;
		EXTI->RTSR =0xf0;

		NVIC_EnableIRQ(EXTI4_15_IRQn);






/******************************  SYStick **************************/

		    SysTick->LOAD = 48000000/1000 - 1;  // (48 mHz / 1000) -1  // 1ms
		    SysTick->VAL = 0;  // reset
		    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
		    NVIC_EnableIRQ(SysTick_IRQn);

		    //NVIC_SetPriority(SysTick_IRQn, 2);


/******************************  TIM14 **************************/
/*
	    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	    TIM14->PSC =  48000000/1000 - 1;
	    TIM14->DIER |= TIM_DIER_UIE; 	  // enable perepolnenia TIM14
	    TIM14->CR1 |= TIM_CR1_CEN;		  // ON TIM14
	    TIM14->ARR = 0; //1 ms
	 // NVIC_EnableIRQ(TIM14_IRQn);

*/


/***************************** CAN module ********************************
   	CAN GPIO Configuration
   	   	   PA11     ------> CAN_RX
   	   	   PA12     ------> CAN_TX

	GPIO configuration for CAN signals
	   	   (1) Select AF mode (10) on PA11 and PA12
	   	   (2) AF4 for CAN signals
 	GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER11 | GPIO_MODER_MODER12)) |
	  	  	  	   	   (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);
 	GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH3 | GPIO_AFRH_AFRH4)) |
	  	  	  	     	 (4 << (3 * 4)) | (4 << (4 * 4));

	 CAN master control register (CAN_MCR) Reset value: 0x0001 0002
	 CAN master status register (CAN_MSR)  Reset value: 0x0000 0C02
*/




	//   выставить порты на альтернативные функции 0x10

	GPIOA->MODER |= (GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1);

	// скорость порта на максимум

	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR11|GPIO_OSPEEDR_OSPEEDR12;

	//  пины 11,12 включить модуль bxCAN

	GPIOA->AFR[1] |= ((0x04 << GPIO_AFRH_AFSEL11_Pos) | (0x04 << GPIO_AFRH_AFSEL12_Pos));

	// включить модуль Can

	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	init_CAN_module();

}

void init_CAN_module(){

		uint32_t speed;

		// перевод в режим иницилизации

		SET_BIT(CAN->MCR, CAN_MCR_INRQ);
		while ((CAN->MSR & CAN_MSR_INAK) == 0){};
		// выход из сна.
		CLEAR_BIT(CAN->MCR, CAN_MCR_SLEEP);
		while (CAN->MSR & CAN_MSR_SLAK){};

		/*// reset default
		CLEAR_BIT(CAN->MCR, CAN_MCR_TTCM);// Clear 0 /Set 1 the time triggered communication mode
		CLEAR_BIT(CAN->MCR, CAN_MCR_ABOM);// Clear 0 /Set 1 the automatic bus-off management
		CLEAR_BIT(CAN->MCR, CAN_MCR_AWUM);// Clear 0 /Set 1 the automatic wake-up mode
		CLEAR_BIT(CAN->MCR, CAN_MCR_NART);// Set 1/ Clear 0 the automatic retransmission
		CLEAR_BIT(CAN->MCR, CAN_MCR_RFLM);// Set the receive FIFO locked mode
		CLEAR_BIT(CAN->MCR, CAN_MCR_TXFP);// Set the transmit FIFO priority
		*/

		// Получение данных


		switch(DC_motor.can_speed){ // 48 Мгц.

			case 0x01: speed = 0x001c0002;break; //1000kb
			case 0x02: speed = 0x001b0003;break; //800kb
			case 0x03: speed = 0x001c0005;break; //500kb
			case 0x04: speed = 0x001c000b;break; //250kb
			case 0x05: speed = 0x001c0017;break; //125kb
			case 0x06: speed = 0x001c001d;break; //100kb
			case 0x07: speed = 0x001c003b;break; //50kb
			default: speed = 0x001c0005;break;//000 -default 500kb

		};

		WRITE_REG(CAN->BTR,speed);

	/*	CAN filter master register (CAN_FMR)  Reset value: 0x2A1C 0E01
	 * 		Bit 0 FINIT: Filter initialization mode
			Initialization mode for filter banks
				0: Active filters mode.
				1: Initialization mode for the filters.(reset default)

		CAN filter mode register (CAN_FM1R)  Reset value: 0x0000 0000
			     FBMx: Filter mode
				Mode of the registers of Filter x.
				0: Two 32-bit registers of filter bank x are in Identifier Mask mode.
				1: Two 32-bit registers of filter bank x are in Identifier List mode.

		CAN filter scale register (CAN_FS1R)  Reset value: 0x0000 0000
			This register can be written only when the filter initialization mode is set (FINIT=1) in the
				FSCx: Filter scale configuration
				These bits define the scale configuration of Filters 13-0.
				0: Dual 16-bit scale configuration
				1: Single 32-bit scale configuration

		CAN filter FIFO assignment register (CAN_FFA1R) Reset value: 0x0000 0000
			This register can be written only when the filter initialization mode is set (FINIT=1) in the
			FFAx: Filter FIFO assignment for filter x
				0: Filter assigned to FIFO 0
				1: Filter assigned to FIFO 1

		CAN filter activation register (CAN_FA1R) Reset value: 0x0000 0000
			FACTx: Filter active
			The software sets this bit to activate Filter x. To modify the Filter x registers (CAN_FxR[0:7]),
			the FACTx bit must be cleared or the FINIT bit of the CAN_FMR register must be set.
				0: Filter x is not active
				1: Filter x is active

		Filter bank i register x (CAN_FiRx) (i = 0..13, x = 1, 2) Reset value: 0xXXXX XXXX
			There are 14 filter banks, i= 0 to 13.
			Each filter bank i is composed of two 32-bit registers,
			CAN_FiR[2:1].
			This register can only be modified when the FACTx bit of the CAN_FAxR register is cleared
			or when the FINIT bit of the CAN_FMR register is set.

				Bits 31:0 FB[31:0]: Filter bits

				Each bit of the register specifies the level of the corresponding bit of the expected identifier.
					0: Dominant bit is expected
					1: Recessive bit is expected
				Mask
				Each bit of the register specifies whether the bit of the associated identifier register must
				match with the corresponding bit of the expected identifier or not.
					0: Do not care, the bit is not used for the comparison
					1: Must match, the bit of the incoming identifier mu

	*/

		id_rxPDO1 = rxPDO1 + DC_motor.can_id;
		id_rxPDO2 = rxPDO2 + DC_motor.can_id;
		id_txPDO1 = txPDO1 + DC_motor.can_id;
		id_rxSDO =  rxSDO +  DC_motor.can_id;
		id_txSDO =  txSDO +  DC_motor.can_id;
		heartbroken =0x700 + DC_motor.can_id;


		CAN->FMR |=  CAN_FMR_FINIT; /*  init mode */
		CAN->FA1R |= (CAN_FA1R_FACT0|CAN_FA1R_FACT1);//Activate filter 0,1
		CAN->FM1R |= (CAN_FM1R_FBM0 |CAN_FM1R_FBM1); //ID_list
		CAN->FS1R  &=~(CAN_FS1R_FSC0|CAN_FS1R_FSC1);  //16bit
		CAN->FFA1R &=~CAN_FFA1R_FFA0; // filtr0 -> FIFO0
		CAN->FFA1R |= CAN_FFA1R_FFA1; // filtr1 -> FIFO1

		CAN->sFilterRegister[0].FR1 = (id_rxPDO1<< 16 | id_rxPDO2 ) << 5;
		CAN->sFilterRegister[0].FR2 = 0;
		CAN->sFilterRegister[1].FR1 = (id_rxSDO << 16 | heartbroken)<< 5;//0x02 test rtr bit for heartbroken
		CAN->sFilterRegister[1].FR2 = (0x80 << 16 | 0x100)<< 5;

		CAN->FMR &=~ CAN_FMR_FINIT; /* Leave filter init */

		/*
		  	Bit 4 FMPIE1: FIFO1 message pending interrupt enable
			Bit 1 FMPIE0: FIFO0 message pending interrupt enable
			    0: No interrupt generated when state of FMP[1:0] bits are not 00b.
			    1: Interrupt generated when state of FMP[1:0] bits are not 00b.
		*/

		CAN->IER |= CAN_IER_FMPIE0; /* Set FIFO0 message pending IT enable */


		/*выйти в нормальный режим*/

		CAN->MCR &=~ CAN_MCR_INRQ;
		while((CAN->MSR & CAN_MSR_INAK)==CAN_MSR_INAK){}



	    /* CAN interrupt Init */
		NVIC_SetPriority(CEC_CAN_IRQn,1);
		NVIC_EnableIRQ(CEC_CAN_IRQn);

};

uint8_t CAN_transmit (CAN_TxMailBox_TypeDef *tx){

	uint8_t n=0;

	if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0){
	}else if((CAN->TSR & CAN_TSR_TME1) == CAN_TSR_TME1){ n=1;
	}else if((CAN->TSR & CAN_TSR_TME2) == CAN_TSR_TME2){ n=2;
	}else return 1; //busy

	CAN->sTxMailBox[n].TDTR = tx->TDTR;
	CAN->sTxMailBox[n].TDLR = tx->TDLR;
	CAN->sTxMailBox[n].TDHR = tx->TDHR;
	CAN->sTxMailBox[n].TIR = tx->TIR ;

	return 0;
}











void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();


  while (1){




  };
  /* USER CODE END Error_Handler_Debug */
};




