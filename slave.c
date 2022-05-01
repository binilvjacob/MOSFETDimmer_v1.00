
								// MOSFET DIMMABLE DRIVER VERSION 1 BETA v1.00 //
/* 
	   Details : 

	-> Faderate concept removed to execute over current shutdown
	-> PWM auto shutdown enabled
	-> PWM auto restart disabled
	-> PWM restart delay increments sequentially
	-> Hardware relay connected to PIN_C2
	-> Comparator 2 with inverted output is used
	-> CCP1 module is used in PWM mode
	-> Device type ID : 7
	-> DALI Tx pin : PIN_A0
	-> DALI Rx pin : PIN_A2
	-> Version : BETA
	-> Last modified date : 30/04/2022    
	-> Github repo URL :  https://github.com/binilvjacob/v1.0DALIDimmableDriverPIC16F684CCSCode.git

*/
 

#include <slave.h>
#include<math.h>
#include <STDLIB.H>

#use delay(clock=4000000)

#define device_type 7  				// setting device type 7 - MOSFET dimamble driver

#define Fixlampid   1 				// LAMP ADDRESS //
#define zoneid_init   210 			// zone address // 
#define G1 0b00000001
#define G2 0b00000000
#define rx pin_a2
#define tx pin_a0
#define self 0x01

#define MaxDuty  100 
#define MinDuty  0

///////////////////////////////////////////////////////////////

#define PowerOnLevelStore    		0        	// Latest power level   
#define MinimumLevelStore           1			// Minimum allowable power level
#define MaximumLevelStore  			2 			// Maximum allowable power level
#define ShortAddressStore  			3  			// Unique device ID
#define Group_07Store    			4			// First byte of group
#define Group_815Store  			5			// Second byte of group
#define SceneStore  				6			// 6-21 Scene level store
#define ZoneIDStore 				32
#define SystemFailureRateStore 		25
#define CurrentThresholdStore 		26			// To store current protection threshold value

////////////////// Device types ////////////////////////
/*
	lamp =1
	fan=2
	curtain=3
	strip=4
	mosfet dimamble driver = 7
*/
////////////////////////////////////////////////////////

#bit PRSEN     = 0x16.7   // PWM autorestart control bit
#bit CCMCON0 = 0x019.5	  // Comparator output invert bit
#bit ECCPASE = 0x017.7	  // PWM shutdown event status bit

#byte dutyreg = 0x15
 
#bit intf = 0x0b.1
#bit timerOnOff =0x10.0

int1 oddevenbit,a,atmp,b,error_flag,over_flowflag;
unsigned int8 dataCount;
char data[3],bitcount,tout;
unsigned char duty;

unsigned int16 power;

char settling_time,i,dly=4,j;
int1 txmit_error=0;
char tx_buffer[3];
char r_a,currentSceen;
char l_st;
char command_st,RetryCount;

char zoneid=zoneid_init;

char stopBitCount,address ,command,databyte;
int1 dataready,forwrdFrameFlag,backwardFrameFlag ,masterFlag ;
int16 readDly=300;
int16 GroupSelectReg;
char gindex;

/////// new  //////
int txmit_count=0;
int error_value=0;
///////////////////

char MinimumLevel;
char MaximumLevel;
char PowerOnLevel;
char DTR,DwriteLocation,DTR_Ready;

char lampid  = Fixlampid;

int1 reset_flag=0;			// For WDT reset operation inside RTCC interrupt

int32 restart_count=0;
int32 restart_delay=0;
char failure_count=0;
char current_threshold=1;

void readData(void);
void init(void);
void handle(void );
void copyData(void);
void commands(void);
void txmit(char priority,char length);
void txmit1(void);
void txmit0(void);
void stopbit(void);
void lamp_on(void);
void lamp_off(void);
void startBit(void);
void init_from_eeprom(void);
void SetDimmLevel(unsigned int dimPesentage);


#rom  0x2100={MaxDuty,MinDuty,MaxDuty,Fixlampid,G1,G2,0,20,30,50,70,90,100,35,40,45,75,25,60,65,95,100}

#rom  0x2120={zoneid_init}

#int_EXT
EXT_isr() 
{
			clear_interrupt(int_ext);
            disable_interrupts(int_ext);
            disable_interrupts(INT_RTCC);
            bitcount=0;
            setup_timer_1(T1_internal|T1_div_by_1);
            set_timer1(0xffff-840); //858  880///old value 923
            enable_interrupts(int_timer1);
            stopBitCount = 0;
            oddevenbit=1;
            data[0]=0;
            data[1]=0;
            data[2]=0;
            tout=0 ;
            datacount = 0;   
			settling_time = 0; 
}


#int_TIMER1
TIMER1_isr()
{

readDly=20;
error_flag=0;
	if(oddevenbit==1)
	{
		a=input(rx); 
		atmp=a ;          
		oddevenbit=0 ;
		
				if(atmp)
				{
					while(atmp)
						{
							atmp=input(rx);
							if(readDly>0)
								readDly--;
							else
								atmp=0;
								
						}
				}         
				else
				{
					while(!atmp)
						{
							atmp=input(rx);
							readDly--;
								if(readdly==0)
									{
									atmp=1;
									}	
						}
				}

			setup_timer_1(T1_internal|T1_div_by_1);//settimer1with1us least count
			set_timer1(0xffff-150);  //374  //  355             350////old value 150
	}
	else
	{ 
		b=input(rx) ; // store data line status in the second half
		oddevenbit=1;
		setup_timer_1(T1_internal|T1_div_by_1);
		set_timer1(0xffff-350);  // delay  till the next call st to 73 us/////old value 350
		readData();  // function  get the dat from the conditions of a and b
					
	}
return(0);
}

#int_RTCC
RTCC_isr()
{	
	
	reset_flag=1;
/*
	if(FadeRateCount>0)
	{
		FadeRateCount--;
	}
	else
	{
		FadeRateCount=FadeRate;
	}
*/
	dly--;
  	if (dly == 0)
  	{	
      dly = 4;
      if(settling_time < 250)
      {
          settling_time++;
      }              
   }
	if(ECCPASE==1 && restart_delay>2) // event just occurred and is not ready for reset
		{
			restart_delay--;
		}
}


void main(void)

{
	setup_wdt(WDT_ON);
	setup_wdt(WDT_72MS|WDT_TIMES_16);		//~1.1 s reset	
	init_from_eeprom();
	init();		
	GroupSelectReg = MAKE16(read_EEPROM (Group_815Store ),read_EEPROM (Group_07Store));	
	PowerOnLevel = read_EEPROM (PowerOnLevelStore);
	if(PowerOnLevel<= MinimumLevel)
	{
		output_low(pin_c2);
		duty=0;
		lamp_off();				
	}
	else
	{
		output_high(pin_c2);
		duty = PowerOnLevel;
		SetDimmLevel(duty);		
		lamp_on();		
	}
	restart_delay=3000;				// initial restart delay
	restart_count=0;				// initial restart counts
	ECCPASE=0;						// Restart operation

start:

	if(restart_delay<=2 && ECCPASE==1)			// restart after variable delay
	{
		ECCPASE=0;								// PWM restart operation
		restart_count++;						// Increment restart event counter
		restart_delay=restart_delay+restart_count*5000;		// Increasing the restart delay
		if(restart_delay>30000)					// Permanent shutdown after many restart attempts
		{
			output_low(pin_c2);
			failure_count++;					// Failed to turn ON
			if(failure_count>254)
			{
				failure_count=254;
			}
			write_eeprom(SystemFailureRateStore,failure_count);
			delay_us(10);			
		}		
	}
	
	if(reset_flag==1)
	{
	restart_wdt(); 
	reset_flag=0;
	}
		
	if (dataReady ==1)
	{
		if(address == 0xff)
		{
			handle(); 
		}		
		else if(address==lampid)		{
			
			handle(); 
		}		
    	else if(address == zoneid)
		{
			handle();	
		}
		else if(address>191 && address<208)
		{	
			gindex = address &0x0F;
			if ( bit_test (GroupSelectReg, gindex)==1)
			{ 				
				handle();
			}	
		}
		dataReady =0;
	}
	if(txmit_error==1 && txmit_count<5)
	{
		txmit_count++;
		txmit(2,2);
	}	
	else
	{
		txmit_count=0;
	}
	
	goto start;
}

void init(void)

{
	setup_timer_2(T2_DIV_BY_1,249,1);		//250 us overflow, 250 us interrupt  // 4000Hz
	setup_ccp1(CCP_PWM|CCP_SHUTDOWN_ON_COMP2|CCP_SHUTDOWN_AC_L|CCP_SHUTDOWN_BD_L);	// Setting up PWM
	setup_comparator(A0_VR_C0_VR);				// Setting up comparator
	setup_vref(VREF_LOW|current_threshold);						// Setting up reference voltage
	PRSEN=0;									// Auto-restart disabled
	CCMCON0=1;									// Comparator output inverted

	setup_timer_0(RTCC_INTERNAL|RTCC_DIV_1);
	setup_timer_1(T1_internal|T1_div_by_1);
	timerOnOff=0;
	clear_interrupt(int_ext);
	ext_int_edge( H_TO_L );	
	enable_interrupts(INT_EXT);
	enable_interrupts(INT_RTCC);
	disable_interrupts(INT_TIMER2);
	enable_interrupts(global);	
	settling_time =23;
	dataReady =0;  
	return;
}

void handle(void )
{
	commands();
	delay_ms(2);
	RetryCount =0;
	return;
}


//				trnsmission of  bit 1			//
/*********************************************************************
 * Function:       void txmit0(void);
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          transmission of  bit 1 to the bus	
 *
 * Side Effects:    None
 *
 * Note:            None
**********************************************************************/

void txmit1(void)

{     
  	txmit_error = 0;
	if (input(rx)==1)
	{  
		output_bit(tx,0);
	}
	delay_us(79);
	if (input(rx)==1)
	{
		output_bit(tx,1);
		txmit_error = 1;
		return;
	}			  
	delay_us(290);//345
	if (input(rx)==0)
	{
		output_bit(tx,1);
	}
	else
	{
		output_bit(tx,1);
		txmit_error = 1;
		return;
	}
	delay_us(79);
	if (input(rx)==0)
	{
		output_bit(tx,1);
		txmit_error = 1;
		return;
	}
    delay_us(290);
	if (input(rx)==0)
	{
		output_bit(tx,1);
		txmit_error = 1;
		return;
	}
    return;
}





//         transmission of 0 to the bus      //
/*********************************************************************
 * Function:       void txmit0(void);
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          transmission of  0 bit to the bus	
 *
 * Side Effects:    None
 *
 * Note:            None
**********************************************************************/

void txmit0(void)

{
	txmit_error = 0;	
	output_bit(tx,1);
	delay_us(79);
	if (input(rx)!=1)
	{		
		txmit_error = 1;
		return;
	}   
	delay_us(290);
	if (input(rx)==1)
	{
		output_bit(tx,0);
	}
    else
	{
		output_bit(tx,1);
		txmit_error = 1;
		return;
	}
    delay_us(79);
    if (input(rx)==1)
	{		
		txmit_error = 1;
		return;
	}
    delay_us(290);
	if (input(rx)==1)
	{		
		txmit_error = 1;
		return;
	}
    return;
}
//-----------------------------------------------------------------------------
                   // txmit2 bit
//-----------------------------------------------------------------------------

void txmit(char priority,char length)
{ 
	
     j= 8*length;
	 while (settling_time < 12+Fixlampid);     // priority
     disable_interrupts(global);
     txmit1();        // start bit  
     for(i=0;i<j;i++)
         {
            if (shift_left(tx_buffer,3,1)==1)
            {
                 txmit1();
            }
            else
            {
                  txmit0();
            }
            if (txmit_error ==1)
            {
			//	output_low(pin_c3);
               goto rr;
            }		
         }        
     stopbit();    
     stopbit(); 
	stopbit(); stopbit();
rr:  output_bit(tx,1);
	 settling_time = 0;
     intf =0;
     enable_interrupts(global);	
	 enable_interrupts(INT_RTCC);
     return;
}

//--------------------------------------------------------------------------
          // stop bit function //
//--------------------------------------------------------------------------
void  stopbit(void)
{
      output_bit(tx,1);
	  //restart_wdt(); 
      delay_us(830);
      return;
}

//--------------------------------------------------------------------------


void readData(void)
{
	//restart_wdt(); 
      error_flag=0;
      datacount++;
      forwrdFrameFlag = 0;
	  backwardFrameFlag =0;
      if(datacount< 27)
      {
         if((a==0 )&& (b==1))
         {
            shift_left(data,3,1);  // a one  detewcted on bus 
         }
         else if((a==1)&&(b==0))
         {
            shift_left(data,3,0);  // a zero is  deted on the bus 
         }
         else if ( a==1 && b==1)
         {
            switch (datacount)
            {
               case 17:
               {
                     stopBitCount ++;
                     break;
               }
               case 18:
               {
                  stopBitCount ++;
                  if(stopBitCount == 2)
                  {
                        r_a=1; 
                        copyData();
                        forwrdFrameFlag = 1;
                        masterflag = 0;
                        backwardFrameFlag =0;

                  }
                  else
                  {
						error_flag =1;
						clear_interrupt(int_ext);
						enable_interrupts(INT_EXT);
    				  disable_interrupts(int_timer1);
    		          enable_interrupts(INT_RTCC);
                  }
                  break;
               }
              	case 25:
				{
					stopBitCount ++;
					break;
				}
              	case 26: 
				{
					stopBitCount ++;
					if(stopBitCount == 2)
					{
						r_a=0; 
						copyData();
						forwrdFrameFlag =0;
						masterflag = 1;
						backwardFrameFlag =0;
					}
					else
					{
						error_flag =1;
						clear_interrupt(int_ext);
						enable_interrupts(INT_EXT);
    				  disable_interrupts(int_timer1);
    		          enable_interrupts(INT_RTCC);
					}
					break;
				}
                default:
                {
                      error_flag=1;
                      timerOnOff=0;
					  clear_interrupt(int_ext);
                      enable_interrupts(INT_EXT);
    				  disable_interrupts(int_timer1);
    		          enable_interrupts(INT_RTCC);
                      settling_time = 0;
                      break;
                }
             }   
          } 
		else
		{
			error_flag=1;    
			settling_time = 0;
			timerOnOff=0;    
			clear_interrupt(int_ext);   
			enable_interrupts(INT_EXT);
    		disable_interrupts(int_timer1);
    		enable_interrupts(INT_RTCC);         
		}
      }
	else  // the  data count grater than 27 
	{
		over_flowflag =1 ;
		settling_time = 0;
		timerOnOff=0;   
		clear_interrupt(int_ext);    
		enable_interrupts(INT_EXT);
        disable_interrupts(int_timer1);
        enable_interrupts(INT_RTCC);        
	}
    return;
}




void copyData(void)
{ 
	dataReady =1;    
	if( r_a==1)
	{
		address = data[1];
		command =data[0];						
	}
	else if( r_a==0)
	{	
		address = data[2];
		command =data[1];
		databyte=data[0]; 
	}       
    timerOnOff=0;
    intf =0;
	clear_interrupt(int_ext);
    enable_interrupts(INT_EXT);
    disable_interrupts(int_timer1);
    enable_interrupts(INT_RTCC);
    settling_time = 0;
    return;
}


void commands(void)
{ 
	command_st =0;	
	switch(command)
	{
	   	case 201:	// goto  level 
		{  
			
			if(databyte>= MaximumLevel )
			{
				output_high(pin_c2);
				duty = MaximumLevel;				
			}
			else if(databyte<= MinimumLevel )
		    {
				output_low(pin_c2);
				duty = 0;								
			}
			else
			{
				output_high(pin_c2)	;
				duty =databyte;							
			} 		
			lamp_on();										
			break;
		}
		case 208:	// on
		{  
			restart_delay=3000;
			restart_count=0;
			output_high(pin_c2);			
			duty = MaximumLevel;
			lamp_on();								
			break;
		}
		case 212:	//off
		{  
			output_low(pin_c2);
			duty =0;
			lamp_off();
			break;
		}
		case 216:	//dim
		case 241:		//ZONE DIM
		{
			if(l_st==1)
			{				
				if(duty>MinimumLevel)
				{							
					duty--;
					SetDimmLevel(duty);					
				}
			}
			break;
		}
		case 220:	//bright					
		case 240:  //zone  bright
		{
			if(l_st==1)
			{			
				if(duty < MaximumLevel)
				{									
					duty++;
					SetDimmLevel(duty);			
				}
			}
			break;
		}	
	
		case 234: // scene select 
		{				
			if(databyte < 17)
			{				
				currentSceen = databyte;			
		        duty = read_EEPROM (currentSceen+SceneStore);	
			     	if(duty<=MinimumLevel)
					{
						duty=0;
						output_low(pin_c2);				
						lamp_off();		
					}
					else
					{	
						output_high(pin_c2);								
						lamp_on();	
						SetDimmLevel(duty);		
					}			
			}
			break;
		 }
		case 231:  // store sceen 
		{
			if(databyte < 17)
			{				
				disable_interrupts (global);
				write_eeprom(databyte+SceneStore,duty);
				delay_us(5);			
				enable_interrupts(global);	
			}
			break;
		}
		case 9:
		{		
				GroupSelectReg = MAKE16(read_EEPROM (Group_815Store ),read_EEPROM (Group_07Store));	
				gindex = databyte &0x0f;				
				switch (databyte & 0x10)
				{
					case 0:
						{
							bit_clear(GroupSelectReg,gindex);
							write_eeprom(Group_07Store  ,make8(GroupSelectReg,0));
							delay_us(10);
							write_eeprom(Group_815Store,make8(GroupSelectReg,1));
 							delay_us(10);
							break;
						}
					case 16:
						{
							bit_set(GroupSelectReg,gindex);
							write_eeprom(Group_07Store  ,make8(GroupSelectReg,0));
							delay_us(10);
							write_eeprom(Group_815Store,make8(GroupSelectReg,1));
 							delay_us(10);
							break;
						}
					
					default: break;

				}
				break ;
		}
		case 34:    // store  short  aress 
		{
			if(databyte <64)
			{
					lampid = databyte;
					write_eeprom(ShortAddressStore ,lampid);
					delay_us(10);
			}
		
			break;		
		}
		case 35:    // write  DTR 
		{
					DTR = databyte;	
					DTR_Ready =1;
					break;
		}
		case 36:    // write  DTR  to  adress  location  in data  byte 
		{
					
				DwriteLocation = databyte;	
				if(DTR_Ready ==1 && DwriteLocation<33 )
				{
					DTR_Ready =0;
					write_eeprom(DwriteLocation,DTR);
					DELAY_US(20);
				}
			init_from_eeprom();
			break;
		}
		case 37:    // Read  DTR  to  adress  location  in data  byte 
		{
				tx_buffer[2]=lampid;tx_buffer[1]=DTR; txmit(2,2);  /////////priority changed
				break;			
		}
		case 38:    // Read  eeprom  and  store  in dtr   to  adress  location  in data  byte 
		{
				DwriteLocation = databyte;	
				if( DwriteLocation<33 )
				{
					DTR=Read_eeprom(DwriteLocation);
				}	
				break;				
		}
		case 39:	// Query current device power level
		{
			tx_buffer[2]=lampid;tx_buffer[1]=Read_eeprom(0); 
	    	txmit(2,2);		
			break;

		}
		case 49: 	// case for device type query
		{
			tx_buffer[2]=lampid;tx_buffer[1]=device_type; 
			txmit(2,2);			
			break;

		}
		case 50: 	// case for first group secion query 
		{
			tx_buffer[2]=lampid;tx_buffer[1]=Read_eeprom(7); 
			txmit(2,2);			
			break;

		}
		case 51: 	// case for second group secion query 
		{
			tx_buffer[2]=lampid;tx_buffer[1]=Read_eeprom(8); 
			txmit(2,2);			
			break;

		}
		case 42: 	// setting max level 
		{
			MaximumLevel=databyte;
			write_eeprom(MaximumLevelStore,MaximumLevel);
			delay_us(10);			
			break;

		}
		case 43: 	// setting min level 
		{
			MinimumLevel=databyte;
			write_eeprom(MinimumLevelStore,MinimumLevel);
			delay_us(10);		
			break;
		}
		case 48:	// PWM Restart operation			
		{
			ECCPASE=0;		
			break;
		}
		case 52:	// Read the no. of times system was shutdown permanently
		{
			tx_buffer[2]=lampid;tx_buffer[1]=Read_eeprom(SystemFailureRateStore); 
			txmit(2,2);
			break;
		}
		case 53:	// Set current protection threshold value
		{
			current_threshold=databyte;
			write_eeprom(CurrentThresholdStore,current_threshold);
			delay_us(10);
		}
		default:
		{
			command_st=1;
			break;
		}
	}
	if(command_st==0)
	{ 		
		  write_eeprom(PowerOnLevelStore,duty);		
	}
	return;
}

void lamp_on()
{	
	SetDimmLevel(duty);
	l_st=1;
	return;
}
	
void lamp_off()
{	
	SetDimmLevel(0);	
	l_st=0;
	return;
}


	
void SetDimmLevel(unsigned int dimPesentage)
{
	if(dimPesentage >= MaximumLevel)
		{
			dimPesentage = MaximumLevel;
		}
	else if(dimPesentage <= MinimumLevel)
		{
			dimPesentage = MinimumLevel;
		}
	
	if(dimPesentage <=MinimumLevel)
		{
			output_low(pin_c2);
		}
	else
		{
			output_high(pin_c2);
		}

	if(dimPesentage<95)
	{
		Power =dimPesentage;
	}
	if(dimPesentage>=95)
	{
		Power =1023;
	}
	if(Power > 1020){Power =1023;}
	set_pwm1_duty(Power);
	}


void init_from_eeprom(void)
{
GroupSelectReg = MAKE16(read_EEPROM (Group_815Store ),read_EEPROM (Group_07Store));	
delay_us(10);
PowerOnLevel 		= read_EEPROM (PowerOnLevelStore);
delay_us(10);    
MinimumLevel		= read_EEPROM ( MinimumLevelStore );   
delay_us(10);       	
MaximumLevel 		= read_EEPROM ( MaximumLevelStore); 
delay_us(10);  	
lampid 				= read_EEPROM ( ShortAddressStore );
delay_us(10);
zoneid=read_EEPROM(zoneidstore);
delay_us(10);
failure_count=read_EEPROM(SystemFailureRateStore);
delay_us(10);
current_threshold=read_EEPROM(CurrentThresholdStore);
delay_us(10);
}


