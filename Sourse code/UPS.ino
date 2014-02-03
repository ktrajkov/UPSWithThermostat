
#include <EEPROM.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#pragma region Pins
#define OUTPUT1      3
#define OUTPUT2	     4

#define OutputDisplay A3
#define RelayOutput   A1

#define AvailableAC220 2

#define BatteryVolt  A4

#define SensorPin    1
#define Piezo		A2

#define SetButton    5
#define UpButton     6
#define DownButton   7

#define PinCommLatch  A0
#define PinClock      13
#define PinData       12

#define BatteryLed    11
#define TempLed       10
#define Term12VLed    9
#define Term200vLed   8

#pragma endregion

#define ToleranceTemp 10
#define MaxTemp        110



#define AddrMode 0
#define AddrTermpstat12V 1
#define AddrTermostat220V 2

int led[]={BatteryLed,TempLed,Term12VLed,Term200vLed};

boolean upsState=false;
boolean enterSetupMode=false;
boolean updateDisplay=false;
boolean checkTemp=false;
boolean ckeckBattery=false;
boolean ckeckBatteryLoop=false;

boolean availableAC;
boolean changeStateAvailableAC=false;
byte mode;//0=batter,1=temp,2=term12v,3=term220v

byte floatDigit;
const int segments = 3;
byte digitsArray[segments];
byte digits [11];


byte termostatTemp12V;
byte termostatTemp220V;
int tempTermostatTemp12V;
int tempTermostatTemp220V;
			
boolean setTermostatTemp12V;
boolean setTermostatTemp220V;

boolean outputState=false;

OneWire oneWire(SensorPin);
DallasTemperature sensor(&oneWire);
DeviceAddress insideThermometer;
boolean availableSensor=false;

boolean changeModeAvailableSensors=true;
boolean changeModeNotAvailableSensors=true;

unsigned long nextMillis = 0;       
unsigned long currentMillis;
unsigned long timeSetTemp =1500;

long milissss=0;
int timePressButton=0;


long interval = 100; 


float Voltage=0;
#define R1  21900.0
#define R2  4580.0


#define minValueVoltage 10.0
#define alertMinVoltage 10.5
boolean alertState=false;
boolean piezoState=false;

int loopp=0;
int looppBlinkLed=0;


float currentTemp=0.0;


byte loop50Hz=0;

byte TimeStartSound=100;
byte StartSound=false;
byte SoundUps=false;
boolean loopSoundState=false;
int StartSoundloop=0;

int loopSound=0;
boolean playLoopSound=false;
void setup()
{
	

	// Setup the digits array
  // a = 8 b = 4 c = 2 d = 64 e = 32 f = 1 g = 16
	#pragma region IOSettings
	digits [0] = B00000011;
	digits [1] = B00111111;
	digits [2] = B01010001;
	digits [3] = B00010101;
	digits [4] = B00101101;
	digits [5] = B10000101;
	digits [6] = B10000001;
	digits [7] = B00011111;
	digits [8] = B00000001;
	digits [9] = B00000101;
	digits[10] = B11111110;

	pinMode(OutputDisplay, OUTPUT);
	pinMode(Piezo, OUTPUT);
	pinMode(OUTPUT1, OUTPUT);
	pinMode(OUTPUT2, OUTPUT);

	pinMode (PinCommLatch, OUTPUT);
    pinMode (PinClock, OUTPUT);
    pinMode (PinData, OUTPUT);

	pinMode(BatteryLed, OUTPUT);
	pinMode(TempLed, OUTPUT);
	pinMode(Term12VLed, OUTPUT);
	pinMode(Term200vLed, OUTPUT);

	pinMode(RelayOutput,OUTPUT);
	pinMode(AvailableAC220, INPUT_PULLUP);
	pinMode(SetButton, INPUT_PULLUP);
	pinMode(UpButton, INPUT_PULLUP);
	pinMode(DownButton, INPUT_PULLUP);
	
#pragma endregion

	#pragma region LoadSeetingsFromEEPROM
	mode=EEPROM.read(AddrMode);
	if(mode>3)
	{
		mode=0;
	}
	termostatTemp12V=EEPROM.read(AddrTermpstat12V);
	if(termostatTemp12V>MaxTemp)
	{
		termostatTemp12V=MaxTemp;
	}
	tempTermostatTemp12V=termostatTemp12V;	
	termostatTemp220V=EEPROM.read(AddrTermostat220V);
	if(termostatTemp220V>MaxTemp)
	{
		termostatTemp220V=MaxTemp;
	}
	tempTermostatTemp220V=termostatTemp220V;
	setTermostatTemp12V=termostatTemp12V;
	setTermostatTemp220V=termostatTemp220V;
#pragma endregion

	digitalWrite(OutputDisplay,HIGH);	
	ReadBatteryVoltage();
	sensor.begin();	
	availableSensor=sensor.getAddress(insideThermometer, 0);
	if(availableSensor)
	{
		sensor.requestTemperatures();
		currentTemp=sensor.getTempC(insideThermometer);
		
	}
	else
	{
		mode=0;		
	}
	
	UpdateLed();
	UpdateDisplay();
	Start();
	availableAC=!digitalRead(AvailableAC220);
	changeStateAvailableAC=!availableAC;	
}


void loop()
{
	#pragma region UpdateDisplay
	if(updateDisplay)
	{
		UpdateDisplay();
		updateDisplay=false;
	}
	#pragma endregion

	#pragma region checkTemp
	if(checkTemp)
	{
		checkTemp=false;
		availableSensor=sensor.getAddress(insideThermometer, 0);
		if(availableSensor)
		{
			
			sensor.requestTemperatures();
			currentTemp=sensor.getTempC(insideThermometer);
			if(changeModeAvailableSensors)
			{
				setTermostatTemp12V=termostatTemp12V;
				setTermostatTemp220V=termostatTemp220V;
				UpdateLed();
				
				
				changeModeAvailableSensors=false;
				changeModeNotAvailableSensors=true;
			}
			if(!availableAC)//no ACVoltage
			{
				NotAvailableAcAndSensor();
			}
			else 
			{
				AvailableAcAndSensor();
			}
			
			
			
		}
		else 
		{
			if(changeModeNotAvailableSensors)
			{
				mode=0;
				setTermostatTemp12V=false;
				setTermostatTemp220V=false;				
				UpdateLed();
				UpdateDisplay();
				changeModeAvailableSensors=true;
				changeModeNotAvailableSensors=false;
				if(!availableAC)//no ACVoltage
				{
					TurnOnInverter();
					digitalWrite(RelayOutput,HIGH);
				}
				else 
				{
						TurnOffInverter();
						digitalWrite(RelayOutput,HIGH);

				}

			}
			
		}

		
		
	}
	#pragma endregion

	#pragma region ckeckBatteryLoop
	if(ckeckBatteryLoop)
	{
		ckeckBatteryLoop=false;
		CkeckBattery();
		

	}
	#pragma endregion

	if(availableSensor)
	{
		#pragma region SetButton
	if(!digitalRead(SetButton))
	{
		WaitButton();		
		if(!digitalRead(SetButton))
		{
			if(mode<2)
			{
				NextMode();
				while(!digitalRead(SetButton))
					{
					}
			}
			else
			{
				if(enterSetupMode)
				{			
					enterSetupMode=false;
					while(!digitalRead(SetButton))
					{
					}
					termostatTemp12V=tempTermostatTemp12V;
					termostatTemp220V=tempTermostatTemp220V;
					EEPROM.write(AddrTermpstat12V,termostatTemp12V);
					EEPROM.write(AddrTermostat220V,termostatTemp220V);

					UpdateLed();
				}
				else
				{
					while(!digitalRead(SetButton))
					{
						if(nextMillis + timeSetTemp < millis())
						{
							termostatTemp12V=tempTermostatTemp12V;
							termostatTemp220V=tempTermostatTemp220V;
							enterSetupMode=true;							
						}
					}
				
					if(!enterSetupMode)
					{
						NextMode();			
					}
				}		
		
						
			}
			WaitButton();		
		}
	}
	
#pragma endregion
		if(enterSetupMode)
		{
			#pragma	 region		UpButton
		if(!digitalRead(UpButton))
		{
				WaitButton();
				if(!digitalRead(UpButton))
				{
					if(mode==2)
					{
						
						tempTermostatTemp12V+=5;
						if(tempTermostatTemp12V>MaxTemp)
						{
							tempTermostatTemp12V=MaxTemp;
						}
						setTermostatTemp12V=true;
						
					}
					else 
					{
						
						tempTermostatTemp220V+=5;
						if(tempTermostatTemp220V>MaxTemp)
						{
							tempTermostatTemp220V=MaxTemp;
						}
						setTermostatTemp220V=true;
						
					}
					UpdateDisplay();
					
				}
				WaitButton();
		}
#pragma endregion
			#pragma region DownButton
		if(!digitalRead(DownButton))
		{
			WaitButton(); 
			if(!digitalRead(DownButton))
			{
				if(mode==2)
				{
						
					tempTermostatTemp12V-=5;
					if(tempTermostatTemp12V<=0)
					{
						tempTermostatTemp12V=0;
						setTermostatTemp12V=false;
					}
						
				}
				else 
				{
						
					tempTermostatTemp220V-=5;
					if(tempTermostatTemp220V<=0)
					{	
						tempTermostatTemp220V=0;
						setTermostatTemp220V=false;
					}
					
				}
				UpdateDisplay();
				
			}
			WaitButton();
		}
		#pragma endregion		
		}
	}
		
	#pragma region UpsState
	availableAC=!digitalRead(AvailableAC220);
	if(changeStateAvailableAC!=availableAC)
	{
		
		changeStateAvailableAC=availableAC;		
		Desition();
	
	
	}
	#pragma endregion
}






ISR(TIMER1_COMPA_vect)
{
	loop50Hz++;
	 if(loop50Hz==30)
	 {
		 if(ckeckBattery)
		 {
			 ckeckBattery=false;
			 ckeckBatteryLoop=true;
		 }
	 }
	if(loop50Hz>60)
	{
		#pragma region A
		loopp++;
		
		
		if(upsState)
		{
			if(outputState)
			{
				digitalWrite(OUTPUT1,LOW);
					digitalWrite(OUTPUT2,HIGH);
			}
			else
			{
				digitalWrite(OUTPUT2,LOW);
					digitalWrite(OUTPUT1,HIGH);
			}
			outputState=!outputState;
			if(loopSoundState)
			{
				loopSound++;
				if(loopSound>2000)
				{
					playLoopSound=true;
					if(loopSound>2010)
					{
						playLoopSound=false;
						loopSound=0;
					}
				}
			}
				
		}
		if(StartSound)
		{
			StartSoundloop++;
			
			if(StartSoundloop==500)
			{
				StartSound=false;
				loopSoundState=true;
				loopSound=0;
			
			}
		}
		
	
		if(loopp==100) // 5 sec.
		{
			checkTemp=true;
		}
		if(loopp==200)
		{
			if(mode<2)
			{
				updateDisplay=true;
			}
		}
		if(loopp>300)
		{
		
			ckeckBattery=true;
			loopp=0;
		}

		if(enterSetupMode)
		{
			looppBlinkLed++;
			if(looppBlinkLed>50)
			{
				looppBlinkLed=0;
				digitalWrite(led[mode],!digitalRead(led[mode]));
			}
		}
		
			
#pragma endregion 
		loop50Hz=0;
	}
	if(alertState||StartSound||playLoopSound)
	{
		piezoState=!piezoState;
		digitalWrite(Piezo,piezoState);
	}
	

}


void pin32Interrupt()
{
	digitalWrite(OutputDisplay,HIGH);	
	detachInterrupt(0);
	Desition();
}

void Start()
	{
			cli();          // disable global interrupts
			TCCR1A = 0;     // set entire TCCR1A register to 0
			TCCR1B = 0;     // same for TCCR1B
			 TCNT1  =0;//initialize counter value to 0
			// set compare match register to desired timer count:
			OCR1A =40;

			// turn on CTC mode:
			TCCR1B |= (1 << WGM12);
			// Set CS10 and CS12 bits for 64 prescaler:
			TCCR1B |= (1 << CS10);
			TCCR1B |= (1 << CS11);
			// enable timer compare interrupt:
			TIMSK1 |= (1 << OCIE1A);
			sei();
	}

void Stop()
	{
		TCCR1B &= ~(_BV(CS10) | _BV(CS11) | _BV(CS12));
		
	}


void TurnOffUps()
	{
		
		TurnOffInverter();
		digitalWrite(OutputDisplay,LOW);
		digitalWrite(RelayOutput,LOW);
		 /* Setup pin2 as an interrupt and attach handler. */
	

	
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
		sleep_enable();
		attachInterrupt(0, pin32Interrupt, LOW);
	
		sleep_mode();		
		sleep_disable(); 
			
	}

void TurnOffInverter()
{
	
			upsState=false;
			
			digitalWrite(OUTPUT1, LOW);
			digitalWrite(OUTPUT2, LOW);
}

void TurnOnInverter()
{
	upsState=true;
	digitalWrite(RelayOutput,HIGH);
}


void NextMode()
{
	
	if(mode==3)
	{	
		mode=0;
	}
	else mode++;
	EEPROM.write(AddrMode,mode);
	UpdateLed();
	UpdateDisplay();
}

void UpdateDisplay()
{
	float displayValue;
	switch (mode)
	{
	case 0:
		{
			displayValue=Voltage;
			break;
		}
	case 1:
		{
			displayValue=currentTemp;
			break;  
		}
	case 2:
		{
			displayValue=tempTermostatTemp12V;
			break;
		}
	case 3:
		{
			displayValue=tempTermostatTemp220V;
		}				
	}			
	LoadValueToDigitsArray(displayValue);
	SendSerialData(segments, digitsArray);
}

void LoadValueToDigitsArray(float displayValue)
{		
	int mappedRead=0;
	if(displayValue>=100)
	{
		mappedRead=displayValue*10;
		digitsArray[0] = digits[(mappedRead / 10) % 10];
		digitsArray[1] = digits[(mappedRead / 100) % 10];
		digitsArray[2] = digits[(mappedRead / 1000) % 10];
	}
	else 
	{
		mappedRead=displayValue*100;
		floatDigit = mappedRead % 10;
		if(floatDigit>5)
		{
			mappedRead +=10;
		}
		digitsArray[0] = digits[(mappedRead / 10) % 10];
		digitsArray[1] = digits[(mappedRead / 100) % 10] - 1;//add point 
		digitsArray[2] = digits[(mappedRead / 1000) % 10];
	}
}

void SendSerialData (byte registerCount,byte *pValueArray)  
{ 
  digitalWrite (PinCommLatch, LOW);  
  for (byte reg = registerCount; reg > 0; reg--)
  {
    byte value = pValueArray [reg - 1];    
    for (byte bitMask = 128; bitMask > 0; bitMask >>= 1)
    {
      digitalWrite (PinClock, LOW);    
      digitalWrite (PinData, value & bitMask ? HIGH : LOW);        
      digitalWrite (PinClock, HIGH);
    }
  }  
  digitalWrite (PinCommLatch, HIGH);
} 

void UpdateLed()
{
	digitalWrite(led[0],LOW);
	digitalWrite(led[1],LOW);

	if(mode<2)
	{
		digitalWrite(Term12VLed,setTermostatTemp12V);
		digitalWrite(Term200vLed,setTermostatTemp220V);
	}
	else 
	{
		digitalWrite(Term12VLed,LOW);
		digitalWrite(Term200vLed,LOW);
	}
	digitalWrite(led[mode],HIGH);
}

void WaitButton()
{
	nextMillis  = millis() + interval;
	while(nextMillis > millis()) 
	{		
	}
}

void Desition()
{
	if(!availableAC)//no ACVoltage
	{
		//StartSound=true;
		StartSoundloop=0;
		if(availableSensor && termostatTemp12V)
		{
			NotAvailableAcAndSensor();
		}
		else //no Sensor or thermostat
		{
			TurnOnInverter();
			digitalWrite(RelayOutput,HIGH);
		}
	}
	else //there a voltage
	{
		loopSoundState=false;
		alertState=false;
		TurnOffInverter();
		if(availableSensor && termostatTemp220V)
		{
			AvailableAcAndSensor();
		}
		else 
		{
			digitalWrite(RelayOutput,HIGH);
		}
	}
}

void AvailableAcAndSensor()
{
	if(currentTemp>termostatTemp220V)
	{
		digitalWrite(RelayOutput,HIGH);
	}
	else 
	{
		if (currentTemp<termostatTemp220V - ToleranceTemp)
		{
			digitalWrite(RelayOutput,LOW);
		}
						
	}
}

void NotAvailableAcAndSensor()
{
	if(currentTemp>termostatTemp12V)
	{
		TurnOnInverter();	
		digitalWrite(RelayOutput,HIGH);
	}
	else 
	{
		if (currentTemp<termostatTemp12V - ToleranceTemp)
		{
			TurnOffInverter();
			digitalWrite(RelayOutput,LOW);
		}						
	}
}

void CkeckBattery()
{
	ReadBatteryVoltage();
	if (availableAC&&Voltage<alertMinVoltage)
	{
		alertState=true;
		if (Voltage<minValueVoltage)
		{						
			TurnOffUps();
		}
	}	
}

void ReadBatteryVoltage()
{
	Voltage = ((analogRead(BatteryVolt)*5.0) / 1024.0) / (R2 / (R1 + R2)) + 0.2; // 5.0 V of Vcc
}
