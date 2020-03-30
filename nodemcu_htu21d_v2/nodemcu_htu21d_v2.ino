/*
 * ESTE ES el que va
 */

     

/* 
 HTU21D Humidity Sensor Example Code
 By: Nathan Seidle
 SparkFun Electronics
 Date: September 15th, 2013
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
 
 Get humidity and temperature_actual from the HTU21D sensor.
 
 Hardware Connections (Breakoutboard to Arduino):
 -VCC = 3.3V
 -GND = GND
 -SDA = A4
 -SCL = A5
 
 Serial.print it out at 9600 baud to serial monitor.
 */
#include <ThingerESP8266.h>
#include <Wire.h>
#define BLYNK_PRINT Serial
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "tEhwrgvlf9AzZ-BjXpblwMTehnB4ZgDI";

// Your WiFi credentials.
// Set password to "" for open networks.
// char ssid[] = "Tutuca.";
// char pass[] = "20junio2016";


char ssid[] = "a la grande le puse cuca";
char pass[] = "capocha123";

#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor

#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG  0xE6
#define READ_USER_REG  0xE7
#define SOFT_RESET  0xFE

//Credenciales para el thing.io
#define USERNAME "javners"
#define DEVICE_ID "node"
#define DEVICE_CREDENTIAL "123456"

byte sensorStatus;
BlynkTimer timer;

float temperature_actual;
float relativeHumidity;
IPAddress ipAddress;  
String ipstring;
String port_state[4];
bool token =false;
int setpoint_int=32;
float setpoint_float;
bool flag_ventilador_on;
float sp_temp_min_float;
int sp_temp_min_int=28;

//Variable para thinger.io
float temp_para_thinger_io;  
float humedad_para_thinger_io;
int  led_D0_thinger_io; 
int  led_D1_thinger_io;
int  led_D2_thinger_io;
int  led_D4_thinger_io;





void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  String temp_str;
  
  temp_str=String(temperature_actual) + String("c") + String(" ") + String(relativeHumidity)+ String("%");
  Blynk.virtualWrite(V5, temp_str);
  
  Blynk.virtualWrite(V1, ipstring);
}

WiFiServer server(80);
ThingerESP8266 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

void setup()
{
//Declaracion de puertos I/O
	{	
		pinMode(D0,OUTPUT);
		pinMode(D1,OUTPUT);
		pinMode(D2,OUTPUT);
		pinMode(D4,OUTPUT);

		digitalWrite(D0,HIGH);
		digitalWrite(D1,HIGH);
		digitalWrite(D4,HIGH);
		digitalWrite(D2,HIGH);
	}
  
  
  Serial.begin(9600);
  Serial.println("HTU21D Example!");
  Serial.println();
  Serial.println();
  
  /* Conexion con thinher.IO*/
  thing.add_wifi(ssid, pass);
 
  Serial.print("Connecting to ");
  Serial.println(ssid);
 
  /* Inicializo la placa WIFI */
	WiFi.begin(ssid, pass);
  
   while (WiFi.status() != WL_CONNECTED) 
   {
     delay(500);
     Serial.print(".");
   }
   Serial.println("");
   Serial.println("WiFi connected");
 
   /* Start the server*/
   server.begin();
   Serial.println("Server started");
 
   /* Print the IP address*/
	Serial.print("Use this URL to connect: ");
	Serial.print("http://");
	Serial.print(WiFi.localIP());
 
 {/*******************************************************************************************************/
 /*Esta parte de codigo se implemento para mostrar la IP en el Display de BLYNK*/
 // 	ipstring=WiFi.localIP().toString().c_str();
 //		ipAddress=WiFi.localIP();
 // 	ipstring=String(ipAddress[0])+ String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]) ;
 //		Blynk.virtualWrite(V1, "ipstring");
 //  	Serial.print(ipstring);
 //		Serial.println("/");
/*******************************************************************************************************/
}
  Wire.begin(D5,D6);
  Blynk.begin(auth, ssid, pass);
  timer.setInterval(1000L, myTimerEvent);
  
  
  /*******************************************************************************************************/
  /* En esta parte se colocan las variable que vas a ser leidas/escritas por thinger.io*/
   thing["node"] >> [](pson& out)
   {
  
    out["Temperatura"] = temp_para_thinger_io;
	out["Humedad"] = humedad_para_thinger_io;
	out["Led_D0"] =  led_D0_thinger_io;       //para el led indicator debe ser una variable entera
	out["Led_D1"] =  led_D1_thinger_io; 
	out["Led_D2"] =  led_D2_thinger_io; 
	out["Led_D4"] =  led_D4_thinger_io; 
	out["setpoint_temp_max"] = setpoint_float ;
	out["setpoint_temp_min"] = sp_temp_min_float ;
	sp_temp_min_int = out ;
	
	
	};
}

void loop()
{ int control;
 

  thing.handle();
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
  
  unsigned int rawHumidity = htdu21d_readHumidity();
  unsigned int rawtemperature_actual = htdu21d_readTemp();
  

   temperature_actual = calc_temp(rawtemperature_actual);
   relativeHumidity = calc_humidity(rawHumidity); //Turn the humidity signal into actual humidity
  
  /*******************************************************************************************************/
  /* En esta parte del codigo refresco las variables para ser visualizadas en el dashboard de thinger.io*/
  temp_para_thinger_io = temperature_actual; 
  led_D0_thinger_io = flag_ventilador_on ;
  humedad_para_thinger_io = relativeHumidity ;
  
  /*******************************************************************************************************/
 



/*********************************************************************************************************/
/*  En esta parte del codigo estan las impresiones por puerto serie al Serial monitor de arduino         */
{

  Serial.print("Temp. Actual: ");
  Serial.print(temperature_actual, 1); //Print float with one decimal
  //Serial.print((char)223);
  Serial.print(" ºC");
  Serial.print(" Humedad Rel: ");
  Serial.print(relativeHumidity, 1);
  Serial.print(" %");
  Serial.print("   ");
  Serial.print("Setpoint Max : ");
  Serial.print(setpoint_float,1);
  Serial.print("   ");
  Serial.print("Setpoint Min : ");
  Serial.print(sp_temp_min_float,1);
  Serial.print("   ");
  Serial.print(" Flag ventilador: ");
  Serial.print(flag_ventilador_on);
  Serial.print("   ");
  Serial.println();
  delay(1000);


  
 } 
  /********************************************************************************************************/
  
  
  
  /********************************************************************************************************/
  /* En esta seccion se ha el lazo de control ON-OFF.*/
{	
	sp_temp_min_float=float(sp_temp_min_int);
	setpoint_float=float(setpoint_int);
	
	
if ((setpoint_float<temperature_actual))
    {
             if  ((sp_temp_min_float<temperature_actual) && (flag_ventilador_on ==LOW))
            {
            digitalWrite(D1, LOW);//enciendo ventilador
			flag_ventilador_on=HIGH ;// Flag ventilador encendido
			digitalWrite(D4,LOW);    //Esta salida se uso a modo de debbug.Se puede Borrar.
			 
			}
	}		
			
			
			
			
if 	((temperature_actual<(sp_temp_min_float))&&(flag_ventilador_on ==HIGH))		
				{
					digitalWrite(D1, HIGH);
					digitalWrite(D4,HIGH);
					flag_ventilador_on= LOW ;
					
				}

}	


   /*******************************************************************************************************/
   /* En esta seccion se utiliza para leer comandor por puerto serie del Serial monitor */
if (Serial.available())  
  {
		control=Serial.read();
				switch (control) 
					{
						case '0': flag_ventilador_on=HIGH ;
								break;
						case '1': flag_ventilador_on= LOW ;
								break;
						default:
    // statements
								break;
					}
  }
  
  
 //********************************************************************************************** 
 /*visualizacion de puertos para blynk y thinger.io */
{
if ((digitalRead(D0))==HIGH)
	{
		port_state[0]="D0:OFF  ";
		led_D0_thinger_io = 0 ;
		//Blynk.tweet("D0 OFF");
	}
else
	{
		port_state[0]="D0:ON  ";
		led_D0_thinger_io = 1 ;
 //Blynk.tweet("D0 ON");
	}

if ((digitalRead(D1))==HIGH)
	{
		port_state[1]="D1:OFF  ";
		led_D1_thinger_io = 0 ;
	}	
else
	{
		port_state[1]="D1:ON   ";
		led_D1_thinger_io = 1 ;
	}

if ((digitalRead(D2))==HIGH)
	{
		port_state[2]="D2:OFF  ";
		led_D2_thinger_io = 0 ;
	}
else
	{
		port_state[2]="D2 ON";
		led_D2_thinger_io = 1 ;
	}

if ((digitalRead(D4))==HIGH)
	{
		port_state[3]="D4:OFF";
		led_D4_thinger_io = 0 ;
	}
else
	{
		port_state[3]="D4:ON";
		led_D4_thinger_io = 1 ;
	}


if (token==false)
{
ipstring=port_state[0]+ " " + port_state[1];
token=true;
}
else{
ipstring=port_state[2]+ " " + port_state[3];
token=false;
}

}
//********************************************************************************************** 





	// Check if a client has connected
 {
		WiFiClient client = server.available();
			if (!client)
				{
				return;
				}
 
  // Wait until the client sends some data
 // Serial.println("new client");
 // while(!client.available()){
  //  temp=analogRead(A0);
   
      
  //  delay(1);
  //}
  
		client.flush();
		client.println("HTTP/1.1 200 OK");
		client.println("Content-Type: text/html");
		client.println(""); //  do not forget this one
		//client.println("Connection: close");
		//client.println("Refresh: 5");
		client.println();
		client.println("<!DOCTYPE HTML>");
		client.println("<html>");
		client.println("<br><br>");
	
//  client.print("Led pin is now: ");
 //  if(value == HIGH) {
//    client.print("On");
//  } else {
//    client.print("Off");
//  }
  
 // client.println("<br><br>");
    
 //client.println("<a href=\"/LED=ON\"\"><button>Turn On </button></a>");
 // client.println("<a href=\"/LED=OFF\"\"><button>Turn Off </button></a><br />");  
  // client.println("<br><br>");
 

		client.println("Temperatura actual:");
		client.println(temperature_actual);
		client.println("<br><br>");
  
		client.println("Humedad:");
		client.print(relativeHumidity);
		client.println("<br><br>");

		client.println("D0:");
		client.println(port_state[0]);
  
		client.println("<br><br>");
		client.println("D1:");
		client.println(port_state[1]);
  
		client.println("<br><br>");
		client.println("D2:");
		client.println(port_state[2]);
  
		client.println("<br><br>");
		client.println("D4:");
		client.println(port_state[3]);
  
  
  
  
//   client.println("<form>");
//  client.println("<input type = 'text' name = 'prueba' /> ");
//  
//  client.println("<input type='submit' value='Enviar'/>");
//  client.println("<br><br>");
// 
// client.println("</form>");
  client.println("</html>");
  //request="";
  client.stop();
 
  delay(1);
  //Serial.println("Client disonnected");
  //Serial.println("");
 }
}// fin de loop

//Read the uncompensated temperature_actual value
unsigned int htdu21d_readTemp()
{
  //Request the temperature_actual
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);
  Wire.endTransmission();

  //Wait for sensor to complete measurement
  delay(60); //44-50 ms max - we could also poll the sensor

  //Comes back in three bytes, data(MSB) / data(LSB) / CRC
  Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
  int counter = 0;
  while(Wire.available() < 3)
  {
    counter++;
    delay(1);
    if(counter > 100) return 998; //Error out
  }

  unsigned char msb, lsb, crc;
  msb = Wire.read();
  lsb = Wire.read();
  crc = Wire.read(); //We don't do anything with CRC for now

  unsigned int temperature_actual = ((unsigned int)msb << 8) | lsb;
  temperature_actual &= 0xFFFC; //Zero out the status bits but keep them in place

  return temperature_actual;
}

//Read the humidity
unsigned int htdu21d_readHumidity()
{
	byte msb, lsb, checksum;

  //Request a humidity reading
	Wire.beginTransmission(HTDU21D_ADDRESS);
	Wire.write(TRIGGER_HUMD_MEASURE_NOHOLD); //Measure humidity with no bus holding
	Wire.endTransmission();

  //Hang out while measurement is taken. 50mS max, page 4 of datasheet.
	delay(55);

  //Read result
	Wire.requestFrom(HTDU21D_ADDRESS, 3);

  //Wait for data to become available
	int counter = 0;
	while(Wire.available() < 3)
		{
				counter++;
				delay(1);
				if(counter > 100)
				return 0; //Error out
		}

  msb = Wire.read();
  lsb = Wire.read();
  checksum = Wire.read();

  unsigned int rawHumidity = ((unsigned int) msb << 8) | (unsigned int) lsb;
  rawHumidity &= 0xFFFC; //Zero out the status bits but keep them in place

  return(rawHumidity);
}


// float temp_thinger( float variable_temp_thiger_io){

// return variable_temp_thiger_io;



// }






//Given the raw temperature_actual data, calculate the actual temperature_actual
float calc_temp(int SigTemp)
{
  float tempSigTemp = SigTemp / (float)65536; //2^16 = 65536
  float realtemperature_actual = -46.85 + (175.72 * tempSigTemp); //From page 14

  return(realtemperature_actual);  
}

//Given the raw humidity data, calculate the actual relative humidity
float calc_humidity(int SigRH)
{
  float tempSigRH = SigRH / (float)65536; //2^16 = 65536
  float rh = -6 + (125 * tempSigRH); //From page 14

  return(rh);  
}

//Read the user register
byte read_user_register(void)
{
  byte userRegister;

  //Request the user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(READ_USER_REG); //Read the user register
  Wire.endTransmission();

  //Read result
  Wire.requestFrom(HTDU21D_ADDRESS, 1);

  userRegister = Wire.read();

  return(userRegister);  
}

//Write to the user register
//NOTE: We disable all bits except for measurement resolution
//Bit 7 & 0 = Measurement resolution
//Bit 6 = Status of battery
//Bit 5/4/3 = Reserved
//Bit 2 = Enable on-board heater
//Bit 1 = Disable OTP reload
void write_user_register(byte thing_to_write)
{
  byte userRegister = read_user_register(); //Go get the current register state
  userRegister &= 0b01111110; //Turn off the resolution bits
  thing_to_write &= 0b10000001; //Turn off all other bits but resolution bits
  userRegister |= thing_to_write; //Mask in the requested resolution bits

  //Request a write to user register
  Wire.beginTransmission(HTDU21D_ADDRESS);
  Wire.write(WRITE_USER_REG); //Write to the user register
  Wire.write(userRegister); //Write to the data
  Wire.endTransmission();
}

//Give this function the 2 byte message (measurement) and the check_value byte from the HTU21D
//If it returns 0, then the transmission was good
//If it returns something other than 0, then the communication was corrupted
//From: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html
//POLYNOMIAL = 0x0131 = x^8 + x^5 + x^4 + 1 : http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
#define SHIFTED_DIVISOR 0x988000 //This is the 0x0131 polynomial shifted to farthest left of three bytes

unsigned int check_crc(uint16_t message_from_sensor, uint8_t check_value_from_sensor)
{
  //Test cases from datasheet:
  //message = 0xDC, result is 0x79
  //message = 0x683A, result is 0x7C
  //message = 0x4E85, result is 0x6B

  uint32_t remainder = (uint32_t)message_from_sensor << 8; //Pad with 8 bits because we have to add in the result/check value
  remainder |= check_value_from_sensor; //Add on the check value

  uint32_t divsor = (uint32_t)SHIFTED_DIVISOR;

  for (int i = 0 ; i < 16 ; i++) //Operate on only 16 positions of max 24. The remaining 8 are our remainder and should be zero when we're done.
  {
    //Serial.print("remainder:  ");
    //Serial.println(remainder, BIN);
    //Serial.print("divsor:     ");
    //Serial.println(divsor, BIN);
    //Serial.println();

    if( remainder & (uint32_t)1<<(23 - i) ) //Check if there is a one in the left position
      remainder ^= divsor;

    divsor >>= 1; //Rotate the divsor max 16 times so that we have 8 bits left of a remainder
  }

  return remainder;

}
 BLYNK_WRITE(V2) // V5 is the number of Virtual Pin  
{
   setpoint_int = param.asInt();
} 

BLYNK_WRITE(V0) // V5 is the number of Virtual Pin  
{
   sp_temp_min_int = param.asInt();
} 