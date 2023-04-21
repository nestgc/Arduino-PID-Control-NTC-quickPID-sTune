#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <LiquidMenu.h>

//Crear el objeto lcd  dirección  0x3F y 16 columnas x 2 filas
LiquidCrystal_I2C lcd(0x27 ,16,2);  // conexion LCD I2C
/*
  Smooth NTC Thermistor
  Reads a temperature from the NTC 3950 thermistor,
  smooth and displays it in the default Serial.
  https://github.com/YuriiSalimov/NTC_Thermistor
  Created by Yurii Salimov, May, 2019.
  Released into the public domain.
*/
#include <Thermistor.h> //libreria para lectura de temperatura NTC
#include <NTC_Thermistor.h> //libreria para lectura de temperatura NTC
//#include <SmoothThermistor.h> //libreria para lectura de temperatura NTC
#include "OneButton.h" //libreria para botones
//#include <PID_v1.h> //libreria para control PID
//#include <PID_AutoTune_v0.h>
#include <QuickPID.h>
#include <sTune.h>
//#include <PIDAutotuner.h>
#include <EEPROMex.h> //libreria para memoria eprom
#define _EEPROMEX_DEBUG   //se Defind Debug para poder limitar la cantidad de escritura a la eprom (cuando se realizan pruebas)

///////////////////////////////////////////////////////////////////////////////////////////////////////////

#define SENSOR_PIN             A0    //pin de lectura de temperatura NTC
#define REFERENCE_RESISTANCE   4700    //resistencia de referencia 4.7 kohm
#define NOMINAL_RESISTANCE     100000  //resistencia de NTC
#define NOMINAL_TEMPERATURE    25     //temperatura nominal NTC
#define B_VALUE                3950   // valot Beta NTC
///////////////////////////////////////////////////////////////////////////////////////////////////////////

//Pins
byte PWM_pin = 6;    //pin Salida control Hotend PID
/**
  Smoothing factor of a temperature value.
*/
#define SMOOTHING_FACTOR 0 // smoothing factor a 0 no suaviza la lectura de temperatura

Thermistor* thermistor = NULL; //variable de lectura de temperatura NTC



//variables

// float temperature_read = 0.0;
// float PID_error = 0;
// float previous_error = 0;
// float elapsedTime, Time, timePrev;
float PID_value = 0;  //valor de Salida PID a PWM
//int PID_valueint = 0;
// float last_set_temperature = 0;

//int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
/*
 * The Button class is not a part of the LiquidMenu library. The first
 * parameter is the button's pin, the second enables or disables the
 * internal pullup resistor (not required) and the third is the debounce
 * time (not required).
 */
// Button objects instantiation
const bool pullup = true;    //configurar entrada de botones pullup interno arduino
//PID constants
//////////////////////////////////////////////////////////
//double kp = 90;   double ki = 30;   double kd = 80;
//double kp = 5.66;   double ki = 0.21;   double kd = 0.1;  //valores por defecto PID
//float kp=11.75,ki=0.57,kd=60.97; //valores por defecto PID
//float kp=3.95,ki=0.01,kd=0.05; //valores por defecto PID
//float kp=3.19,ki=0.02,kd=0.06; //valores por defecto PID
//float kp=5.66,ki=0.21,kd=0.1; //valores por defecto PID
float kp=9.00,ki=0.11,kd=0.5; //valores por defecto PID
//double kp_new = 5.66;   double ki_new = 0.21;   double kd_new = 0.1;
//double kp_old = 5.66;   double ki_old = 0.21;   double kd_old = 0.1;
//int kp = 9.1;   int ki = 0.3;   int kd = 1.8;
//////////////////////////////////////////////////////////
float celsius = 0;  //variable lectura de temperatura Celsius
//int hotendSP = 200;
float hotendSP = 230;            //Default temperature setpoint
float kstep = 0.01;  //incremento decremento de kp ki kd
//double hotendSP_new = 100;            //Default temperature setpoint
//double hotendSP_old = 100;            //Default temperature setpoint
bool startstop = false;           //variable inicio detencion de Hotend
bool startstop_old = false;       //estado anterior del hotend
bool autotunepid = false;         //variable inicio detencion de autotune PID
bool aTuneRun = false;            //variable inicio detencion de autotune PID
char startstopchar[10]= "STOPPED";  //variable dialogo de inicio o parada de hotend
char autotunepid_char[10]= "STOPPED";  //variable dialogo de inicio o parada de hotend
//char startstopchar_pid[10]= "";  //variable dialogo de inicio o parada de hotend

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// user settings autotune PID
uint32_t settleTimeSec = 200; // time to wait for temperature to settle before starting test
uint32_t testTimeSec = 500;  // runPid interval = testTimeSec / samples
const uint16_t samples = 500; // number of samples to collect
const float inputSpan = 250; // temperature range to test
const float outputSpan = 255; // output range to test Arduino PWM 0-255
float outputStart = 70;  // output start value 0 apagado 255 encendido
float outputStep = 100; // output step size valor salida del PID para calcular el error
float tempLimit = 280;  // temperature limit to stop test
//uint8_t debounce = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////

int addressinit,addresshotendSP, addresskp, addresski, addresskd;  //variables de direcciones de Eprom



const byte maxAllowedWrites = 10;  //variable maxiimo de escritura en eprom durante las pruebas
const int memBase          = 0;   //direccion inicio memoria eprom

///////////////////////////////////////////////////////////////////////////////////////////////////////////

bool enterold = false;      //variable indicadora enter ya presionado
//byte line;
//int inthotendSP = (int)hotendSP;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

OneButton left(12, pullup);     // configuracion del boton izquierdo
OneButton right(10, pullup);    // configuracion boton derecho
OneButton enter(11, pullup);    // configuracion enter


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//menu inicial
LiquidLine welcome_line1(0, 0, "T:", celsius," S:",hotendSP);   //linea 1 pantalla inicial
LiquidLine welcome_line2(0, 1, startstopchar);  //linea 2 pantalla inicial
LiquidScreen welcome_screen(welcome_line1, welcome_line2);  //pantalla inicial

//menu segunda pantalla hotend setpoint
LiquidLine hotendsp_line(0, 0, "T setp: ", hotendSP);   //linea 1 pantalla hotend setpoint
LiquidLine hotendspcm_line(0, 1, "hotend setp ");   //linea 2 pantalla hotend setpoint
LiquidScreen hotendsp_screen(hotendsp_line, hotendspcm_line);   //pantalla hotend setpoint

//menu tercera pantalla autotune pid
LiquidLine auto_pid(0, 0,"Auto PID");
LiquidLine auto_pid_status(0, 1, autotunepid_char);
LiquidScreen auto_pid_screen(auto_pid, auto_pid_status);

//menu cuarta pantalla KP
LiquidLine kpsp_line(0, 0,kp);  //linea 1 pantalla KP
LiquidLine kpcm_line(0, 1,"P val");  //linea 2 pantalla KP
LiquidScreen kp_screen(kpsp_line, kpcm_line);   //pantalla KP

//menu quinta pantalla KI
LiquidLine kisp_line(0, 0,ki);  //linea 1 pantalla KI
LiquidLine kicm_line(0, 1,"I  val");  //linea 2 pantalla KI
LiquidScreen ki_screen(kisp_line, kicm_line);   //pantalla KI

//menu sexta pantalla KD
LiquidLine kdsp_line(0, 0, kd);   //linea 1 pantalla KD
LiquidLine kdcm_line(0, 1,"D  val");  //linea 2 pantalla KD
LiquidScreen kd_screen(kdsp_line, kdcm_line);   //pantalla KD

LiquidMenu menu(lcd);   //inicializacion menu


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//inicializacion de PID y el stune
//PID myPID(&celsius, &PID_value, &hotendSP, kp, ki, kd, DIRECT); //PID Directo ya que el auto PID no tiene la opcion reverso, se debe en la salida restar a 255
//QuickPID myPID(&celsius, &PID_value, &hotendSP, kp, ki, kd, myPID.pMode::pOnErrorMeas, myPID.dMode::dOnMeas, myPID.iAwMode::iAwClamp, myPID.Action::direct); //configuracion PID Reverse para que salida sea 255 cuando debe estar apagado y sea 0 cuando debe estar encendido
//QuickPID myPID(&celsius, &PID_value, &hotendSP, kp, ki, kd, myPID.Action::direct); //configuracion PID Reverse para que salida sea 255 cuando debe estar apagado y sea 0 cuando debe estar encendido
QuickPID myPID(&celsius, &PID_value, &hotendSP); //configuracion PID Reverse para que salida sea 255 cuando debe estar apagado y sea 0 cuando debe estar encendido
sTune aTune(&celsius, &PID_value, aTune.ZN_PID, aTune.directIP, aTune.printALL);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void hotendsp_up() {      //funcion hotend SP up
	if (hotendSP < 290) {   //maximo 290 grados
		hotendSP += 1;     //incremento de 1 grado
	} else {              //si se supera el maximo
		hotendSP = 290;  //se establece el maximo
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void hotendsp_down() {      //funcion hotend SP down
	if (hotendSP > 25) {   //minimo 25 grados
		hotendSP -= 1;   //decremento de 1 grado
	} else {           //si se supera el minimo
		hotendSP = 0; //se establece el minimo
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kp_up() {      //funcion hotend KP up
	kp = kp+kstep ; //incremento de 0.01
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kp_down() {    //funcion hotend KP down
  if (kp >= kstep) {  //minimo 0.01
	kp = kp-kstep; //decremento de 0.01
  } else {          //si se supera el minimo
    kp = 0;    //se establece el minimo
  }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ki_up() {      //funcion hotend KI up
	ki = ki+kstep ; //incremento de 0.01
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ki_down() {    //funcion hotend KI down
  if (ki >= kstep) {  //minimo 0.01
	ki = ki-kstep; //decremento de 0.01
  } else {          //si se supera el minimo
    ki = 0;    //se establece el minimo
  }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kd_up() {      //funcion hotend KD up
	kd = kd+kstep ; //incremento de 0.01
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void kd_down() {    //funcion hotend KD Down
  if (kd >= kstep) {  //minimo 0.01
	kd = kd-kstep; //decremento de 0.01
  } else {          //si se supera el minimo
    kd = 0;    //se establece el minimo
  }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EVsave() {     //funcion guardar si no es igual las variables de funcionamiento
  EEPROM.updateDouble(addresshotendSP,hotendSP); //guardar en la direccion 01 la variable hotendSP
  EEPROM.updateDouble(addresskp,kp); //guardar en la direccion 09 la variable kp
  EEPROM.updateDouble(addresski,ki); //guardar en la direccion 17 la variable ki
  EEPROM.updateDouble(addresskd,kd); //guardar en la direccion 25 la variable kd
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void EVread() {   // funcion de leer o inicializar las variables de la Eprom
  // Serial.print(F("direccion inicial: "));   //imprimir en el monitor serie
  // Serial.print(addressinit);               //imprimir en el monitor serie
  // Serial.println();                      //imprimir en el monitor serie
  // Serial.print(F("direccion hotendsp: ")); //imprimir en el monitor serie
  // Serial.print(addresshotendSP);        //imprimir en el monitor serie
  // Serial.println();
  // Serial.print(F("direccion kp: "));
  // Serial.print(addresskp);
  // Serial.println();
  // Serial.print(F("direccion ki: "));
  // Serial.print(addresski);
  // Serial.println();
  // Serial.print(F("direccion kd: "));
  // Serial.print(addresskd);
  // Serial.println();
  if (EEPROM.readByte(addressinit) == 84) {  //leer o inicializar las variables si la direccion 00 es igual a 84 solo lee si no inicializa con los valores de else
  // Serial.print(F("leyendo variables"));  //imprimir en el monitor serie
hotendSP = EEPROM.readDouble(addresshotendSP); //leer la direccion 01 y guardar en la variable hotendSP
kp       = EEPROM.readDouble(addresskp); //leer la direccion 09 y guardar en la variable kp
ki       = EEPROM.readDouble(addresski); //leer la direccion 17 y guardar en la variable ki
kd       = EEPROM.readDouble(addresskd); //leer la direccion 25 y guardar en la variable kd
  }
  else{ //inicializar las variables
  // Serial.print(F("inicializando variables: ")); //imprimir en el monitor serie

  // Serial.print(hotendSP); //imprimir en el monitor serie
  // Serial.print(F(" ,")); //imprimir en el monitor serie
  // Serial.print(kp); //imprimir en el monitor serie
  // Serial.print(F(" ,"));
  // Serial.print(ki);
  // Serial.print(F(" ,"));
  // Serial.print(kd);
  // Serial.println();
  EEPROM.writeDouble(addresshotendSP,hotendSP); //inicializar la direccion 01 con el valor 215
  EEPROM.writeDouble(addresskp,kp); //inicializar la direccion 09 con el valor 11.75
  EEPROM.writeDouble(addresski,ki); //inicializar la direccion 17 con el valor 0.57
  EEPROM.writeDouble(addresskd,kd); //inicializar la direccion 25 con el valor 60.97
  EEPROM.writeByte(addressinit,84); //inicializar la direccion 00 con el valor 84
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void AutoPID() {  //funcion AutoPID
 //Serial.println(F("Auto PID"));
 if (aTuneRun == false) { //si no esta en modo AutoPID
 aTuneRun = true;
 startstop_old = startstop; //guardar el estado del hotend actual
 startstop = false; //apagar el hotend 
 //myPID.SetMode(myPID.Control::manual);
 //sTune aTune(&celsius, &PID_value, aTune.Mixed_PID, aTune.directIP, aTune.printALL);
 aTune.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
 aTune.SetEmergencyStop(tempLimit);
 strncpy(autotunepid_char, "RUNING", sizeof(autotunepid_char)); //cambia el dialogo
 menu.update(); //actualiza la pantalla
 Serial.println(F("tuning set parameters"));
 }
 
 analogWrite(PWM_pin, 255 - PID_value); //escribir 255 en el pin PWM para apagar el hotend

  switch (aTune.Run()) { 
    case aTune.sample: // active once per sample during test
      
      //aTune.plotter(celsius, PID_value, hotendSP, 0.5f, 3); // output scale 0.5, plot every 3rd sample
       Serial.println(F("tuning mode"));
      //   Serial.print("setpoint.: ");Serial.print(hotendSP); Serial.print(" ");
         Serial.print(F("input.: "));Serial.print(celsius); Serial.print(F(" "));
         Serial.print(F("output.: "));Serial.print(PID_value); Serial.print(F(" "));
         Serial.print(F("ADC.: "));Serial.print(analogRead(SENSOR_PIN)); Serial.print(F(" "));
         Serial.print(F("kp.: "));Serial.print(aTune.GetKp());Serial.print(F(" "));
         Serial.print(F("ki.: "));Serial.print(aTune.GetKi());Serial.print(F(" "));
         Serial.print(F("kd.: "));Serial.print(aTune.GetKd());Serial.println();
      break;

    case aTune.tunings: // active just once when sTune is done
      aTune.GetAutoTunings(&kp, &ki, &kd); // sketch variables updated by sTune
     // myPID.SetOutputLimits(0, outputSpan * 0.1);
     // myPID.SetSampleTimeUs((outputSpan - 1) * 1000);
      //debounce = 0; // ssr mode
      //PID_value = 255; //setear el valor del PID en 255
      myPID.SetTunings(kp, ki, kd); // update PID with the new tunings
      //settunPID();
         Serial.print(F("setpoint..: "));Serial.print(hotendSP); Serial.print(F(" "));
         Serial.print(F("input..: "));Serial.print(celsius); Serial.print(F(" "));
         Serial.print(F("output..: "));Serial.print(PID_value); Serial.print(F(" "));
         Serial.print(F("ADC..: "));Serial.print(analogRead(SENSOR_PIN)); Serial.print(F(" "));
         Serial.print(F("kp..: "));Serial.print(myPID.GetKp());Serial.print(F(" "));
         Serial.print(F("ki..: "));Serial.print(myPID.GetKi());Serial.print(F(" "));
         Serial.print(F("kd..: "));Serial.print(myPID.GetKd());Serial.println();
      //myPID.SetMode(myPID.Control::automatic); // the PID is turned on
      //myPID.SetDerivativeMode(myPID.dMode::dOnMeas);
      //myPID.SetProportionalMode(myPID.pMode::pOnErrorMeas);
      //myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
      
      break;

    case aTune.runPid: // active once per sample after tunings
     // celsius = (float)thermistor->readCelsius();
      //myPID.Compute();
      //aTune.plotter(celsius, PID_value, hotendSP, 0.5f, 3);
         Serial.print(F("setpoint...: "));Serial.print(hotendSP); Serial.print(F(" "));
         Serial.print(F("input...: "));Serial.print(celsius); Serial.print(F(" "));
         Serial.print(F("output...: "));Serial.print(PID_value); Serial.print(F(" "));
         Serial.print(F("ADC...: "));Serial.print(analogRead(SENSOR_PIN)); Serial.print(F(" "));
         Serial.print(F("kp...: "));Serial.print(myPID.GetKp());Serial.print(F(" "));
         Serial.print(F("ki...: "));Serial.print(myPID.GetKi());Serial.print(F(" "));
         Serial.print(F("kd...: "));Serial.print(myPID.GetKd());Serial.println();
        autotunepid = false;
        enterold = false;
        EVsave();
        startstop = startstop_old;
        aTuneRun = false;
        strncpy(autotunepid_char, "STOPPED", sizeof(autotunepid_char)); //cambia el dialogo
        menu.update(); //actualiza la pantalla
      break;
  }
 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Check all the buttons
	void rightclick() {    //funcion boton derecha o arriba
		// Serial.println(F("RIGHT button pressed"));  //imprimir en el monitor serie
    if (enterold == false){   //si el boton enter no fue presionado
		  menu.next_screen();   //pasa a la siguiente pantalla
    }
    if (enterold == true){  //si el boton enter esta presionado
    // Calls the function identified with one
		// for the focused line.
		menu.call_function(1); //llama a la funcion 1
    }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void leftclick() {    //funcion boton izquierda o abajo
		// Serial.println(F("LEFT button pressed")); //imprimir en el monitor serie
		if (enterold == false){ //si el boton enter no fue presionado
    menu.previous_screen(); //pasa a la pantalla anterior
    }
    if (enterold == true){ //si el boton enter fue presionado
    // Calls the function identified with one
		// for the focused line.
		menu.call_function(2); //llama a la funcion 2
    }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void upclick() {      //funcion boton derecha o arriba
		// Serial.println(F("UP button pressed"));
    if (enterold == true){ //si el boton enter fue presionado
    // Calls the function identified with one
		// for the focused line.
		menu.call_function(1); //llama a la funcion 1
  	}
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void downclick() {    //funcion boton izquierda o abajo
		// Serial.println(F("DOWN button pressed"));
    if (enterold == true){ //si el boton enter fue presionado
    // Calls the function identified with one
		// for the focused line.
		menu.call_function(2); //llama a la funcion 2
    }
	}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void enterclick(){    //funcion enter escape o guardar
		// Serial.println(F("ENTER button pressed"));
    if (menu.get_currentScreen() != &welcome_screen){ //si no esta en la pantalla de bienvenida
      if (enterold == false){ //si el boton enter no fue presionado
        enterold = true; //cambia el estado del boton enter
        } 
        else { //si el boton enter fue presionado
      enterold = false; //cambia el estado del boton enter
      EVsave(); //guarda las variables en la Eprom

      //resetpid();
      }
    //line = menu.get_focusedLine();
    //Serial.print(line);
    // Switches focus to the next line.
      if (menu.get_currentScreen() == &auto_pid_screen){
       if (enterold == true){ //si el boton enter fue presionado
       autotunepid = true; //activar autoPID
       Serial.println(F("AutoPID click"));
        //menu.call_function(3);
        // goto function hotendatotune()
        //hotendautotune();
          }
      }
		  menu.switch_focus(); //cambia el foco a la siguiente linea
      
    }
    if (menu.get_currentScreen() == &welcome_screen){ //Iniciar o detener el hotend, cambiar el dialogo
      if (startstop == false){ //si el hotend esta detenido
          startstop =true; //cambia el estado del hotend
        strncpy(startstopchar, "RUNING", sizeof(startstopchar)); //cambia el dialogo
	    }
        else{ //si el hotend esta corriendo
        startstop =false; //cambia el estado del hotend
         strncpy(startstopchar, "STOPPED", sizeof(startstopchar)); //cambia el dialogo
        }
    }

    menu.update(); //actualiza la pantalla
   }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() { //funcion setup
  Serial.begin(115200); //inicializar el puerto serie
  while (!Serial) delay(10);
  delay(3000);
  myPID.SetMode(myPID.Control::automatic); //activar el PID
  myPID.SetProportionalMode(myPID.pMode::pOnErrorMeas);
  myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
  myPID.SetDerivativeMode(myPID.dMode::dOnMeas);
  
  //snprintf(startstopchar_pid,sizeof(autotunepid_char), "p %lf,i %lf,d %lf", kp, ki, kd); //cambia el dialogo

  // start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeUno 
  // Writes before membase or beyond EEPROMSizeUno will only give errors when _EEPROMEX_DEBUG is set
  EEPROM.setMemPool(memBase, EEPROMSizeUno);  // Set memory pool to use
  
  // Set maximum allowed writes to maxAllowedWrites. 
  // More writes will only give errors when _EEPROMEX_DEBUG is set
  EEPROM.setMaxAllowedWrites(maxAllowedWrites); // Set maximum allowed writes
  
  // Always get the adresses first and in the same order
  // addressByte      = EEPROM.getAddress(sizeof(byte));
  // addressInt       = EEPROM.getAddress(sizeof(int));
  // addressLong      = EEPROM.getAddress(sizeof(long));
  // addressFloat     = EEPROM.getAddress(sizeof(float));
  addressinit        = EEPROM.getAddress(sizeof(byte)); //direccion de memoria para inicializar la eprom
  addresshotendSP    = EEPROM.getAddress(sizeof(double)); //direccion de memoria para SP Hotend
  addresskp          = EEPROM.getAddress(sizeof(double)); //direccion de memoria para KP
  addresski          = EEPROM.getAddress(sizeof(double)); //direccion de memoria para KI
  addresskd          = EEPROM.getAddress(sizeof(double)); //direccion de memoria para KD
  // addressByteArray = EEPROM.getAddress(sizeof(byte)*7);  
  // addressCharArray = EEPROM.getAddress(sizeof(char)*7);  

  EVread(); //lee las variables de la eeprom

  pinMode(PWM_pin,OUTPUT); //pin 3 como salida
  //TCCR0B = TCCR0B & B11111000 | B00000011;    // set timer 0 divisor to    64 for PWM frequency of   976.56 Hz (The DEFAULT)
  //TCCR0B = TCCR0B & B11111000 | B00000100;    // set timer 0 divisor to   256 for PWM frequency of   244.14 Hz
  //TCCR2B = TCCR2B & B11111000 | 0x03;    // pin 3 and 11 PWM frequency of 928.5 Hz
	
  menu.add_screen(welcome_screen); //agregar pantalla de bienvenida
  welcome_line1.set_decimalPlaces(0); //agregar linea de dialogo
  menu.add_screen(auto_pid_screen); //agregar pantalla de auto pid
  menu.add_screen(kd_screen); //agregar pantalla de kd
  menu.add_screen(ki_screen); //agregar pantalla de ki
  menu.add_screen(kp_screen); //agregar pantalla de kp
	menu.add_screen(hotendsp_screen); //agregar pantalla de setpoint
  hotendsp_line.set_decimalPlaces(0); //agregar linea de dialogo
  
  
  
  
  

//libreria one button, definicion de presiones de botones

   left.attachClick(leftclick); //boton izquierda o abajo
   left.attachDuringLongPress(downclick); //boton izquierda o abajo
   right.attachClick(rightclick); //boton derecha o arriba
   right.attachDuringLongPress(upclick); //boton derecha o arriba
  //  up.attachClick(upclick);
  //  up.attachDuringLongPress(upclick);
  //  down.attachClick(downclick);
  //  down.attachDuringLongPress(downclick);
   enter.attachClick(enterclick); //boton enter o escape



  	// Function to attach functions to LiquidLine objects.
	hotendsp_line.attach_function(1, hotendsp_up); //funcion 1 para subir el setpoint
  hotendsp_line.attach_function(2, hotendsp_down); //funcion 2 para bajar el setpoint

  kpsp_line.attach_function(1, kp_up);  //funcion 1 para subir kp
	kpsp_line.attach_function(2, kp_down); //funcion 2 para bajar kp
  kisp_line.attach_function(1, ki_up); //funcion 1 para subir ki
	kisp_line.attach_function(2, ki_down); //funcion 2 para bajar ki
  kdsp_line.attach_function(1, kd_up); //funcion 1 para subir kd
	kdsp_line.attach_function(2, kd_down); //funcion 2 para bajar kd
  //auto_pid.attach_function(3, hotendautotune); //funcion 1 para auto pid


  
  lcd.init(); //inicializar el lcd
  lcd.backlight(); //encender el backlight

 

    //Thermistor* originThermistor = new NTC_Thermistor(
    thermistor = new NTC_Thermistor( //definicion de la resistencia termistor
    SENSOR_PIN,  // pin to read from
    REFERENCE_RESISTANCE, // reference resistor
    NOMINAL_RESISTANCE, // nominal resistance
    NOMINAL_TEMPERATURE, // nominal temperature
    B_VALUE // B value
  );
 // thermistor = new SmoothThermistor(originThermistor, SMOOTHING_FACTOR);
 //turn the PID on
  //myPID.SetMode(AUTOMATIC); //activar el PID
  myPID.SetMode(myPID.Control::automatic); //activar el PID

 menu.update(); //actualizar la pantalla
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop() { //funcion loop
  // keep watching the push button:
  // up.tick();
  // down.tick();
  left.tick(); //funcion de botones
  right.tick(); //funcion de botones
  enter.tick(); //funcion de botones

celsius = thermistor->readCelsius(); //leer la temperatura

if (autotunepid == true){ //si esta en auto pid
 AutoPID(); //ejecutar auto pid
  }

if (startstop == true){ //si el hotend esta encendido
myPID.SetTunings(kp, ki, kd); //setear los valores de kp, ki y kd
//settunPID(); //setear los valores de kp, ki y kd
myPID.Compute(); //calcular el PID

//PID_valueint= (int)PID_value;
analogWrite(PWM_pin, 255 - PID_value); //escribir 255 en el pin PWM para apagar el hotend
}
else{ //si el hotend esta apagado
  if (autotunepid == false){
  PID_value = 0; //setear el valor del PID en 255
  analogWrite(PWM_pin, 255 - PID_value); //escribir 255 en el pin PWM para apagar el hotend
  }
}
	 // Periodically
   if (autotunepid == false){
	 static unsigned long lastMillis = 0; //variable para el tiempo
	 static unsigned int period = 2000; //periodo de lectura de temperatura
     if (millis() - lastMillis > period) { //si el tiempo es mayor al periodo
	 	lastMillis = millis(); //actualizar lastMillis
     Serial.print(F("Temperature: ")); //imprimir en el puerto serie
  //     // lcd.setCursor(0,0);
  //     // lcd.print("temp:");
  //     // lcd.print(celsius);
  //     // lcd.print("  C");
       Serial.print(celsius); //imprimir la temperatura
       Serial.print(F(" C, ")); //imprimir en el puerto serie
       Serial.println(); //imprimir en el puerto serie
       Serial.print(PID_value); //imprimir el valor del PID
       Serial.println(); //imprimir en el puerto serie
       Serial.flush(); //imprimir en el puerto serie
   menu.update(); //actualizar la pantalla
     }  
  }
}