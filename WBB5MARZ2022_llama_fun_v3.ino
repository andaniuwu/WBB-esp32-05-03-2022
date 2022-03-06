#include <SPI.h>  //Librería para comunicación con SD
#include "SD.h"   //Librería de SD
#include <Wire.h>   // Librería de I2C para RTC y OLED
#include <Adafruit_GFX.h>   //Librería general para pantallas
#include <Adafruit_SSD1306.h> //Librería para OLED
#include "RTClib.h"   //Librería para RTC
File myFile; //Crea instancia para direccionar datos a SD
#define sclk    17      // TX pin SCLK ADS1222 1 y 2, entrada de reloj de serie -D2
#define Dout1   32       // pin Dout ADS1222 1, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D3
#define Dout2   4       // pin Dout ADS1222 2, DRDY/DOUT drdy-> nivel bajo datos a leer listos, salida de datos por flaco positivo de slck, MSB primero -D4
#define mux     16      // RX pin MUX ADS1222 1 y 2, selección de la entrada analógica 0->AINP1+ AINN1- 1->AINP2+ AINN2 -D5
#define Buzz    2       //Pin de conexión al Buzzer (también es el del LED de la ESP32), en la tarjeta es el D2,PIN4 (conecte el led wbb en él)
#define PULSADOR_wii 15
#define ON_wii       1     // Enciende WBB
//Para la uSD se tienen los siguientes pines: CS(D5), SCK(D18), MOSI(D23), MISO(D19)

unsigned long sensor [4] = {0, 0, 0, 0}, sensor_cal[4] = {0, 0, 0, 0};
double calib [4] = {0.0, 0.0, 0.0, 0.0};  //Array para datos de calibración
double factor[4] ={0.0, 0.0, 0.0, 0.0};
double mV[4] ={0.0, 0.0, 0.0, 0.0};
double Val [4] = {0.0, 0.0, 0.0, 0.0};  //Para guardar datos de valores a memoria SD
float sum_sensors=0.0;  //Suma de sensores para umbral
const float umbral=50.0;  //Umbral de suma de sensores
int flagUmbral=0;  //BAndera de que no se pasa el umbral
int peaks=0;  //Contador de picos de umbral
char filenameOA[] = "/12345678_0205_OA.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo AO
char filenameOC[] = "/12345678_0205_OC.txt"; // DíaMesAño_HoraMin(01012022_0205) Archivo OC
float ch0=102.0;  //Variables para corregir picos en sensores
float ch1=887.0; //Cargo valores normales cuando no tiene carga para inicializar
float ch2=300.0;
float ch3=403.0;

//************Declara variables para interrupción************************
volatile int FlagInt=0;;  //Indicador de que hubo unterrupción
hw_timer_t * timer = NULL;  //Creamos una variable para renombrar el timer.
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; //Variable para sincronizar var entre int y loop

//******************Rutina de interrupción***************************
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);  //Bloquea proceso
  FlagInt = 1; //Activa bandera de interrupción
  portEXIT_CRITICAL_ISR(&timerMux);   //Desbloquea proceso
}

//******************configuración del RTC y pantalla oled********************************
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Dom", "Lun", "Mar", "Mie", "Jue", "Vie", "Sab"};
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// **********Inicializa ADS1222*****************************************
void iniADS1222(void) {
  byte x, n;
  digitalWrite(mux, HIGH);
  for (n = 0; n < 3; n++) {         // realizamos 3 lecturas completas
    delay (10);
    for (x = 0; x < 24; x++) {      // 24 pulsos-> 24 bits
      digitalWrite(sclk, HIGH);     // HIGH pulse, algo más de 33us
      delayMicroseconds(30);
      digitalWrite(sclk, LOW);      // LOW pulse, algo más de 33us
      delayMicroseconds(30);
    }
  }
  delay (10);
  while (digitalRead(Dout1) + digitalRead(Dout2)) {}  // esperamos datos preparados
  for (x = 0; x < 26; x++) {                          // auto-calibrado 26 pulsos-> 24bits + 2  On calibración
    digitalWrite(sclk, HIGH);
    delayMicroseconds(5);
    digitalWrite(sclk, LOW);
    delayMicroseconds(5);
  }
#ifdef debug
  Serial.println("Auto-calibrado..");
#endif
  while (digitalRead(Dout1) + digitalRead(Dout2)) {}     // esperamos fin calibracion
#ifdef debug
  Serial.println("Inicializando");
#endif
}

//***********************************************************************************
// ADS1222 leemos (se leen los 20 bits MSB, solo son efectivos los 20 bits de más peso)
void read_ads1222(void) {
    byte x, n = 0, z = 2;
    bool canal = 0;
  digitalWrite(mux, canal);                      // selecionamos el canal 0
  delayMicroseconds(8);  //tenia 8
  do {
    delayMicroseconds(2); // tenía 2
  } while (digitalRead(Dout1) + digitalRead(Dout2));   // esperamos datos listos, Dout1 y Dout2
  for (x = 23; x >= 4; x--) {               // del bit 23 al 3
    digitalWrite(sclk, HIGH);
    digitalRead(Dout1) ? bitWrite(sensor[n], x, 1) : bitWrite(sensor[n], x, 0); // algo más de 16us, leemos 0 y 1
    digitalRead(Dout2) ? bitWrite(sensor[z], x, 1) : bitWrite(sensor[z], x, 0); // algo más de 16us, leemos 2 y 3
    digitalWrite(sclk, LOW);
    delayMicroseconds(30);  //tenía 30
  }
  for (x = 0; x < 5; x++) {               // realizamos 5 pulsos, bits del 3 al 0 + pulso 25 -> forzamos Dout1 y Dout2 a 1
    digitalWrite(sclk, HIGH);
    delayMicroseconds(30);  //tenía 30
    digitalWrite(sclk, LOW);
    delayMicroseconds(30);    //tenía 30
  }
// Lee canal 1
    n = 1;
    z = 3;
    canal = 1;
  digitalWrite(mux, canal);                      // selecionamos el canal 1
  delayMicroseconds(8);  //tenia 8
  do {
    delayMicroseconds(2); // tenía 2
  } while (digitalRead(Dout1) + digitalRead(Dout2));   // esperamos datos listos, Dout1 y Dout2
  for (x = 23; x >= 4; x--) {               // del bit 23 al 3
    digitalWrite(sclk, HIGH);
    digitalRead(Dout1) ? bitWrite(sensor[n], x, 1) : bitWrite(sensor[n], x, 0); // algo más de 16us, leemos 0 y 1
    digitalRead(Dout2) ? bitWrite(sensor[z], x, 1) : bitWrite(sensor[z], x, 0); // algo más de 16us, leemos 2 y 3
    digitalWrite(sclk, LOW);
    delayMicroseconds(30);  //tenía 30
  }
  for (x = 0; x < 5; x++) {               // realizamos 5 pulsos, bits del 3 al 0 + pulso 25 -> forzamos Dout1 y Dout2 a 1
    digitalWrite(sclk, HIGH);
    delayMicroseconds(30);  //tenía 30
    digitalWrite(sclk, LOW);
    delayMicroseconds(30);    //tenía 30
  }
// Revisa si no tiene valores negativos y convierte a mV
    for (int j=0;j<=3;j++){   //Si el bit 23 es 1, entonces es un valor negativo y convierte
    if (bitRead(sensor[j],23)){
      mV[j]=(double)(sensor[j]-16777216.0)/8388.607*3.3;}
      else{mV[j]=(double) sensor[j]/8388.607*3.3;}
    }
    
/*   
    if (abs(mV[0]-ch0)>200){mV[0]=ch0;}  //Si existe un pico, asigna dato anterior
    if (abs(mV[1]-ch1)>200){mV[1]=ch1;}
    if (abs(mV[2]-ch2)>200){mV[2]=ch2;}
    if (abs(mV[3]-ch3)>200){mV[3]=ch3;}
    ch0=mV[0];    //Almacena datos guarda memoria
    ch1=mV[1];
    ch2=mV[2];
    ch3=mV[3]; */
    
    
//      for(int j=0;j<=3;j++){  //Manda los 4 canales a serial
//     if (j<3){Serial.print(mV[j]);  //Manda datos para ver nivel
//         Serial.print(" ");}
//     if (j==3){Serial.println(mV[j]);}
//    }
}

//Función para tono en Buzzer. Cuando se llame requiere el Pin donde saldrá el pulso y
//la frequencia deseada para el pulso
void tone(byte pin, int freq) {
  ledcSetup(0, 2000, 8); // setup beeper
  ledcAttachPin(pin, 0); // attach beeper
  ledcWriteTone(0, freq); // play tone
}


//******************************************************
//***************** SETUP ******************************
void setup(){

  //******************Inicia Comunicación serial*****************
  Serial.begin(115200);
  if (!SD.begin(5)) {   //Inicializa SD
    Serial.println("Fallo conexion serial!");
    while (1);
  }
  //***********************  Inicia RTC  ****************************+
  if (! rtc.begin()) {
    Serial.println("Fallo en RTC");
    while (1);
  }
  //*********************  Inicia OLED  ****************************
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { Serial.println(F("NO PUDO INICIAR OLED"));
    for (;;); // Don't proceed, loop forever
  }

  //**************Inicia Timer para interrupción**************
  timer = timerBegin(0, 80, true); //Inicializa timer 0, preescaler 80, conteo incremental (1us/incremento)
  timerAttachInterrupt(timer, &onTimer, true); //Usando timer0 nombramos rutina "onTimer" que se activará en edge
  timerAlarmWrite(timer, 10000, true); //Interrumpe cada 10mS (100Hz) se interrumpirá, y recarga el contador
  timerAlarmEnable(timer); //Habilitamos el timer para iniciar las interrupciones
  //**********************************************************
//  rtc.adjust(DateTime(__DATE__, __TIME__));  //Solo se usa la primera vez para configurar el RTC con el time de la PC


//*********************  ENCIENDE WBB  ****************************************+
  byte ciclo = 0;
  pinMode(Buzz, OUTPUT);    //Indica que el pin del Buzzer será tipo salida
  pinMode(ON_wii, OUTPUT);
  digitalWrite(ON_wii, HIGH);          // encendemos controladores WII y TFT
  pinMode(mux, OUTPUT);
  digitalWrite(mux, LOW);
  pinMode(sclk, OUTPUT);
  digitalWrite(sclk, LOW);
  //  pinMode(LED_wii, OUTPUT);
  //  digitalWrite(LED_wii, HIGH);
  //  pinMode(PULSADOR_wii, INPUT_PULLUP);
  pinMode(Dout1, INPUT);
  pinMode(Dout2, INPUT);

  //  digitalWrite(LED_wii, HIGH);
  iniADS1222();  //Inicia ADCs
  tone(Buzz,1000);  //Manda señal a Buzzer a 1kHz, para indicar que ya se prendió la WBB
  delay(200);   //Espera 200ms (Este ratardo igual es necesario para estabilizar los ADCs
  tone(Buzz,0); //Apaga buzzer
//************************???CALIBRA???***********************************
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("CALIBRANDO SENSORES");
  display.display();
int cont=0;
int muestras=200;
while (cont<muestras){
if (FlagInt==1){
    read_ads1222();   //Lee datos de sensor 2 y 4 (estos no tenian error en mi WBB)
    for (int j=0;j<=3;j++){  //FOR para leer datos e ir sumando valores
    calib[j]+=mV[j]; //va sumando el valor de cada lectura para los 4 sensores en mV   
    }
    cont++;   //Incrementa contador
   FlagInt = 0; //Borra bandera de interrupción para indicar que ya hizo la tarea
  }  //Cierra ciclo de comparación si hay interrupción
}  //Cierra while de número de muestras
    for (int j=0;j<=3;j++){  //Calcula el promedio de las muestras
    calib[j]=calib[j]/muestras;}


//interrupción
attachInterrupt(digitalPinToInterrupt(PULSADOR_wii), botonwbb,  RISING);

void programa(void);

}

int bot=0;
//??????????????????????/LOOP//??????????????????????????????????
//????????????????????????????????????????????????????????????
void loop(){
//????????????????????????????????????????????????????????????
  display.clearDisplay();//Borra OLED
  display.setCursor(0, 0);
  display.print("presione el boton");  //Mensaje para indicar que el sistema está listo
  delay(1000);
  display.display();  //Envía a pantalla
//espera que se presione el boton del wbb para que inicie calibración y esas cosas
if(bot>0){  
  
//Chequea si el peso es superior a un umbral
programa();
}  //Cierra ciclo de grabación, ahora vuelve a esperar a que se suban a la plataforma

}


void botonwbb(){

bot=bot+1;

}


void programa(void){

      while (flagUmbral==0){   //Mientras no sobrepase el umbral
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Suba a WBB");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
if (FlagInt==1){  //Si hay una interrupción, lee datos
    read_ads1222();   //Lee datos de sensor 2 y 4
    for(int j=0;j<=3;j++){  //Para cada canal
    mV[j]=mV[j]-calib[j]; //Resta el offset de calibración al valor leído 
    if (j<3){Serial.print(mV[j]);  //Manda datos para ver nivel
       Serial.print(" ");}
    if (j==3){Serial.println(mV[j]);}
    }  //Cierra ciclo de eliminación de offset
    sum_sensors=abs(mV[0])+abs(mV[1])+abs(mV[2])+abs(mV[3]); //suma valores sensores
    if(sum_sensors>umbral){
      peaks++; //Incrementa dcontador de picos
      if (peaks>100){  //Si ya pasó 1 segundo con umbra arriba
      flagUmbral=1;  //Activa bandera de que ya pasó el umbral
      peaks=0;    //Reinicia contador de picos
        }  
     } 
   FlagInt = 0; //Borra bandera de interrupción para indicar que ya hizo la tarea
}
}

//*******************Si hay alguien sobre la plataforma, inicia grabación **********************************
if(flagUmbral==1){   //Si ya detectó que alguien subió a la plataforma
  tone(Buzz,5000); //enciende Buzzer
  DateTime now = rtc.now();   //Obtiene datos para nombrar archivo
  sprintf (filenameOA, "/%02d%02d%04d_%02d%02d_OA.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  sprintf (filenameOC, "/%02d%02d%04d_%02d%02d_OC.txt", now.day(), now.month(), now.year(), now.hour(), now.minute());
  while (myFile==0){
  myFile = SD.open(filenameOA, FILE_WRITE); //Crea archivo para ojos cerrados
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Creando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print("archivo");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
    }

  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Inicia");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print("grabación");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000);  //Espera dos segundos antes de grabar para estabilizar un poco
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Grabando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print("Etapa OA");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
    tone(Buzz,0); //Apaga Buzzer
int contador=0;    //Iniciazaliza contador de muestras 
while (contador<3000){
if (FlagInt==1){  //Checa si hubo interrupción 
    read_ads1222();   //Lee datos de sensores
    for(int j=0;j<=3;j++){  //Para cada canal resta offset
      mV[j]=mV[j]-calib[j];} 
    if (myFile) { //Guarda datos en microSD
      myFile.print(mV[0]);
      myFile.print(" ");
      myFile.print(mV[1]);
      myFile.print(" ");
      myFile.print(mV[2]);
      myFile.print(" ");
      myFile.println(mV[3]);}
      else {Serial.println("Error de archivo");}  //Manda mensaje de error si no hay archivo
    for (int j=0;j<=3;j++){  //Manda datos a serial para control
      if (j<3){Serial.print(mV[j]);
               Serial.print(" ");}
      if (j==3){Serial.println(mV[j]);}
      }
      FlagInt=0;  //Borra bandera de interrupción para esperar una nueva
      contador++; //Incrementa contador de muestras 
     } 
}
  myFile.close();  //Cierra archivo
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Cierre");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print(" los ojos");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000); //Espera 2 segundo a que cierre ojos y estabilice señal
//*************************Inicia grabación ojos cerrados**********************************
  tone(Buzz,5000);  //Enciende buzzer
  myFile = SD.open(filenameOC, FILE_WRITE); //Crea archivo para ojos cerrados
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("Grabando");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print("Etapa OC");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000); // Para buzzer y estabilizar señal
  tone(Buzz,0); //enciende Buzzer
contador=0;    //Iniciazaliza contador de muestras 
while (contador<3000){
if (FlagInt==1){  //Checa si hubo interrupción 
    read_ads1222();   //Lee datos de sensores
    for(int j=0;j<=3;j++){  //Para cada canal resta offset
      mV[j]=mV[j]-calib[j];} 
    if (myFile) { //Guarda datos en microSD
      myFile.print(mV[0]);
      myFile.print(" ");
      myFile.print(mV[1]);
      myFile.print(" ");
      myFile.print(mV[2]);
      myFile.print(" ");
      myFile.println(mV[3]);}
      else {Serial.println("Error de archivo");}  //Manda mensaje de error si no hay archivo
    for (int j=0;j<=3;j++){  //Manda datos a serial para control
      if (j<3){Serial.print(mV[j]);
               Serial.print(" ");}
      if (j==3){Serial.println(mV[j]);}
      }
      FlagInt=0;  //Borra bandera de interrupción para esperar una nueva
      contador++; //Incrementa contador de muestras 
     } 
}
  myFile.close();  //Cierra archivo
  flagUmbral=0;   //Borra bandera de uso, para detener ciclos
  display.clearDisplay();  //Borra OLED
  display.setCursor(0, 0);
  display.print("PRUEBA");  //Mensaje para indicar que el sistema está listo
  display.setCursor(0, 40);
  display.print("FINALIZADA");  //Mensaje para indicar que el sistema está listo
  display.display();  //Envía a pantalla
  delay(2000); //Espera 2 segundo


  bot=0; //reinicia el contador del boton
}
  
}
