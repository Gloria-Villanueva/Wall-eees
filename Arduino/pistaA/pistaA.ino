//  PISTA A 

//  Librerías: 
//  - Adafruit MPU6050 y Adafruit TCS34725 de Adafruit
//  - LiquidCrystal I2C de Frank de Brabander
//  (Wire y Servo ya vienen con el IDE

//  MODO PRUEBAS (en el Monitor Serial, 115200, "Nueva línea"):

//    e = escanear paredes (frente/der/izq)   
//    c = leer color + valores crudos
//    a = avanzar 1 casilla                   
//    d = girar 90 der    
//    i = girar 90 izq
//    f = ajustar distancia con pared frontal 
//    y = mostrar ángulo del giroscopio


#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_TCS34725.h>

// PINES (AJUSTAR) 
const int IZQ_IN1 = 4, IZQ_IN2 = 7, IZQ_PWM = 5;
const int DER_IN1 = 8, DER_IN2 = 12, DER_PWM = 6;
const int TRIG = A0, ECHO = A1;       // ultrasonico (el que va sobre el servo)
const int PIN_SERVO = 9;
const int BTN_START = 2;              // boton a GND (usa INPUT_PULLUP)
// A4 = SDA, A5 = SCL (MPU6050, TCS34725 y LCD comparten el bus I2C)

// CONSTANTES (AJUSTAR) 
const int   SERVO_FRENTE = 90, SERVO_IZQ = 170, SERVO_DER = 10;  // si salen al reves avisame 
const int   SERVO_ESPERA_MS = 350;       // tiempo para que el servo llegue a su posicion
const float UMBRAL_PARED_CM = 25.0;      // menos que esto significa q hay pared
const float DIST_FRENTE_CENTRO_CM = 10.0;
                                         // (15 cm - distancia del sensor al centro del robot)
const int   VEL_AVANCE = 140;
const unsigned long MS_CASILLA = 1100;   // ms para avanzar 30 cm a VEL_AVANCE
const byte  COLORES_ESPERADOS = 4;     
const unsigned long T_LIMITE_MS = 240000UL; 
const bool  HACER_BONUS_1 = true;        // regresar al inicio por el mismo camino

//  TIPOS 
enum Dir { NORTE, ESTE, SUR, OESTE };
enum ColorPiso { C_NINGUNO, C_CIAN, C_AMARILLO, C_NARANJA, C_ROSA, C_ROJO, C_VERDE };
enum Fase { F_ESPERA, F_EXPLORAR, F_IR_A_ROJO, F_REGRESO, F_FIN };

const char* NOMBRE_COLOR[] = { "---", "CIAN", "AMARILLO", "NARANJA", "ROSA", "ROJO", "VERDE" };
const int8_t DX[4] = { 0, 1, 0, -1 };    // NORTE=+y, ESTE=+x
const int8_t DY[4] = { 1, 0, -1, 0 };
const Dir ORDEN[4] = { NORTE, ESTE, SUR, OESTE };

// OBJETOS Y VARIABLES 
Servo servo;
Adafruit_MPU6050 mpu;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
LiquidCrystal_I2C lcd(0x27, 16, 2);      // si no se ve nada, prueba 0x3F

float yaw = 0, gyroBias = 0;
unsigned long tPrev = 0;
Dir rumbo = NORTE;

const byte G = 9;
byte cx = G / 2, cy = G / 2;
bool visitada[G][G];
byte paredes[G][G];                      // bits: 1<<NORTE, 1<<ESTE, ... = hay pared
Dir pila[40];  byte nPila = 0;           // camino limpio inicio -> posicion actual
Dir rutaRojo[40]; byte nRuta = 0;        // camino limpio inicio -> rojo
bool rojoEncontrado = false;
bool encontrado[4];
int  arucoId = -1;
unsigned long tInicio = 0;
Fase fase = F_ESPERA;
    
float dF, dD, dI;
char textoL0[17] = "";
bool enPrueba = false;

char bufRx[40]; byte nRx = 0;


//  MOTORES Y ULTRASÓNICO
void motores(int izq, int der) {
  digitalWrite(IZQ_IN1, izq > 0); digitalWrite(IZQ_IN2, izq < 0);
  analogWrite(IZQ_PWM, constrain(abs(izq), 0, 255));
  digitalWrite(DER_IN1, der > 0); digitalWrite(DER_IN2, der < 0);
  analogWrite(DER_PWM, constrain(abs(der), 0, 255));
}
void parar() { motores(0, 0); }

float leerCm() {
  digitalWrite(TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long t = pulseIn(ECHO, HIGH, 20000);
  if (t == 0) return 999;
  return t * 0.0343 / 2.0;
}

// Mediana de 3 lecturas (quita lecturas locas)
float leerEstable() {
  float v[3];
  for (int i = 0; i < 3; i++) { v[i] = leerCm(); delay(15); }
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { float t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
}


//  COMUNICACION SERIAL (Pi o Monitor Serial)
void comandoPrueba(char c);

void procesarLinea(char* s) {
  if (strncmp(s, "ARUCO:", 6) == 0) {
    int id = atoi(s + 6);
    if (arucoId < 0 && id >= 0 && id < 50) { arucoId = id; actualizarLCD(); }  // se queda fijo
  } else if (strlen(s) == 1 && fase == F_ESPERA && !enPrueba) {
    comandoPrueba(s[0]);
  }
}

void leerSerialPi() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      bufRx[nRx] = 0;
      char linea[40];
      strcpy(linea, bufRx);
      nRx = 0;                 // liberar el buffer antes de procesar (puede haber llamadas anidadas)
      procesarLinea(linea);
    } else if (c != '\r' && nRx < sizeof(bufRx) - 1) {
      bufRx[nRx++] = c;
    }
  }
}

// delay que sigue escuchando el serial
void esperar(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) leerSerialPi();
}

//  LCD
void actualizarLCD() {
  char l0[17], l1[17], idTxt[4];
  snprintf(l0, 17, "%-16s", textoL0);
  if (arucoId >= 0) snprintf(idTxt, 4, "%d", arucoId); else strcpy(idTxt, "--");
  snprintf(l1, 17, "[%c%c%c%c] ID:%-6s",
           encontrado[0] ? 'C' : '-', encontrado[1] ? 'A' : '-',
           encontrado[2] ? 'N' : '-', encontrado[3] ? 'R' : '-', idTxt);
  lcd.setCursor(0, 0); lcd.print(l0);   // se reescribe sin lcd.clear() para que no parpadee
  lcd.setCursor(0, 1); lcd.print(l1);
}

void mostrarMsg(const char* m) { strncpy(textoL0, m, 16); textoL0[16] = 0; actualizarLCD(); }

void mostrarColor(ColorPiso c) {
  snprintf(textoL0, 17, "Color: %s", NOMBRE_COLOR[c]);
  actualizarLCD();
}

void errorFatal(const char* m) {
  lcd.clear(); lcd.print(m);
  while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
}

//  GIROSCOPIO Y MOVIMIENTO

void calibrarGiro() { 
  float suma = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    suma += g.gyro.z;
    delay(3);
  }
  gyroBias = suma / 500.0;
  yaw = 0;
  tPrev = micros();
}

void actualizarYaw() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  unsigned long ahora = micros();
  float dt = (ahora - tPrev) / 1000000.0;
  tPrev = ahora;
  yaw -= (g.gyro.z - gyroBias) * dt * 180.0 / PI;    // derecha = positivo
  leerSerialPi();
}

float errorAngulo(float objetivo, float actual) {
  float e = objetivo - actual;
  while (e > 180)  e -= 360;
  while (e < -180) e += 360;
  return e;
}

void girarAAngulo(float objetivo) {
  tPrev = micros();
  unsigned long inicio = millis();
  while (millis() - inicio < 3000) {
    actualizarYaw();
    float e = errorAngulo(objetivo, yaw);
    if (abs(e) < 1.5) break;
    int vel = constrain(abs(e) * 3, 70, 160);
    if (e > 0) motores(vel, -vel); else motores(-vel, vel);
  }
  parar();
}

void girarA(Dir destino) {
  girarAAngulo(destino * 90.0);
  rumbo = destino;
}

// vel > 0 avanza, vel < 0 retrocede - se mantiene derecho con el giroscopio
void avanzarRecto(unsigned long ms, int vel) {
  tPrev = micros();
  float ang = rumbo * 90.0;
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    actualizarYaw();
    float e = errorAngulo(ang, yaw);
    int corr = constrain(e * 4, -60, 60);
    motores(vel + corr, vel - corr);
  }
  parar();
}

void avanzarCasilla() { avanzarRecto(MS_CASILLA, VEL_AVANCE); }


//  PAREDES: escaneo con el servo

float medirEn(int ang) {
  servo.write(ang);
  esperar(SERVO_ESPERA_MS);
  return leerEstable();
}

// Llena las paredes de la casilla actual
void escanear() {
  dF = medirEn(SERVO_FRENTE);
  dD = medirEn(SERVO_DER);
  dI = medirEn(SERVO_IZQ);
  servo.write(SERVO_FRENTE);             // regresa mientras el robot se mueve
  byte m = 0;
  if (dF < UMBRAL_PARED_CM) m |= (1 << rumbo);
  if (dD < UMBRAL_PARED_CM) m |= (1 << ((rumbo + 1) % 4));
  if (dI < UMBRAL_PARED_CM) m |= (1 << ((rumbo + 3) % 4));
  paredes[cx][cy] = m;
}

// Si hay pared al frente, se acerca/aleja hasta quedar centrado en la casilla
// Esto corrige el error acumulado del avance por tiempo
void ajustarConFrente() {
  servo.write(SERVO_FRENTE);
  esperar(250);
  for (byte i = 0; i < 6; i++) {
    float d = leerEstable();
    if (d > UMBRAL_PARED_CM) return;     // no hay pared enfrente
    float err = d - DIST_FRENTE_CENTRO_CM;
    if (abs(err) < 1.5) return;
    avanzarRecto(70, err > 0 ? 90 : -90);
    esperar(80);
  }
}


//  SENSOR DE COLOR (TCS34725) - AJUSTAR umbrales con TUS pistas
ColorPiso leerColorUnaVez() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float R = r, G_ = g, B = b;
  if (c < 30) return C_NINGUNO;
  float mnRG = min(R, G_), mxRG = max(R, G_);
  if (B < 0.45 * mnRG && mnRG > 0.75 * mxRG) return C_AMARILLO;
  if (R < 0.6 * min(G_, B) && B > 0.7 * G_) return C_CIAN;
  if (G_ > 1.2 * R && G_ > 1.2 * B) return C_VERDE;
  if (R >= G_ && R >= B) {
    float gr = G_ / R, br = B / R;
    if (gr < 0.30 && br < 0.45) return C_ROJO;
    if (gr < 0.55 && br >= 0.45) return C_ROSA;
    if (gr < 0.80) return C_NARANJA;
  }
  return C_NINGUNO;
}

ColorPiso leerColor() {                  // 3 lecturas, gana la que se repita
  ColorPiso a = leerColorUnaVez(), b = leerColorUnaVez(), c = leerColorUnaVez();
  if (a == b || a == c) return a;
  if (b == c) return b;
  return C_NINGUNO;
}


//  LOGICA DEL MAPA (DFS)
Dir opuesto(Dir d) { return (Dir)((d + 2) % 4); }

void moverA(Dir d) {
  if (d != rumbo) girarA(d);
  avanzarCasilla();
  cx += DX[d]; cy += DY[d];
}

// Se llama al llegar a una casilla NUEVA. Devuelve true si es el rojo.
bool procesarCeldaNueva() {
  visitada[cx][cy] = true;
  ColorPiso c = leerColor();
  if (c >= C_CIAN && c <= C_ROSA) encontrado[c - 1] = true;
  mostrarColor(c);
  if (c == C_ROJO) return true;          // el rojo es un "final": no hace falta escanear
  ajustarConFrente();
  escanear();
  return false;
}

byte coloresEncontrados() {
  byte n = 0;
  for (byte i = 0; i < 4; i++) if (encontrado[i]) n++;
  return n;
}

bool listoParaTerminar() {
  return rojoEncontrado &&
         (coloresEncontrados() >= COLORES_ESPERADOS || millis() - tInicio > T_LIMITE_MS);
}

void retroceder() {                       // deshace el último movimiento de la pila
  Dir d = opuesto(pila[--nPila]);
  moverA(d);
}

void pasoExplorar() {
  if (listoParaTerminar()) { fase = F_IR_A_ROJO; return; }

  // 1) buscar una casilla vecina libre y sin visitar
  for (byte i = 0; i < 4; i++) {
    Dir d = ORDEN[i];
    int nx = cx + DX[d], ny = cy + DY[d];
    if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
    if (visitada[nx][ny]) continue;
    if (paredes[cx][cy] & (1 << d)) continue;

    moverA(d);
    pila[nPila++] = d;
    if (procesarCeldaNueva()) { 
      nRuta = nPila;
      for (byte k = 0; k < nPila; k++) rutaRojo[k] = pila[k];
      mostrarMsg("ROJO!");
      if (listoParaTerminar()) {
        fase = HACER_BONUS_1 ? F_REGRESO : F_FIN;
      } else {
        retroceder();                     // faltan colores
      }
    }
    return;
  }

  // 2) sin opciones: retroceder o terminar
  if (nPila == 0) {
    if (rojoEncontrado) fase = F_IR_A_ROJO;
    else { mostrarMsg("SIN ROJO"); fase = F_FIN; }
    return;
  }
  retroceder();
}

// Vuelve al inicio por la pila y luego repite el camino limpio hasta el rojo
void irARojo() {
  mostrarMsg("YENDO A ROJO");
  while (nPila > 0) retroceder();
  for (byte i = 0; i < nRuta; i++) { moverA(rutaRojo[i]); pila[nPila++] = rutaRojo[i]; }
  mostrarMsg("FINAL: ROJO");
  fase = HACER_BONUS_1 ? F_REGRESO : F_FIN;
}

// Bonus 1: mismas casillas en orden inverso. Sin detectar colores ni escanear.
void regresar() {
  mostrarMsg("REGRESANDO");
  while (nPila > 0) retroceder();
  mostrarMsg("BONUS 1 LISTO");
  fase = F_FIN;
}


//  RONDA Y MODO PRUEBAS
void iniciarRonda() {
  memset(visitada, 0, sizeof(visitada));
  memset(paredes, 0, sizeof(paredes));
  memset(encontrado, 0, sizeof(encontrado));
  nPila = 0; nRuta = 0; rojoEncontrado = false; arucoId = -1;
  cx = G / 2; cy = G / 2;
  yaw = 0; rumbo = NORTE; tPrev = micros();
  servo.write(SERVO_FRENTE);
  Serial.println("MODE:ARUCO");          // le dice a la Pi que busque el marcador
  tInicio = millis();
  procesarCeldaNueva();
  paredes[cx][cy] |= (1 << opuesto(rumbo));   // la espalda del robot es pared
  fase = F_EXPLORAR;
}

void comandoPrueba(char c) {
  enPrueba = true;
  switch (c) {
    case 'e':
      escanear();
      Serial.print("Frente "); Serial.print(dF);
      Serial.print(" | Der "); Serial.print(dD);
      Serial.print(" | Izq "); Serial.println(dI);
      break;
    case 'c': {
      uint16_t r, g, b, cl;
      tcs.getRawData(&r, &g, &b, &cl);
      Serial.print("R="); Serial.print(r); Serial.print(" G="); Serial.print(g);
      Serial.print(" B="); Serial.print(b); Serial.print(" C="); Serial.print(cl);
      ColorPiso col = leerColor();
      Serial.print(" -> "); Serial.println(NOMBRE_COLOR[col]);
      mostrarColor(col);
      break;
    }
    case 'a': avanzarCasilla(); break;
    case 'd': girarA((Dir)((rumbo + 1) % 4)); break;
    case 'i': girarA((Dir)((rumbo + 3) % 4)); break;
    case 'f': ajustarConFrente(); break;
    case 'y': Serial.print("yaw = "); Serial.println(yaw); break;
  }
  enPrueba = false;
}

void loop() {
  leerSerialPi();
  switch (fase) {
    case F_ESPERA:
      parar();
      if (digitalRead(BTN_START) == LOW) { delay(50); iniciarRonda(); }
      break;
    case F_EXPLORAR:   pasoExplorar(); break;
    case F_IR_A_ROJO:  irARojo();      break;
    case F_REGRESO:    regresar();     break;
    case F_FIN:        parar();        break;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(IZQ_IN1, OUTPUT); pinMode(IZQ_IN2, OUTPUT); pinMode(IZQ_PWM, OUTPUT);
  pinMode(DER_IN1, OUTPUT); pinMode(DER_IN2, OUTPUT); pinMode(DER_PWM, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  servo.attach(PIN_SERVO);
  servo.write(SERVO_FRENTE);

  Wire.begin();
  lcd.init(); lcd.backlight();
  mostrarMsg("INICIANDO...");

  if (!mpu.begin()) errorFatal("Falla MPU6050");
  if (!tcs.begin()) errorFatal("Falla TCS34725");

  delay(500);
  mostrarMsg("CALIBRANDO GIRO");
  calibrarGiro();
  mostrarMsg("LISTO: PULSA START");
}
