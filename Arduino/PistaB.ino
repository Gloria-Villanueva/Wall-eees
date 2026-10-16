//  PISTA B: NIVELES  (Candidates 2026)
//
//  Librerías: Adafruit TCS34725, Adafruit NeoPixel
//  (Wire, Servo y EEPROM ya vienen con el IDE)
//
//  SUPUESTOS DE HARDWARE (ajusta si tu robot es distinto):
//   - Servo de garra en pin 10 (cerrada = recogida, para caber en 30x30 cm).
//   - Sensor de color mirando al piso, ADELANTE de las llantas
//     (SENSOR_ADELANTE_CM del centro del robot, entre 5 y 12 cm).
//   - Ultrasónico sobre el servo, igual que en la Pista A.
//   - Botón extra BTN_SEC (pin 11, a GND) para elegir sección a mano.
//
//  COMO SE RETOMA TRAS UN LACK OF PROGRESS:
//   Cada vez que se alcanza un checkpoint se guarda la sección en EEPROM.
//   El capitán reinicia el robot SIEMPRE igual (apagar/encender), lo coloca en
//   el checkpoint y pulsa START: el robot continúa en la sección guardada.
//   - Para empezar una ronda desde cero: encender el robot MANTENIENDO START.
//   - Para el 4to lack of progress (saltar de sección): pulsa BTN_SEC antes
//     de START hasta que el LCD muestre la sección deseada.
//   - Si se reinicia en Sección 2 o 3, la pelota queda al frente (imagen 3.1)
//     y el robot la toma antes de empezar.
//
//  MODO PRUEBAS (Monitor Serial 115200, "Nueva línea", solo en espera):
//    e = escanear paredes   c = leer color   a = avanzar 1 casilla
//    d = girar der          i = girar izq    f = ajustar con pared frontal
//    y = ver pulsos encoder g = abrir/cerrar garra

#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include <stdlib.h>

// ===== PINES (AJUSTAR) =====
const int IZQ_IN1 = 2, IZQ_IN2 = 4, IZQ_PWM = 3;   
const int DER_IN1 = 6, DER_IN2 = 7, DER_PWM = 5;  
const int TRIG = 8, ECHO = 9;                      
const int PIN_SERVO = 10;                  
const int PIN_GARRA = 11;                        
const int BTN_START = 13;
const uint8_t LED_PIN = 12;
const uint8_t NUM_LEDS = 8;
Adafruit_NeoPixel tira(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);                          
const int BTN_SEC   = A1; // A3 queda reservado para el encoder derecho                          

// ===== CONSTANTES (AJUSTAR CON TU ROBOT Y PISTA) =====
const int   SERVO_FRENTE = 90, SERVO_IZQ = 170, SERVO_DER = 10;
const int   SERVO_ESPERA_MS = 350;
const int   GARRA_ABIERTA = 40, GARRA_CERRADA = 120;
const int   GARRA_ESPERA_MS = 400;

const float UMBRAL_PARED_CM = 20.0;      // < esto = pared (antes 25: se bajó para no confundir la pelota)
const float DIST_FRENTE_CENTRO_CM = 10.0;
const float PELOTA_MIN_CM = 22.0;        // la pelota en el centro de la casilla vecina se lee ~25 cm
const float PELOTA_MAX_CM = 34.0;        // (las paredes se leen a ~10, 40, 70 cm: no se cruzan)
const float DIST_AGARRE_CM = 6.0;        // distancia a la que la pelota queda dentro de la garra

const int   VEL_AVANCE = 140;
const int   VEL_LENTO  = 100;
// El avance por casilla se controla con PULSOS_CASILLA (encoders).
const float MS_POR_CM_LENTO = 55.0;      // MIDE: ms por cm a VEL_LENTO
const unsigned long MS_APROX_MAX = (unsigned long)(22.0 * MS_POR_CM_LENTO);

const float SENSOR_ADELANTE_CM = 8.0;    // sensor de color adelante del centro del robot
const unsigned long T_SEC1_MAX_MS = 150000UL; // si pasa este tiempo y ya se vio el rojo, se va sin pelota

// Sección 2
const float SEC2_FILA_CM = 15.0;         // el checkpoint está centrado entre las 2 filas: se desplaza a una
const float PASO_LAT_CM = 4.0;           // paso del barrido lateral para buscar el hueco de 30 cm
const float SEC2_LAT_MAX_CM = 24.0;      // límite lateral del barrido (desde el centro)
const float SEC2_RECENTRAR_CM = 95.0;    // tras estos cm dentro de la cuadrícula se re-centra para el checkpoint
const float CRUCE_LINEA_CM = 6.0;        // avance para pasar la línea una vez centrado en el hueco
const int   UMBRAL_BLANCO_C = 1500;      // canal "clear" mínimo para considerar blanco (CALIBRAR)

const bool  ESPALDA_ES_PARED = true;     // Sección 1: al iniciar hay pared a la espalda
const byte  MAX_PASOS = 40;

// ===== TIPOS =====
enum Dir { NORTE, ESTE, SUR, OESTE };
enum ColorPiso { C_NINGUNO, C_CIAN, C_AMARILLO, C_NARANJA, C_ROSA, C_ROJO, C_VERDE, C_BLANCO };
enum Fase { F_ESPERA, F_CORRER, F_FIN };

const char* NOMBRE_COLOR[] = { "---", "CIAN", "AMARILLO", "NARANJA", "ROSA", "ROJO", "VERDE", "BLANCO" };
const int8_t DX[4] = { 0, 1, 0, -1 };    // NORTE=+y, ESTE=+x
const int8_t DY[4] = { 1, 0, -1, 0 };
const Dir ORDEN[4] = { NORTE, ESTE, SUR, OESTE };

// ===== OBJETOS Y VARIABLES =====
Servo servo, garra;
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float yaw = 0; // variable heredada; no se usa giroscopio
Dir rumbo = NORTE;

const byte G = 9;
byte cx = G / 2, cy = G / 2;
bool visitada[G][G];
byte paredes[G][G];
Dir pila[MAX_PASOS];     byte nPila = 0;
Dir rutaRojo[MAX_PASOS]; byte nRuta = 0;
bool rojoEncontrado = false, enRojo = false;
bool tienePelota = false;
byte seccion = 1;
float latCm = 0;                         // posición lateral en sección 2 (+ = ESTE/derecha)
unsigned long tInicioSec = 0;
Fase fase = F_ESPERA;

float dF, dD, dI;
char textoL0[17] = "";

//  MOTORES Y ULTRASÓNICO  (copiado de Pista A)
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

float leerEstable() {                    // mediana de 3 lecturas
  float v[3];
  for (int i = 0; i < 3; i++) { v[i] = leerCm(); delay(15); }
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { float t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
}

void esperar(unsigned long ms) { delay(ms); }   // (sin Raspberry Pi en la Pista B)

//  GARRA
void abrirGarra()  { garra.write(GARRA_ABIERTA);  esperar(GARRA_ESPERA_MS); }
void cerrarGarra() { garra.write(GARRA_CERRADA);  esperar(GARRA_ESPERA_MS); }


// MENSAJES: Serial + NeoPixel, sin LCD.
void actualizarLCD() { /* reemplazo compatible: ya no hay LCD */ }
void mostrarMsg(const char* m) {
  strncpy(textoL0, m, 16); textoL0[16] = 0;
  Serial.print(F("ESTADO: ")); Serial.println(m);
  uint32_t color = tira.Color(0, 0, 255);
  if (strstr(m, "ERROR") || strstr(m, "NO VI") || strstr(m, "SIN")) color = tira.Color(255, 0, 0);
  else if (strstr(m, "FIN") || strstr(m, "CHECKPOINT")) color = tira.Color(0, 255, 0);
  else if (strstr(m, "SECCION") || strstr(m, "PELOTA") || strstr(m, "LINEA")) color = tira.Color(255, 120, 0);
  for (uint8_t i = 2; i < NUM_LEDS; i++) tira.setPixelColor(i, color);
  tira.show();
}
void mostrarColor(ColorPiso c) {
  snprintf(textoL0, 17, "Color: %s", NOMBRE_COLOR[c]);
  Serial.print(F("COLOR: ")); Serial.println(NOMBRE_COLOR[c]);
  uint32_t rgb = 0;
  switch (c) {
    case C_CIAN: rgb = tira.Color(0,255,255); break;
    case C_AMARILLO: rgb = tira.Color(255,255,0); break;
    case C_NARANJA: rgb = tira.Color(255,80,0); break;
    case C_ROSA: rgb = tira.Color(255,0,100); break;
    case C_ROJO: rgb = tira.Color(255,0,0); break;
    case C_VERDE: rgb = tira.Color(0,255,0); break;
    case C_BLANCO: rgb = tira.Color(255,255,255); break;
    default: rgb = 0; break;
  }
  tira.setPixelColor(0, rgb); tira.setPixelColor(1, rgb); tira.show();
}
void errorFatal(const char* m) {
  Serial.print(F("ERROR FATAL: ")); Serial.println(m);
  tira.fill(tira.Color(255,0,0)); tira.show();
  while (1) { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(150); }
}

// ENCODERS: señal izquierda A2 (PC2) y derecha A3 (PC3).
// En Arduino Uno se usa Pin Change Interrupt para A2/A3.
const uint8_t ENC_IZQ = A2;
const uint8_t ENC_DER = A3;
volatile unsigned long pulsosIzq = 0;
volatile unsigned long pulsosDer = 0;
volatile uint8_t estadoEncPrevio = 0;

ISR(PCINT1_vect) {
  uint8_t estado = PINC;
  if ((estado & _BV(PC2)) && !(estadoEncPrevio & _BV(PC2))) pulsosIzq++;
  if ((estado & _BV(PC3)) && !(estadoEncPrevio & _BV(PC3))) pulsosDer++;
  estadoEncPrevio = estado;
}

unsigned long contarIzq() { noInterrupts(); unsigned long n = pulsosIzq; interrupts(); return n; }
unsigned long contarDer() { noInterrupts(); unsigned long n = pulsosDer; interrupts(); return n; }
void reiniciarConteo() { noInterrupts(); pulsosIzq = 0; pulsosDer = 0; interrupts(); }

// CALIBRAR: valores iniciales; medir con las ruedas elevadas y luego en la pista.
const unsigned long PULSOS_CASILLA = 100; // pulsos de flanco ascendente por casilla de 30 cm
const unsigned long PULSOS_GIRO_90 = 45;  // pulsos por rueda para un giro de 90 grados

bool avanzarPulsos(unsigned long objetivo, int vel) {
  unsigned long iniI = contarIzq(), iniD = contarDer();
  unsigned long inicio = millis();
  const unsigned long timeout = 6000;
  while (true) {
    unsigned long izq = contarIzq() - iniI;
    unsigned long der = contarDer() - iniD;
    unsigned long promedio = (izq + der) / 2;
    if (promedio >= objetivo) break;
    if (millis() - inicio > timeout) { parar(); Serial.println(F("ERROR: timeout encoder avance")); return false; }
    int correccion = constrain((long)izq - (long)der, -35, 35);
    motores(vel - correccion, vel + correccion);
  }
  parar();
  return true;
}

bool girar90Encoder(bool derecha) {
  unsigned long iniI = contarIzq(), iniD = contarDer();
  unsigned long inicio = millis();
  const unsigned long timeout = 4000;
  while (true) {
    unsigned long izq = contarIzq() - iniI;
    unsigned long der = contarDer() - iniD;
    if ((izq + der) / 2 >= PULSOS_GIRO_90) break;
    if (millis() - inicio > timeout) { parar(); Serial.println(F("ERROR: timeout encoder giro")); return false; }
    if (derecha) motores(110, -110); else motores(-110, 110);
  }
  parar();
  delay(100);
  return true;
}

void girarA(Dir destino) {
  int delta = ((int)destino - (int)rumbo + 4) % 4;
  if (delta == 1) girar90Encoder(true);
  else if (delta == 3) girar90Encoder(false);
  else if (delta == 2) { girar90Encoder(true); girar90Encoder(true); }
  rumbo = destino;
}

// Movimiento corto temporizado para correcciones de centrado; avance de casilla usa encoder.
void actualizarYaw() { /* sin MPU6050 */ }

void avanzarRecto(unsigned long ms, int vel) {
  unsigned long iniI = contarIzq(), iniD = contarDer();
  unsigned long inicio = millis();
  while (millis() - inicio < ms) {
    long dif = (long)(contarIzq() - iniI) - (long)(contarDer() - iniD);
    int corr = constrain((int)dif, -30, 30);
    motores(vel - corr, vel + corr);
  }
  parar();
}

void avanzarCasilla() { avanzarPulsos(PULSOS_CASILLA, VEL_AVANCE); }


// avanzar/retroceder una distancia en cm a velocidad lenta
void avanzarCm(float cm) {
  if (cm >= 0) avanzarRecto((unsigned long)(cm * MS_POR_CM_LENTO), VEL_LENTO);
  else         avanzarRecto((unsigned long)(-cm * MS_POR_CM_LENTO), -VEL_LENTO);
}

//  PAREDES  (copiado de Pista A)
float medirEn(int ang) {
  servo.write(ang);
  esperar(SERVO_ESPERA_MS);
  return leerEstable();
}

void escanear() {
  dF = medirEn(SERVO_FRENTE);
  dD = medirEn(SERVO_DER);
  dI = medirEn(SERVO_IZQ);
  servo.write(SERVO_FRENTE);
  byte m = 0;
  if (dF < UMBRAL_PARED_CM) m |= (1 << rumbo);
  if (dD < UMBRAL_PARED_CM) m |= (1 << ((rumbo + 1) % 4));
  if (dI < UMBRAL_PARED_CM) m |= (1 << ((rumbo + 3) % 4));
  paredes[cx][cy] = m;
}

void ajustarConFrente() {
  servo.write(SERVO_FRENTE);
  esperar(250);
  for (byte i = 0; i < 6; i++) {
    float d = leerEstable();
    if (d > UMBRAL_PARED_CM) return;
    float err = d - DIST_FRENTE_CENTRO_CM;
    if (abs(err) < 1.5) return;
    avanzarRecto(70, err > 0 ? 90 : -90);
    esperar(80);
  }
}

//  SENSOR DE COLOR  (Pista A + BLANCO) - CALIBRAR con tu pista
ColorPiso leerColorUnaVez() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  float R = r, G_ = g, B = b;
  if (c < 30) return C_NINGUNO;
  float mn = min(R, min(G_, B)), mx = max(R, max(G_, B));
  if (c >= UMBRAL_BLANCO_C && mn > 0.7 * mx) return C_BLANCO;   //canales parejos y brillantes
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

bool hayBlanco() { return leerColor() == C_BLANCO; }

// avanza lento y recto leyendo el piso hasta ver 'objetivo' o recorrer maxCm. 'rec' devuelve los cm recorridos. Devuelve 'objetivo' si lo vio, C_NINGUNO si no.
ColorPiso avanzarHasta(ColorPiso objetivo, float maxCm, float &rec) {
  unsigned long iniI = contarIzq(), iniD = contarDer();
  unsigned long ini = millis();
  unsigned long maxMs = (unsigned long)(maxCm * MS_POR_CM_LENTO);
  ColorPiso res = C_NINGUNO;
  while (millis() - ini < maxMs) {
    long dif = (long)(contarIzq() - iniI) - (long)(contarDer() - iniD);
    int corr = constrain((int)dif, -25, 25);
    motores(VEL_LENTO - corr, VEL_LENTO + corr);
    if (leerColorUnaVez() == objetivo) {
      parar(); esperar(30);
      if (leerColorUnaVez() == objetivo) { res = objetivo; break; }
    }
  }
  parar();
  rec = (millis() - ini) / MS_POR_CM_LENTO;
  return res;
}

//  PELOTA
Dir opuesto(Dir d) { return (Dir)((d + 2) % 4); }

// Con la garra abierta avanza despacio hasta que la pelota queda a DIST_AGARRE_CM
// y cierra la garra. Devuelve los ms que avanzó (para poder retroceder lo mismo).
unsigned long tomarPelota(unsigned long maxMs) {
  servo.write(SERVO_FRENTE);
  esperar(SERVO_ESPERA_MS);
  abrirGarra();
  unsigned long iniI = contarIzq(), iniD = contarDer();
  unsigned long ini = millis();
  while (millis() - ini < maxMs) {
    if (leerCm() <= DIST_AGARRE_CM) break;
    long dif = (long)(contarIzq() - iniI) - (long)(contarDer() - iniD);
    int corr = constrain((int)dif, -25, 25);
    motores(VEL_LENTO - corr, VEL_LENTO + corr);
  }
  parar();
  unsigned long avanzado = millis() - ini;
  cerrarGarra();
  tienePelota = true;
  Serial.print(F("Pelota tomada; ms de aproximacion=")); Serial.println(avanzado);
  return avanzado;
}

// Reinicio en Sección 2 o 3: la pelota está justo enfrente (imagen 3.1)
void recogerPelotaFrente() {
  mostrarMsg("TOMANDO PELOTA");
  tomarPelota((unsigned long)(12.0 * MS_POR_CM_LENTO));
}

// Si el escaneo ve algo a la distancia de la pelota (centro de la casilla vecina
// sin visitar), devuelve esa dirección; si no, -1.
int dirPelota() {
  float v[3] = { dF, dD, dI };
  Dir ds[3] = { rumbo, (Dir)((rumbo + 1) % 4), (Dir)((rumbo + 3) % 4) };
  for (byte i = 0; i < 3; i++) {
    if (v[i] >= PELOTA_MIN_CM && v[i] <= PELOTA_MAX_CM) {
      int nx = cx + DX[ds[i]], ny = cy + DY[ds[i]];
      if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
      if (visitada[nx][ny]) continue;
      return ds[i];
    }
  }
  return -1;
}

void capturarPelota(Dir d) {
  mostrarMsg("PELOTA!");
  int nx = cx + DX[d], ny = cy + DY[d];
  if (d != rumbo) girarA(d);
  unsigned long avanzado = tomarPelota(MS_APROX_MAX);
  avanzarRecto(avanzado + 100, -VEL_LENTO);        // regresar para sacar la pelota de la unidad
  visitada[nx][ny] = true;                         // casilla de la pelota: no volver a entrar
  paredes[cx][cy] |= (1 << d);
}

//  SECCIÓN 1: explorar (DFS como en Pista A), encontrar la pelota y llegar al checkpoint
void moverA(Dir d) {
  if (d != rumbo) girarA(d);
  avanzarCasilla();
  cx += DX[d]; cy += DY[d];
}

void moverRecentrando(Dir d) {
  moverA(d);
  if (paredes[cx][cy] & (1 << rumbo)) ajustarConFrente();
}

void retroceder() {
  Dir d = opuesto(pila[--nPila]);
  moverRecentrando(d);
}

// Se llama al llegar a una casilla nueva. Devuelve true si es el checkpoint (rojo).
bool procesarCeldaNueva() {
  visitada[cx][cy] = true;
  ColorPiso c = leerColor();
  mostrarColor(c);
  if (c == C_ROJO) return true;
  ajustarConFrente();
  escanear();
  if (!tienePelota) {
    int dp = dirPelota();
    if (dp >= 0) capturarPelota((Dir)dp);
  }
  return false;
}

void irARojo() {                          // igual que Pista A: solo hasta el punto de bifurcación
  mostrarMsg("YENDO A ROJO");
  byte comun = 0;
  while (comun < nPila && comun < nRuta && pila[comun] == rutaRojo[comun]) comun++;
  while (nPila > comun) retroceder();
  for (byte i = comun; i < nRuta; i++) {
    moverRecentrando(rutaRojo[i]);
    pila[nPila++] = rutaRojo[i];
  }
  enRojo = true;
}

// Un paso de exploración. Devuelve false cuando la sección 1 terminó.
bool pasoExplorar1() {
  bool tiempoAgotado = (millis() - tInicioSec) > T_SEC1_MAX_MS;
  if (rojoEncontrado && (tienePelota || tiempoAgotado)) { irARojo(); return false; }

  if (nPila < MAX_PASOS) {
    for (byte i = 0; i < 4; i++) {
      Dir d = ORDEN[i];
      int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
      if (visitada[nx][ny]) continue;
      if (paredes[cx][cy] & (1 << d)) continue;

      moverA(d);
      pila[nPila++] = d;
      if (procesarCeldaNueva()) {         // llegó al checkpoint
        rojoEncontrado = true;
        nRuta = nPila;
        for (byte k = 0; k < nPila; k++) rutaRojo[k] = pila[k];
        mostrarMsg("CHECKPOINT 1");
        if (tienePelota || tiempoAgotado) { enRojo = true; return false; }
        retroceder();                     // todavía falta la pelota: sigue explorando
      }
      return true;
    }
  }
  if (nPila == 0) {                       // se acabó el mapa
    if (rojoEncontrado) irARojo();
    return false;
  }
  retroceder();
  return true;
}

// Ya en el checkpoint: si al frente hay pared, gira hacia el lado libre y
// declara ese rumbo como "adelante" (NORTE) para las secciones 2 y 3.
void orientarSalida() {
  escanear();
  if (paredes[cx][cy] & (1 << rumbo)) {
    Dir der = (Dir)((rumbo + 1) % 4), izq = (Dir)((rumbo + 3) % 4);
    if (!(paredes[cx][cy] & (1 << der))) girarA(der);
    else if (!(paredes[cx][cy] & (1 << izq))) girarA(izq);
  }
  yaw = 0; rumbo = NORTE;
}

bool seccion1() {
  mostrarMsg("SECCION 1");
  memset(visitada, 0, sizeof(visitada));
  memset(paredes, 0, sizeof(paredes));
  nPila = 0; nRuta = 0; rojoEncontrado = false; enRojo = false; tienePelota = false;
  cx = G / 2; cy = G / 2; rumbo = NORTE; yaw = 0;
  tInicioSec = millis();
  servo.write(SERVO_FRENTE);

  procesarCeldaNueva();
  if (ESPALDA_ES_PARED) paredes[cx][cy] |= (1 << opuesto(rumbo));

  while (pasoExplorar1()) {}
  if (!enRojo) { mostrarMsg("SIN CHECKPOINT"); return false; }
  orientarSalida();
  return true;
}


//  SECCIÓN 2: cuadrícula verde 2x4 con líneas blancas (hueco de 30 cm)
//  Marco de referencia: NORTE = adelante, ESTE = derecha, latCm + = derecha.
void desplazarLateral(float cm) {         // mueve de lado (gira, avanza, regresa a NORTE)
  if (fabs(cm) < 0.5) return;
  girarA(cm > 0 ? ESTE : OESTE);
  avanzarCm(fabs(cm));
  girarA(NORTE);
  latCm += cm;
}

// El sensor está sobre una línea blanca. Barre de lado hasta encontrar el hueco
// (lecturas no blancas), mide sus dos bordes y se centra en medio.
bool resolverLinea() {
  int s = (latCm > 0) ? -1 : 1;           // primero hacia el centro de la cuadrícula
  for (byte intento = 0; intento < 2; intento++) {
    bool hueco = false;
    float p1 = 0, p2 = 0;
    while (fabs(latCm + s * PASO_LAT_CM) <= SEC2_LAT_MAX_CM) {
      desplazarLateral(s * PASO_LAT_CM);
      if (!hayBlanco()) { hueco = true; p1 = p2 = latCm; break; }
    }
    if (hueco) {
      while (fabs(latCm + s * PASO_LAT_CM) <= SEC2_LAT_MAX_CM) {
        desplazarLateral(s * PASO_LAT_CM);
        if (hayBlanco()) break;
        p2 = latCm;
      }
      desplazarLateral((p1 + p2) / 2.0 - latCm);   // centro del hueco
      return true;
    }
    s = -s;                               // no hubo hueco de ese lado: probar el otro
  }
  return false;
}

bool seccion2() {
  mostrarMsg("SECCION 2");
  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_24MS);   // lectura más rápida al avanzar
  latCm = 0;
  float rec;

  desplazarLateral(SEC2_FILA_CM);         // el checkpoint queda entre las dos filas: elegir una
  avanzarHasta(C_VERDE, 40, rec);         // salta el separador blanco y entra a la cuadrícula

  float x = 0;                            // cm recorridos dentro de la cuadrícula
  while (x < SEC2_RECENTRAR_CM) {
    ColorPiso r = avanzarHasta(C_BLANCO, SEC2_RECENTRAR_CM - x + 3, rec);
    x += rec;
    if (r != C_BLANCO) break;
    mostrarMsg("LINEA BLANCA");
    resolverLinea();
    avanzarCm(CRUCE_LINEA_CM);            // cruzar por el hueco
    x += CRUCE_LINEA_CM;
  }

  desplazarLateral(-latCm);               // centrar para entrar al checkpoint (está centrado)
  ColorPiso r = avanzarHasta(C_ROJO, 60, rec);
  if (r == C_ROJO) avanzarCm(SENSOR_ADELANTE_CM + 4);   // que el centro del robot quede dentro (>50%)
  tcs.setIntegrationTime(TCS34725_INTEGRATIONTIME_50MS);
  mostrarMsg(r == C_ROJO ? "CHECKPOINT 2" : "NO VI CHECKPOINT");
  return r == C_ROJO;
}


//  SECCIÓN 3: seguir el camino marcado por colores
//  Direcciones respecto a la PISTA, con "arriba" = hacia adelante desde el checkpoint.
//  CIAN = derecha, AMARILLO = izquierda, NARANJA = arriba, ROSA = abajo, VERDE = fin.
bool seccion3() {
  mostrarMsg("SECCION 3");
  moverA(NORTE);                          // primera casilla: adelante desde el checkpoint
  for (byte paso = 0; paso < 25; paso++) {
    ColorPiso c = C_NINGUNO;
    for (byte k = 0; k < 3 && (c == C_NINGUNO || c == C_BLANCO); k++) c = leerColor();
    mostrarColor(c);
    Dir d;
    switch (c) {
      case C_VERDE:    return true;       // casilla final
      case C_CIAN:     d = ESTE;  break;
      case C_AMARILLO: d = OESTE; break;
      case C_NARANJA:  d = NORTE; break;
      case C_ROSA:     d = SUR;   break;
      default: mostrarMsg("SIN COLOR"); return false;
    }
    moverA(d);
  }
  return false;
}

//  PRUEBAS, LOOP Y SETUP
void guardarSeccion(byte s) { seccion = s; EEPROM.update(0, s); }

void comandoPrueba(char c) {
  static bool abierta = false;
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
    case 'y': Serial.print("Pulsos I/D: "); Serial.print(contarIzq()); Serial.print(" / "); Serial.println(contarDer()); break;
    case 'g': abierta = !abierta; if (abierta) abrirGarra(); else cerrarGarra(); break;
  }
}

void loop() {
  switch (fase) {
    case F_ESPERA:
      parar();
      if (Serial.available()) {
        char c = Serial.read();
        if (c != '\n' && c != '\r') comandoPrueba(c);
      }
      if (digitalRead(BTN_SEC) == LOW) {          // elegir sección a mano
        delay(50);
        guardarSeccion(seccion % 3 + 1);
        actualizarLCD();
        while (digitalRead(BTN_SEC) == LOW) {}
        delay(50);
      }
      if (digitalRead(BTN_START) == LOW) {
        delay(50);
        mostrarMsg("SUELTA ROBOT");
        delay(1500);
        mostrarMsg("ENCODERS LISTOS");
        rumbo = NORTE; yaw = 0;
        if (seccion >= 2) recogerPelotaFrente();  // reinicio: la pelota está enfrente
        fase = F_CORRER;
      }
      break;

    case F_CORRER:
      if (seccion == 1) {
        if (seccion1()) guardarSeccion(2); else fase = F_FIN;
      } else if (seccion == 2) {
        if (seccion2()) guardarSeccion(3); else fase = F_FIN;
      } else {
        if (seccion3()) { mostrarMsg("FIN DE PISTA"); guardarSeccion(1); }
        fase = F_FIN;
      }
      break;

    case F_FIN:
      parar();                                    // la garra se queda cerrada (bonus: sin soltar la pelota)
      break;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(IZQ_IN1, OUTPUT); pinMode(IZQ_IN2, OUTPUT); pinMode(IZQ_PWM, OUTPUT);
  pinMode(DER_IN1, OUTPUT); pinMode(DER_IN2, OUTPUT); pinMode(DER_PWM, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(BTN_SEC, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  servo.attach(PIN_SERVO);  servo.write(SERVO_FRENTE);
  garra.attach(PIN_GARRA);  garra.write(GARRA_CERRADA);

  Wire.begin();
  tira.begin(); tira.setBrightness(40); tira.clear(); tira.show();
  mostrarMsg("INICIANDO...");

  if (!tcs.begin()) errorFatal("Falla TCS34725");

  // Sección guardada (continúa tras un lack of progress). START al encender = ronda nueva.
  seccion = EEPROM.read(0);
  if (seccion < 1 || seccion > 3 || digitalRead(BTN_START) == LOW) guardarSeccion(1);

  pinMode(ENC_IZQ, INPUT_PULLUP); pinMode(ENC_DER, INPUT_PULLUP);
  estadoEncPrevio = PINC;
  PCICR |= _BV(PCIE1); PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
  delay(500);
  mostrarMsg("PULSA START");
}
