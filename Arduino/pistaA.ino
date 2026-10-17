/  PISTA A (versión corregida)

//  Librerías:
//  - Adafruit TCS34725
//  - Adafruit NeoPixel
//  (Wire y Servo ya vienen con el IDE)

//  MODO PRUEBAS (en el Monitor Serial, 115200, "Nueva línea"):
//    e = escanear paredes (frente/der/izq)
//    c = leer color + valores crudos
//    a = avanzar 1 casilla
//    d = girar 90 der
//    i = girar 90 izq
//    f = ajustar distancia con pared frontal
//    y = mostrar conteos de encoder

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>
#include <string.h>
#include <stdlib.h>

// PINES (AJUSTAR)
const int IZQ_IN1 = 2, IZQ_IN2 = 4, IZQ_PWM = 3;   
const int DER_IN1 = 6, DER_IN2 = 7, DER_PWM = 5;   
const int TRIG = 8, ECHO = 9;                    
const int PIN_SERVO = 10;                    
const int BTN_START = 13;                                            


// TIRA WS2812B: 8 LED en el pin digital 3.
// LED 0-1: color del TCS34725. LED 2-7: ID ArUco en binario.
const uint8_t LED_PIN = 12;
const uint8_t NUM_LEDS = 8;
const uint8_t LEDS_COLOR = 2;
Adafruit_NeoPixel tira(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);


// CONSTANTES (AJUSTAR)
const int   SERVO_FRENTE = 90, SERVO_IZQ = 170, SERVO_DER = 10;
const int   SERVO_ESPERA_MS = 350;       // tiempo para que el servo llegue a su posicion
const float UMBRAL_PARED_CM = 25.0;      // menos que esto significa q hay pared
const float DIST_FRENTE_CENTRO_CM = 10.0;
                                         // (15 cm - distancia del sensor al centro del robot)
const int   VEL_AVANCE = 140;
// El avance por casilla se controla con PULSOS_CASILLA (encoders).
const bool  HACER_BONUS_1 = true;        // regresar al inicio por el mismo camino

const unsigned long T_RONDA_MS   = 360000UL;
const unsigned long MS_POR_PASO  = 1800UL;   // MIDE esto en tu pista (avance + giro + recentrado)
const unsigned long COLCHON_MS   = 20000UL;  // margen de seguridad
const bool ESPALDA_ES_PARED = true;
const byte MAX_PASOS = 40;

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
Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float yaw = 0; // conservado solo para compatibilidad con los comandos antiguos
Dir rumbo = NORTE;

const byte G = 9;
byte cx = G / 2, cy = G / 2;
bool visitada[G][G];
byte paredes[G][G];                      // bits: 1<<NORTE, 1<<ESTE, ... = hay pared
Dir pila[MAX_PASOS];     byte nPila = 0; // camino limpio inicio -> posicion actual
Dir rutaRojo[MAX_PASOS]; byte nRuta = 0; // camino limpio inicio -> rojo
bool rojoEncontrado = false;
bool encontrado[4];
int  arucoId = -1;
unsigned long tInicio = 0;
Fase fase = F_ESPERA;

float dF, dD, dI;
char textoL0[17] = "";
bool enPrueba = false;

char bufRx[40]; byte nRx = 0;

void actualizarLCD(); // ahora los mensajes salen por Serial y NeoPixel


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


//  TIRA LED: mostrar color detectado en los dos primeros LED.
void mostrarColorLED(ColorPiso color) {
  uint32_t rgb = tira.Color(0, 0, 0);

  switch (color) {
    case C_CIAN:      rgb = tira.Color(0, 255, 255); break;
    case C_AMARILLO:  rgb = tira.Color(255, 255, 0); break;
    case C_NARANJA:   rgb = tira.Color(255, 80, 0); break;
    case C_ROSA:      rgb = tira.Color(255, 0, 100); break;
    case C_ROJO:      rgb = tira.Color(255, 0, 0); break;
    case C_VERDE:     rgb = tira.Color(0, 255, 0); break;
    case C_NINGUNO:
    default:          rgb = tira.Color(0, 0, 0); break;
  }

  // Actualiza únicamente los LED 0 y 1; conserva los bits ArUco.
  tira.setPixelColor(0, rgb);
  tira.setPixelColor(1, rgb);
  tira.show();
}

// Mostrar el ID ArUco en los LED 2-7 (6 bits, MSB a LSB).
void mostrarArucoBinario(int id) {
  if (id < 0 || id > 49) return;

  for (int bit = 5; bit >= 0; bit--) {
    uint8_t led = 2 + (5 - bit);
    bool encendido = (id >> bit) & 0x01;
    tira.setPixelColor(led, encendido ? tira.Color(255, 255, 255) : 0);
  }

  tira.show();
}

//  COMUNICACION SERIAL (Pi o Monitor Serial)
void comandoPrueba(char c);

void procesarLinea(char* s) {
  if (strncmp(s, "ARUCO:", 6) == 0) {
    int id = atoi(s + 6);
    if (arucoId < 0 && id >= 0 && id < 50) {
      arucoId = id;
      mostrarArucoBinario(arucoId);
      actualizarLCD();
    }  // se queda fijo
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

// MENSAJES: Serial + NeoPixel, sin LCD.
void actualizarLCD() { /* reemplazo compatible: ya no hay LCD */ }

void mostrarMsg(const char* m) {
  strncpy(textoL0, m, 16); textoL0[16] = 0;
  Serial.print(F("ESTADO: ")); Serial.println(m);
  uint32_t color = tira.Color(0, 0, 255);
  if (strstr(m, "ERROR") || strstr(m, "SIN ROJO")) color = tira.Color(255, 0, 0);
  else if (strstr(m, "FINAL") || strstr(m, "LISTO") || strstr(m, "BONUS")) color = tira.Color(0, 255, 0);
  else if (strstr(m, "EXPLOR") || strstr(m, "REGRES") || strstr(m, "YENDO")) color = tira.Color(255, 120, 0);
  for (uint8_t i = 2; i < NUM_LEDS; i++) tira.setPixelColor(i, color);
  tira.show();
}

void mostrarColor(ColorPiso c) {
  snprintf(textoL0, 17, "Color: %s", NOMBRE_COLOR[c]);
  Serial.print(F("COLOR: ")); Serial.println(NOMBRE_COLOR[c]);
  mostrarColorLED(c);
}

void errorFatal(const char* m) {
  Serial.print(F("ERROR FATAL: ")); Serial.println(m);
  tira.fill(tira.Color(255, 0, 0)); tira.show();
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
//  (los tonos pueden variar según el reglamento: calibra con varias luces/tonos)
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
void moverRecentrando(Dir d) {
  moverA(d);
  if (paredes[cx][cy] & (1 << rumbo)) ajustarConFrente();
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
bool listoParaTerminar() {
  if (!rojoEncontrado) return false;
  unsigned long usado = millis() - tInicio;
  if (usado >= T_RONDA_MS) return true;
  unsigned long pasos = (unsigned long)nPila + nRuta + (HACER_BONUS_1 ? nRuta : 0);
  return (T_RONDA_MS - usado) < pasos * MS_POR_PASO + COLCHON_MS;
}

void retroceder() {                       // deshace el último movimiento de la pila
  Dir d = opuesto(pila[--nPila]);
  moverRecentrando(d);                 
}

void pasoExplorar() {
  if (listoParaTerminar()) { fase = F_IR_A_ROJO; return; }

  // 1) buscar una casilla vecina libre y sin visitar
  //    [CAMBIO 5] si la pila está llena no se avanza más: se retrocede
  if (nPila < MAX_PASOS) {
    for (byte i = 0; i < 4; i++) {
      Dir d = ORDEN[i];
      int nx = cx + DX[d], ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= G || ny >= G) continue;
      if (visitada[nx][ny]) continue;
      if (paredes[cx][cy] & (1 << d)) continue;

      moverA(d);
      pila[nPila++] = d;
      if (procesarCeldaNueva()) {
        rojoEncontrado = true;
        nRuta = nPila;
        for (byte k = 0; k < nPila; k++) rutaRojo[k] = pila[k];
        mostrarMsg("ROJO!");
        if (listoParaTerminar()) {
          fase = HACER_BONUS_1 ? F_REGRESO : F_FIN;
        } else {
          retroceder();                   // sigue explorando (colores, ArUco, obstáculos)
        }
      }
      return;
    }
  }

  // 2) sin opciones: retroceder o terminar
  if (nPila == 0) {
    if (rojoEncontrado) fase = F_IR_A_ROJO;
    else { mostrarMsg("SIN ROJO"); fase = F_FIN; }
    return;
  }
  retroceder();
}
void irARojo() {
  mostrarMsg("YENDO A ROJO");

  byte comun = 0;
  while (comun < nPila && comun < nRuta && pila[comun] == rutaRojo[comun]) comun++;

  while (nPila > comun) retroceder();

  for (byte i = comun; i < nRuta; i++) {
    moverRecentrando(rutaRojo[i]);        // [CAMBIO 4]
    pila[nPila++] = rutaRojo[i];
  }
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
// El tiempo de ronda (tInicio) se toma al presionar START, antes de calibrar.
void iniciarRonda() {
  memset(visitada, 0, sizeof(visitada));
  memset(paredes, 0, sizeof(paredes));
  memset(encontrado, 0, sizeof(encontrado));
  nPila = 0; nRuta = 0; rojoEncontrado = false; arucoId = -1;
  cx = G / 2; cy = G / 2;
  yaw = 0; rumbo = NORTE;
  servo.write(SERVO_FRENTE);
  Serial.println("MODE:ARUCO");          // le dice a la Pi que busque el marcador
  procesarCeldaNueva();
  if (ESPALDA_ES_PARED) paredes[cx][cy] |= (1 << opuesto(rumbo));   // [CAMBIO 6]
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
    case 'y': Serial.print("Pulsos I/D: "); Serial.print(contarIzq()); Serial.print(" / "); Serial.println(contarDer()); break;
  }
  enPrueba = false;
}

void loop() {
  leerSerialPi();
  switch (fase) {
    case F_ESPERA:
      parar();
      if (digitalRead(BTN_START) == LOW) {
        delay(50);
        tInicio = millis();              
        mostrarMsg("SUELTA ROBOT");     
        esperar(1500);
        mostrarMsg("ENCODERS LISTOS");
        iniciarRonda();
      }
      break;
    case F_EXPLORAR:   pasoExplorar(); break;
    case F_IR_A_ROJO:  irARojo();      break;
    case F_REGRESO:    regresar();     break;
    case F_FIN:        parar();        break;
  }
}

void setup() {
  Serial.begin(115200);

  // Inicializar la tira WS2812B.
  tira.begin();
  tira.setBrightness(40);
  tira.clear();
  tira.show();

  pinMode(IZQ_IN1, OUTPUT); pinMode(IZQ_IN2, OUTPUT); pinMode(IZQ_PWM, OUTPUT);
  pinMode(DER_IN1, OUTPUT); pinMode(DER_IN2, OUTPUT); pinMode(DER_PWM, OUTPUT);
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(BTN_START, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  servo.attach(PIN_SERVO);
  servo.write(SERVO_FRENTE);

  Wire.begin();
  mostrarMsg("INICIANDO...");

  if (!tcs.begin()) errorFatal("Falla TCS34725");

  pinMode(ENC_IZQ, INPUT_PULLUP); pinMode(ENC_DER, INPUT_PULLUP);
  estadoEncPrevio = PINC;
  PCICR |= _BV(PCIE1); PCMSK1 |= _BV(PCINT10) | _BV(PCINT11);
  delay(500);
  mostrarMsg("LISTO: PULSA START");
}