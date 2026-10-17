// =====================================================================
//  PISTA B - version basica (basada en el codigo de la Pista A)
//
//  SECCION 1: entra al cuadro de 3x3, le da la vuelta a la casilla del
//             centro mirando a la DERECHA hasta encontrar el lado abierto
//             (la pelota) y CUENTA cuantas esquinas dio. Gira hacia la
//             pelota SIN avanzar, hace la secuencia de LEDs 2 veces y,
//             segun las esquinas, sigue la ruta de salida al checkpoint 1.
//  SECCION 2: avanza de 30 en 30 cm. Antes de cada paso mira al frente:
//             libre -> avanza | pared -> izquierda | tambien pared -> derecha
//             | todo cerrado -> media vuelta. Termina al pisar el ROJO (checkpoint 2).
//  SECCION 3: lee el color de cada casilla y va hacia donde indica:
//             CIAN = derecha | AMARILLO = izquierda | NARANJA = arriba
//             ROSA = abajo   | VERDE claro = FIN
//
//  Los LEDs siempre quedan en BLANCO cuando no estan mostrando otra cosa.
//  Abre el Monitor Serie a 9600 para ver todo lo que el robot decide.
// =====================================================================

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_NeoPixel.h>

Servo miServo;

 // Ruedas izquierdas
 const int IZQ_IN1 = 2;
 const int IZQ_PWM = 3;
 const int IZQ_IN2 = 4;

 // Ruedas derechas
 const int DER_PWM = 5;
 const int DER_IN1 = 6;
 const int DER_IN2 = 7;

 // HC-SR04
 const int TRIG = 8;
 const int ECHO = 9;

 // Servo del ultrasonico
 const int PIN_SERVO = 10;
 const int SERVO_FRENTE = 90;    // posicion inicial
 const int SERVO_DER    = 0;
 const int SERVO_IZQ    = 180;
 const int ESPERA_SERVO_MS = 800;   // MODIFICA: tiempo para que el servo llegue

 // Encoders
 const int Enc1 = A2;
 const int Enc2 = A3;

 // ===================== MOVIMIENTO (los mismos de la Pista A) =====================
 const float DIST_PARED    = 10.0;  // cm: menos o igual = pared (igual que tu Pista A)
 const int PULSOS_AVANCE   = 18;    // MODIFICA: 30 cm (igual que tu Pista A)
 const int PULSOS_GIRO_DER = 17;    // MODIFICA: 90 grados a la derecha
 const int PULSOS_GIRO_IZQ = 29;    // MODIFICA: 90 grados a la izquierda
 const int PULSOS_GIRO_180 = 40;    // MODIFICA: 180 grados (calibrar en el piso)
 // Velocidad de cada lado en PORCENTAJE (0 a 100). 100 = maximo del motor.
 // El robot se desviaba al avanzar, asi que el lado DERECHO va un poquito mas rapido.
 //   - Si todavia se desvia hacia la DERECHA: baja VEL_IZQ (de 2 en 2).
 //   - Si ahora se desvia hacia la IZQUIERDA: sube VEL_IZQ.
 const int VEL_IZQ = 94;   // MODIFICA: ruedas izquierdas
 const int VEL_DER = 100;  // MODIFICA: ruedas derechas (ya estan al maximo)
 const int PAUSA_MS = 500;          // MODIFICA: pausa entre movimientos
 const unsigned long TIMEOUT_MOV_MS = 4000;  // si un movimiento tarda mas, se corta (robot atorado)

 // ANTI-CHOQUE. Al avanzar, el ultrasonico (viendo al frente) revisa la pared:
 //   - ANTES de arrancar: si ya hay pared a esta distancia o menos, NO avanza.
 //   - MIENTRAS avanza: si la ve, se detiene aunque no haya completado los pulsos.
 // Despues el robot sigue su recorrido normal (en la seccion 1, por ejemplo,
 // al llegar a la esquina se detiene antes de la pared y luego gira a la derecha).
 // Si todavia choca: SUBE este valor. Si se queda muy lejos del centro: BAJALO.
 const float DIST_CHOQUE_CM = 10.0;          // MODIFICA (antes 5: muy poco a toda velocidad)
 const unsigned long PERIODO_VIGILANCIA_MS = 35;  // cada cuanto mide mientras avanza

 // NUEVO: freno activo al terminar cada movimiento (el L298N pone el motor en corto).
 // Sin esto, el robot sigue rodando por inercia y se pasa de la casilla.
 const int FRENO_MS = 80;                    // MODIFICA: 0 = sin freno (como antes)

 // NUEVO: colores de INICIO y FIN. Al encender lee la casilla de inicio ANTES de
 // moverse, y al llegar al FIN lo muestra. En los dos casos parpadea el color y
 // luego lo deja fijo este tiempo.
 const int ESPERA_COLOR_MS  = 2000;          // MODIFICA
 const int PARPADEOS_COLOR  = 3;

 // ===================== QUE SECCION CORRE AL ENCENDER =====================
 // 1 = empieza en el INICIO y hace la seccion 1 (y luego la 2 y la 3 cuando esten listas)
 // 3 = empieza directo en la seccion 3: pon el robot SOBRE el checkpoint 2
 //     viendo hacia la seccion 3 (sirve para probarla sola)
 const int SECCION_INICIAL = 1;     // MODIFICA

 // ===================== SECCION 1: PELOTA =====================
 // Cuantas casillas avanza desde el INICIO antes de empezar a mirar a la derecha.
 // Tiene que quedar en el CENTRO DE UN LADO del 3x3 con la casilla de la pelota
 // a su DERECHA. Con el inicio como en el PDF (abajo de la esquina izquierda):
 //   1 casilla  -> queda en la esquina (a la derecha NO esta la pelota)
 //   2 casillas -> queda en el centro del lado izquierdo  <- correcto
 const int CASILLAS_ENTRADA = 2;    // MODIFICA segun donde este el inicio
 // Si mira a la derecha y la lectura esta entre DIST_PARED y esto, es el lado
 // abierto de la casilla del centro (ve la pelota o la pared del fondo, ~45 cm)
 const float DIST_MAX_CENTRO = 60.0;
 const int VECES_SECUENCIA = 2;     // cuantas veces se repite la secuencia de LEDs
 const int MS_POR_LED      = 150;   // MODIFICA: velocidad de la secuencia
 const int MAX_PASOS_CHECKPOINT = 16;  // 16 casillas = 2 vueltas al 3x3 buscando el checkpoint

 // ----- SALIDA AL CHECKPOINT 1 SEGUN LAS ESQUINAS QUE DIO -----
 // true  = usa las rutas fijas de abajo (rapido, para el acomodo del PDF:
 //         inicio abajo de la esquina izquierda, checkpoint a la derecha
 //         de la casilla de en medio del lado derecho)
 // false = busca el checkpoint rodeando el 3x3 (sirve para cualquier acomodo)
 const bool USAR_RUTA_FIJA = true;   // MODIFICA

 // Cada ruta empieza con el robot VIENDO A LA PELOTA (despues de los LEDs).
 // Letras:  A = avanzar 30 cm | D = girar derecha | I = girar izquierda | V = vuelta 180
 //
 //   Mapa del 3x3 (C = checkpoint, P = pelota):      Donde queda el robot:
 //        +---+---+---+                               0 esquinas -> lado IZQUIERDO
 //        |   | 1 |   |                               1 esquina  -> lado de ARRIBA
 //        +---+---+---+                               2 esquinas -> lado DERECHO (junto al checkpoint)
 //        | 0 | P | 2 | C                             3 esquinas -> lado de ABAJO
 //        +---+---+---+
 //        |   | 3 |   |
 //        +---+---+---+
 //        INICIO
 const char* RUTA_SALIDA[4] = {
   "IADAADAIA",   // 0 esquinas: izq (ve arriba), sube, derecha, cruza arriba, baja por la derecha, izq, sale
   "IADAIA",      // 1 esquina:  izq (ve a la derecha), avanza, baja, izq, sale
   "VA",          // 2 esquinas: media vuelta y sale (el checkpoint esta junto)
   "DAIADA"       // 3 esquinas: der (ve a la derecha), avanza, sube, der, sale
 };

 // ===================== SECCION 2 =====================
 // true  = al llegar al checkpoint 1 hace la seccion 2 y luego la 3
 // false = se detiene en el checkpoint 1
 const bool HACER_SECCION2 = true;   // MODIFICA
 const int MAX_PASOS_S2 = 20;        // seguridad: maximo de pasos buscando el checkpoint 2
 // Hacia que lado busca primero la salida cuando topa con pared:
 // true = izquierda primero (como en la pista de prueba), false = derecha primero
 const bool PRIMERO_IZQUIERDA_S2 = true;   // MODIFICA

 // ===================== SECCION 3: COLORES =====================
 const int MAX_PASOS_S3 = 15;       // seguridad: maximo de casillas en la seccion 3

 // Direcciones de la pista (NO del robot). "Arriba" = hacia donde ve el robot
 // al salir del checkpoint 2.
 const int ARRIBA    = 0;
 const int DERECHA   = 1;
 const int ABAJO     = 2;
 const int IZQUIERDA = 3;

 // Sensor de color TCS34725 (I2C): SDA -> A4, SCL -> A5
 Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
 bool colorOk = false;

 // LEDs WS2812B: DATA -> D12
 const int PIN_LEDS   = 12;
 const int NUM_LEDS   = 8;
 const int LEDS_COLOR = 8;   // los 8 LEDs muestran el color
 // Si los colores salen cambiados (rojo se ve verde), cambia NEO_GRB por NEO_RGB
 Adafruit_NeoPixel tira(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

 // ================= CALIBRACION DEL COLOR (igual que en la Pista A) =================
 // Pon el robot sobre piso BLANCO, abre el Monitor Serie (9600) y copia
 // aqui los valores "crudo R G B" que imprime.
 const float BLANCO_R = 539.0;   // tus valores calibrados de la Pista A
 const float BLANCO_G = 468.0;
 const float BLANCO_B = 327.0;

 const unsigned int LUZ_MINIMA = 50;    // C menor a esto = muy oscuro
 const unsigned int SATURADO   = 20000; // C mayor a esto = sensor saturado (baja la ganancia a 1X)
 const float SAT_MINIMA  = 0.15;        // menos saturado que esto = blanco o gris
 const float GAMMA       = 1.5;         // corrige como se ven los tonos en el LED
 const int BRILLO_LEDS   = 255;         // brillo de la tira (0-255)
 const int BRILLO_BLANCO = 40;          // brillo del blanco "de siempre"

 // ---- NUEVO: rangos de TONO (hue, 0-360 grados) para saber QUE color es ----
 // El Monitor Serie imprime "tono = X" en cada lectura. Pon el robot sobre
 // cada color, anota su tono y ajusta estos limites si hace falta.
 //   0 = rojo, 30 = naranja, 60 = amarillo, 120 = verde, 180 = cian, 300 = rosa
 const float TONO_ROJO_MAX     = 10;   // rojo: tono < 10  o  tono >= TONO_ROSA_MAX
 const float TONO_NARANJA_MAX  = 40;   // naranja: 10 a 40
 const float TONO_AMARILLO_MAX = 75;   // amarillo: 40 a 75
 const float TONO_VERDE_MAX    = 160;  // verde: 75 a 160
 const float TONO_CIAN_MAX     = 230;  // cian: 160 a 230
 const float TONO_ROSA_MIN     = 280;  // (230 a 280 es azul: no se usa)
 const float TONO_ROSA_MAX     = 340;  // rosa: 280 a 340
 // =========================================================

 // Colores que el robot reconoce
 const int C_NINGUNO  = 0;
 const int C_BLANCO   = 1;
 const int C_ROJO     = 2;
 const int C_NARANJA  = 3;
 const int C_AMARILLO = 4;
 const int C_VERDE    = 5;
 const int C_CIAN     = 6;
 const int C_ROSA     = 7;
 const char* NOMBRE_COLOR[] = { "ninguno", "blanco", "ROJO", "NARANJA", "AMARILLO", "VERDE", "CIAN", "ROSA" };

 int colorActual = C_NINGUNO;   // lo actualiza detectarColor() cada vez que el robot se detiene
 int servoActual = -1;          // NUEVO: a donde esta viendo el servo ahora
 bool vigilarFrente = false;    // NUEVO: anti-choque, lo prende avanzar()
 bool terminado = false;

 void setup() {
    Serial.begin(9600);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

   pinMode(IZQ_IN1, OUTPUT);
   pinMode(IZQ_IN2, OUTPUT);
   pinMode(IZQ_PWM, OUTPUT);

   pinMode(DER_IN1, OUTPUT);
   pinMode(DER_IN2, OUTPUT);
   pinMode(DER_PWM, OUTPUT);

   pinMode(Enc1, INPUT);
   pinMode(Enc2, INPUT);

   moverMotores(0, 0);

   // LEDs: prueba al encender -> los 8 en ROJO, VERDE y AZUL.
   tira.begin();
   tira.setBrightness(BRILLO_LEDS);
   pintarLeds(255, 0, 0);  delay(400);
   pintarLeds(0, 255, 0);  delay(400);
   pintarLeds(0, 0, 255);  delay(400);

   // Sensor de color
   colorOk = tcs.begin();
   if (colorOk) {
     Serial.println(F("TCS34725 encontrado"));
   } else {
     Serial.println(F("No se encontro el TCS34725 (revisa A4, A5, VIN y GND)"));
     for (int i = 0; i < 3; i++) {
       pintarLeds(255, 0, 0); delay(300);
       pintarLeds(0, 0, 0);   delay(300);
     }
   }
   ledsBlancos();   // blanco "de siempre"

   miServo.attach(PIN_SERVO);
   moverServo(SERVO_FRENTE);  // Posición inicial: viendo al frente
   delay(ESPERA_SERVO_MS);

   // NUEVO: lee el color de la casilla donde arranca (antes de avanzar los
   // primeros 30 cm) y lo muestra en los LEDs, igual que cuando se detiene.
   Serial.println(F("=== COLOR DE LA CASILLA DE INICIO ==="));
   detectarColor();
   mostrarColorDetectado();   // NUEVO: parpadea el color para que el juez lo vea
   delay(ESPERA_COLOR_MS);    // se queda mostrando el color antes de arrancar
 }

// =====================================================================
//  LOOP: corre las secciones UNA sola vez, en orden
// =====================================================================
void loop() {
  if (terminado) {
    moverMotores(0, 0);
    return;
  }

  bool ok = true;
  if (SECCION_INICIAL <= 1) ok = seccion1();
  if (ok && SECCION_INICIAL <= 2) ok = seccion2();
  if (ok) seccion3();

  moverServo(SERVO_FRENTE);
  moverMotores(0, 0);
  Serial.println(F("=== FIN DEL PROGRAMA ==="));
  terminado = true;
}

// =====================================================================
//  SECCION 1: encontrar la pelota y llegar al checkpoint 1
// =====================================================================
bool seccion1() {
  Serial.println(F("=== SECCION 1 ==="));

  // A) Entrar al 3x3 hasta quedar en el centro de un lado
  for (int i = 0; i < CASILLAS_ENTRADA; i++) {
    avanzar();
  }

  // B) Rodear la casilla del centro (sentido horario), mirando SIEMPRE a la derecha.
  //    En cada lado: mira a la derecha. Si hay pared -> avanza a la esquina,
  //    gira a la derecha, avanza al centro del siguiente lado y vuelve a mirar.
  //    Son 4 lados como maximo (una vuelta completa).
  bool pelota = false;
  int esquinas = 0;           // NUEVO: cuantas vueltas de 90 grados dio antes de encontrar la pelota
  for (int lado = 0; lado < 4; lado++) {
    float d = medirEn(SERVO_DER);
    Serial.print(F("Esquinas = ")); Serial.print(esquinas);
    Serial.print(F(" - derecha: ")); Serial.println(d);

    if (d > DIST_PARED && d < DIST_MAX_CENTRO) {
      pelota = true;          // aqui NO hay pared: es la entrada de la casilla de la pelota
      break;
    }
    avanzar();     // a la esquina
    girarDer();    // dar la vuelta a la esquina
    esquinas++;    // una esquina mas
    avanzar();     // al centro del siguiente lado
  }
  moverServo(SERVO_FRENTE);
  delay(ESPERA_SERVO_MS);

  if (pelota) {
    Serial.print(F("-> PELOTA ENCONTRADA despues de "));
    Serial.print(esquinas); Serial.println(F(" esquinas"));
    girarDer();                          // voltea hacia la pelota, SIN avanzar
    secuenciaLeds(VECES_SECUENCIA);      // LED 1, 2, 3 ... 8, dos veces
    // AQUI va la garra cuando la usen (servo en D11): avanzar, cerrar, retroceder

    // C) Salir al checkpoint 1 con la ruta que toca segun las esquinas
    if (USAR_RUTA_FIJA) {
      ejecutarRuta(RUTA_SALIDA[esquinas]);
      if (colorActual == C_ROJO) {
        Serial.println(F("-> CHECKPOINT 1 ALCANZADO"));
      } else {
        Serial.println(F("-> Termine la ruta pero no veo ROJO (revisa pulsos o el acomodo)"));
      }
      return true;
    }
    girarIzq();                          // regresa a su rumbo alrededor del 3x3
  } else {
    Serial.println(F("-> No encontre la pelota, busco el checkpoint de todos modos"));
  }

  // C) Buscar el checkpoint 1 rodeando el 3x3 (si no hay ruta fija o no hubo pelota)
  if (buscarCheckpoint()) {
    Serial.println(F("-> CHECKPOINT 1 ALCANZADO"));
    return true;
  }
  Serial.println(F("-> No encontre el checkpoint 1"));
  return false;
}

// Sigue rodeando el 3x3 (mismo sentido) y en cada casilla mira HACIA AFUERA
// (a la izquierda; en las esquinas tambien al frente). Si encuentra un hueco
// en la pared exterior, entra y lee el color del piso:
//   ROJO  -> es el checkpoint: listo.
//   otro  -> era el INICIO (verde): regresa y sigue buscando.
bool buscarCheckpoint() {
  bool enEsquina = false;   // empezamos en el centro de un lado
  for (int paso = 0; paso < MAX_PASOS_CHECKPOINT; paso++) {

    // a) Hueco a la izquierda (hacia afuera del 3x3)
    if (estaLibre(SERVO_IZQ)) {
      Serial.println(F("Hueco a la izquierda: reviso si es el checkpoint"));
      girarIzq();
      avanzar();
      if (colorActual == C_ROJO) return true;
      // No era: regresa a la misma casilla y al mismo rumbo
      girar180();
      avanzar();
      girarIzq();
    }

    // b) En las esquinas tambien hay pared exterior al frente
    if (enEsquina) {
      if (estaLibre(SERVO_FRENTE)) {
        Serial.println(F("Hueco al frente: reviso si es el checkpoint"));
        avanzar();
        if (colorActual == C_ROJO) return true;
        girar180();
        avanzar();
        girar180();
      }
      girarDer();   // dar la vuelta a la esquina
    }

    avanzar();
    enEsquina = !enEsquina;   // se alternan: lado, esquina, lado, esquina...
  }
  return false;
}

// Ejecuta una ruta escrita con letras: A = avanzar, D = derecha,
// I = izquierda, V = vuelta 180. Ejemplo: "IADA"
void ejecutarRuta(const char* ruta) {
  Serial.print(F("Ruta: ")); Serial.println(ruta);
  for (int i = 0; ruta[i] != '\0'; i++) {
    char paso = ruta[i];
    if (paso == 'A')      avanzar();
    else if (paso == 'D') girarDer();
    else if (paso == 'I') girarIzq();
    else if (paso == 'V') girar180();
  }
}

// =====================================================================
//  SECCION 2: llegar al checkpoint 2 aunque la salida cambie de lugar
//  (en el reglamento esta derecho; en la pista de prueba esta a la izquierda)
//
//  1) Avanza DERECHO de 30 en 30 mientras no haya pared enfrente.
//  2) Cuando hay pared enfrente, busca un HUECO a los lados
//     (primero a la izquierda, luego a la derecha).
//  3) Si hay hueco, entra una casilla y lee el piso:
//       ROJO  -> es la salida (checkpoint 2): listo.
//       otro  -> era otro carril de la seccion 2: se vuelve a poner viendo
//                hacia adelante y sigue buscando (recordando hacia que lado iba).
//  Mientras avanza, el anti-choque sigue vigilando.
// =====================================================================
bool seccion2() {
  if (!HACER_SECCION2) {
    Serial.println(F("=== SECCION 2: desactivada, me detengo en el checkpoint 1 ==="));
    return false;
  }

  Serial.println(F("=== SECCION 2 ==="));
  bool buscarIzq = PRIMERO_IZQUIERDA_S2;   // hacia que lado busca primero la salida

  for (int paso = 0; paso < MAX_PASOS_S2; paso++) {

    // 1) Frente libre -> avanza
    float dFrente = medirEn(SERVO_FRENTE);
    Serial.print(F("Frente: ")); Serial.println(dFrente);
    if (hayEspacio(dFrente) && avanzar()) {
      if (colorActual == C_ROJO) {
        Serial.println(F("-> CHECKPOINT 2 ALCANZADO (salida derecho)"));
        return true;
      }
      continue;
    }

    // 2) Pared enfrente -> busca hueco a los lados
    bool meMovi = false;
    for (int intento = 0; intento < 2 && !meMovi; intento++) {
      bool izq = (intento == 0) ? buscarIzq : !buscarIzq;
      float d = medirEn(izq ? SERVO_IZQ : SERVO_DER);
      Serial.print(izq ? F("Izq: ") : F("Der: ")); Serial.println(d);
      if (!hayEspacio(d)) continue;

      // 3) Hay hueco: entra y revisa el color
      Serial.println(izq ? F("-> Hueco a la izquierda, reviso") : F("-> Hueco a la derecha, reviso"));
      if (izq) girarIzq(); else girarDer();
      if (!avanzar()) {                       // no pudo avanzar: deshace el giro
        if (izq) girarDer(); else girarIzq();
        continue;
      }
      if (colorActual == C_ROJO) {
        Serial.println(izq ? F("-> CHECKPOINT 2 ALCANZADO (salida a la izquierda)")
                           : F("-> CHECKPOINT 2 ALCANZADO (salida a la derecha)"));
        return true;
      }
      // No era la salida: se vuelve a poner viendo hacia adelante
      if (izq) girarDer(); else girarIzq();
      buscarIzq = izq;                        // sigue buscando hacia ese mismo lado
      meMovi = true;
    }

    // Frente y los dos lados cerrados: media vuelta y vuelve a buscar
    if (!meMovi) {
      Serial.println(F("-> Todo cerrado: media vuelta"));
      girar180();
      buscarIzq = !buscarIzq;
    }
  }

  Serial.println(F("-> No encontre el checkpoint 2, me detengo"));
  return false;
}

// true si en esa lectura NO hay pared. -1 (sin eco) cuenta como libre:
// la pared mas cercana esta lejos. El anti-choque protege si fue una lectura mala.
bool hayEspacio(float d) {
  return (d < 0) || (d > DIST_PARED);
}

// =====================================================================
//  SECCION 3: seguir el camino que marcan los colores
// =====================================================================
bool seccion3() {
  Serial.println(F("=== SECCION 3 ==="));
  // NUEVO: como en la seccion 2 la salida puede estar a un lado, el robot no
  // siempre queda viendo hacia la seccion 3. Aqui busca la primera casilla de
  // color y esa direccion pasa a ser "ARRIBA".
  if (!entrarSeccion3()) {
    Serial.println(F("-> No encontre la entrada a la seccion 3, me detengo"));
    return false;
  }
  int rumbo = ARRIBA;   // la direccion con la que entro desde el checkpoint 2 es "arriba"

  for (int paso = 0; paso < MAX_PASOS_S3; paso++) {
    int c = colorActual;

    // Si no reconocio el color, vuelve a leer una vez (pudo quedar en la orilla blanca)
    if (direccionDeColor(c) < 0 && c != C_VERDE) {
      detectarColor();
      c = colorActual;
    }

    if (c == C_VERDE) {
      Serial.println(F("=== COLOR DE LA CASILLA DE FIN ==="));
      mostrarColorDetectado();   // NUEVO: parpadea el color del FIN y lo deja fijo
      Serial.println(F("-> FIN DE LA PISTA"));
      return true;
    }

    int destino = direccionDeColor(c);
    if (destino < 0) {
      Serial.print(F("-> Color sin direccion (")); Serial.print(NOMBRE_COLOR[c]);
      Serial.println(F("), me detengo para no salirme del camino"));
      return false;
    }

    girarHacia(rumbo, destino);
    rumbo = destino;
    avanzar();
  }
  Serial.println(F("-> Demasiados pasos, me detengo"));
  return false;
}

// Parado en el checkpoint 2: prueba FRENTE, IZQUIERDA y DERECHA. En cada hueco
// entra una casilla y lee el color. Si es un color con direccion (cian, amarillo,
// naranja o rosa) ya esta dentro de la seccion 3. Si no, regresa y prueba otro.
bool entrarSeccion3() {
  const int angulos[3] = { SERVO_FRENTE, SERVO_IZQ, SERVO_DER };
  for (int k = 0; k < 3; k++) {
    float d = medirEn(angulos[k]);
    if (!hayEspacio(d)) continue;

    if (k == 1) girarIzq();
    else if (k == 2) girarDer();

    if (avanzar()) {
      if (direccionDeColor(colorActual) >= 0) {
        Serial.println(F("-> Entre a la seccion 3"));
        return true;
      }
      // No era la seccion 3: regresa al checkpoint
      girar180();
      avanzar();
      // y vuelve a la orientacion de antes
      if (k == 0) girar180();
      else if (k == 1) girarIzq();
      else girarDer();
    } else {
      // no pudo avanzar: solo deshace el giro
      if (k == 1) girarDer();
      else if (k == 2) girarIzq();
    }
  }
  return false;
}

// Color -> direccion de la pista (segun la imagen 2.6 del PDF)
// MODIFICA aqui si cambian las reglas
int direccionDeColor(int c) {
  if (c == C_CIAN)     return DERECHA;
  if (c == C_AMARILLO) return IZQUIERDA;
  if (c == C_NARANJA)  return ARRIBA;
  if (c == C_ROSA)     return ABAJO;
  return -1;   // blanco, rojo, ninguno: no indica direccion
}

// Gira del rumbo actual al rumbo destino por el lado mas corto
void girarHacia(int rumbo, int destino) {
  int giro = (destino - rumbo + 4) % 4;
  if (giro == 1)      { Serial.println(F("-> giro derecha"));   girarDer(); }
  else if (giro == 3) { Serial.println(F("-> giro izquierda")); girarIzq(); }
  else if (giro == 2) { Serial.println(F("-> giro 180"));       girar180(); }
  else                { Serial.println(F("-> sigo derecho")); }
}

// =====================================================================
//  MOVIMIENTOS BASICOS (usan tu moversePulsos)
// =====================================================================
// Avanza 30 cm. Regresa false si NO pudo (habia pared enfrente desde antes).
bool avanzar() {
  // El ultrasonico tiene que ver AL FRENTE para vigilar choques
  if (servoActual != SERVO_FRENTE) {
    moverServo(SERVO_FRENTE);
    delay(ESPERA_SERVO_MS);
  }

  // NUEVO: si YA hay pared muy cerca enfrente, ni siquiera arranca
  if (paredMuyCerca()) {
    Serial.println(F("!! Pared enfrente antes de avanzar: no avanzo"));
    return false;
  }

  vigilarFrente = true;                       // anti-choque mientras avanza
  moversePulsos(PULSOS_AVANCE, VEL_IZQ, VEL_DER);
  vigilarFrente = false;
  delay(PAUSA_MS);
  return true;
}

// true si 2 de 3 lecturas rapidas ven una pared a DIST_CHOQUE_CM o menos
bool paredMuyCerca() {
  int cerca = 0;
  for (int i = 0; i < 3; i++) {
    float d = medirDistanciaRapida();
    if (d > 0 && d <= DIST_CHOQUE_CM) cerca++;
    delay(30);
  }
  return cerca >= 2;
}

void girarDer() {
  moversePulsos(PULSOS_GIRO_DER, VEL_IZQ, -VEL_DER);
  delay(PAUSA_MS);
}

void girarIzq() {
  moversePulsos(PULSOS_GIRO_IZQ, -VEL_IZQ, VEL_DER);
  delay(PAUSA_MS);
}

void girar180() {
  moversePulsos(PULSOS_GIRO_180, VEL_IZQ, -VEL_DER);
  delay(PAUSA_MS);
}

void moverMotor(int IN1, int IN2, int PWM, int velocidad) {

   // velocidad en porcentaje: -100 (atras a tope) ... 100 (adelante a tope)
   velocidad = constrain(velocidad, -100, 100);

   int pwm = map(abs(velocidad), 0, 100, 0, 255);   // 100% = 255 (nunca se pasa de 255)

   if (velocidad > 0) {
     digitalWrite(IN1, HIGH);
     digitalWrite(IN2, LOW);
   }
   else if (velocidad < 0) {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, HIGH);
   }
   else {
     digitalWrite(IN1, LOW);
     digitalWrite(IN2, LOW);
   }

   analogWrite(PWM, pwm);
 }

 void moverMotores(int izquierda, int derecha) {
   moverMotor(IZQ_IN1, IZQ_IN2, IZQ_PWM, izquierda);
   moverMotor(DER_IN1, DER_IN2, DER_PWM, derecha);
 }

// =====================================================================
//  LEDS
// =====================================================================
int ledActualR = 0, ledActualG = 0, ledActualB = 0;   // ultimo color pintado

void pintarLeds(int rojo, int verde, int azul) {
  ledActualR = rojo; ledActualG = verde; ledActualB = azul;
  for (int i = 0; i < LEDS_COLOR; i++) {
    tira.setPixelColor(i, tira.Color(rojo, verde, azul));
  }
  tira.show();
}

// NUEVO: parpadea el color que esta mostrando y lo deja encendido
void mostrarColorDetectado() {
  int r = ledActualR, g = ledActualG, b = ledActualB;
  Serial.print(F("Mostrando color: ")); Serial.println(NOMBRE_COLOR[colorActual]);
  for (int i = 0; i < PARPADEOS_COLOR; i++) {
    pintarLeds(0, 0, 0); delay(250);
    pintarLeds(r, g, b); delay(250);
  }
}

// El blanco "de siempre"
void ledsBlancos() {
  pintarLeds(BRILLO_BLANCO, BRILLO_BLANCO, BRILLO_BLANCO);
}

// Secuencia de la pelota: se prende el LED 1, luego 1-2, 1-2-3 ... hasta los 8,
// se apagan, y se repite "veces" veces. Al final regresan al blanco.
void secuenciaLeds(int veces) {
  for (int v = 0; v < veces; v++) {
    tira.clear();
    tira.show();
    delay(MS_POR_LED);
    for (int i = 0; i < NUM_LEDS; i++) {
      tira.setPixelColor(i, tira.Color(255, 255, 255));   // MODIFICA el color de la secuencia
      tira.show();
      delay(MS_POR_LED);
    }
    delay(300);
  }
  ledsBlancos();
}

// Lee el piso, muestra en los LEDs el MISMO tono pero bien vivo (igual que en
// la Pista A) y ADEMAS guarda en colorActual que color es (para la seccion 3).
void detectarColor() {
  colorActual = C_NINGUNO;
  if (!colorOk) {
    Serial.println(F("Sin sensor de color"));
    return;
  }

  delay(100);  // que el robot termine de frenar

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);   // se descarta: se tomo mientras el robot se movia

  long sumaR = 0, sumaG = 0, sumaB = 0, sumaC = 0;
  for (int i = 0; i < 3; i++) {
    tcs.getRawData(&r, &g, &b, &c);
    sumaR += r;
    sumaG += g;
    sumaB += b;
    sumaC += c;
  }
  float crudoR = sumaR / 3.0;
  float crudoG = sumaG / 3.0;
  float crudoB = sumaB / 3.0;
  float crudoC = sumaC / 3.0;

  Serial.print(F("crudo R G B C: "));
  Serial.print(crudoR, 0); Serial.print(F(" "));
  Serial.print(crudoG, 0); Serial.print(F(" "));
  Serial.print(crudoB, 0); Serial.print(F(" "));
  Serial.print(crudoC, 0);

  if (crudoC > SATURADO) {
    Serial.print(F("  (SATURADO: cambia a TCS34725_GAIN_1X)"));
  }

  // Muy oscuro: no hay color, LEDs en blanco
  if (crudoC < LUZ_MINIMA) {
    ledsBlancos();
    Serial.println(F("  -> oscuro"));
    return;
  }

  // 1) Balance de blancos: el piso blanco queda como (1, 1, 1)
  float rf = crudoR / BLANCO_R;
  float gf = crudoG / BLANCO_G;
  float bf = crudoB / BLANCO_B;

  // 2) Que tan "colorido" es (0 = gris/blanco, 1 = color puro)
  float maximo = max(rf, max(gf, bf));
  float minimo = min(rf, min(gf, bf));
  float saturacion = (maximo - minimo) / maximo;

  if (saturacion < SAT_MINIMA) {
    colorActual = C_BLANCO;
    ledsBlancos();
    Serial.println(F("  -> blanco/gris"));
    return;
  }

  // NUEVO: tono (hue) en grados, sirve para saber QUE color es
  float delta = maximo - minimo;
  float tono;
  if (maximo == rf)      tono = 60.0 * ((gf - bf) / delta);
  else if (maximo == gf) tono = 60.0 * ((bf - rf) / delta + 2.0);
  else                   tono = 60.0 * ((rf - gf) / delta + 4.0);
  if (tono < 0) tono += 360.0;

  if (tono < TONO_ROJO_MAX || tono >= TONO_ROSA_MAX) colorActual = C_ROJO;
  else if (tono < TONO_NARANJA_MAX)                  colorActual = C_NARANJA;
  else if (tono < TONO_AMARILLO_MAX)                 colorActual = C_AMARILLO;
  else if (tono < TONO_VERDE_MAX)                    colorActual = C_VERDE;
  else if (tono < TONO_CIAN_MAX)                     colorActual = C_CIAN;
  else if (tono >= TONO_ROSA_MIN)                    colorActual = C_ROSA;
  else                                               colorActual = C_NINGUNO;   // azul

  // 3) Quitar la parte blanca y estirar (para que el LED se vea vivo)
  rf = (rf - minimo) / delta;
  gf = (gf - minimo) / delta;
  bf = (bf - minimo) / delta;

  // 4) Gamma
  int ledR = (int)(pow(rf, GAMMA) * 255 + 0.5);
  int ledG = (int)(pow(gf, GAMMA) * 255 + 0.5);
  int ledB = (int)(pow(bf, GAMMA) * 255 + 0.5);

  pintarLeds(ledR, ledG, ledB);

  Serial.print(F("  tono = ")); Serial.print(tono, 0);
  Serial.print(F("  -> ")); Serial.println(NOMBRE_COLOR[colorActual]);
}

// =====================================================================
//  ULTRASONICO Y SERVO
// =====================================================================
float medirDistancia() {

  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duracion = pulseIn(ECHO, HIGH, 30000);

  if (duracion == 0) {
    return -1;
  }

  float distancia = duracion * 0.0343 / 2.0;

  return distancia;
}


void moverServo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  miServo.write(angulo);
  servoActual = angulo;
}

// NUEVO: medicion rapida (~3 ms) para usar MIENTRAS avanza.
// Solo ve hasta ~50 cm, que es lo unico que importa para no chocar,
// y no bloquea tanto como para perder pulsos de los encoders.
float medirDistanciaRapida() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duracion = pulseIn(ECHO, HIGH, 3000);
  if (duracion == 0) return -1;
  return duracion * 0.0343 / 2.0;
}

// Mueve el servo, espera y mide 3 veces. Regresa la lectura de en medio
// (asi una lectura loca no engana al robot). -1 = ninguna lectura tuvo eco.
float medirEn(int angulo) {
  moverServo(angulo);
  delay(ESPERA_SERVO_MS);
  float v[3];
  int n = 0;
  for (int i = 0; i < 3; i++) {
    float d = medirDistancia();
    if (d > 0) v[n++] = d;
    delay(30);
  }
  if (n == 0) return -1;
  if (n == 1) return v[0];
  if (n == 2) return (v[0] + v[1]) / 2.0;
  // mediana de 3
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  if (v[1] > v[2]) { float t = v[1]; v[1] = v[2]; v[2] = t; }
  if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
  return v[1];
}

// true si NO hay pared en esa direccion.
// Ojo: aqui "sin eco" (-1) cuenta como LIBRE, porque al ver por un hueco
// hacia afuera del 3x3 la siguiente pared puede estar muy lejos.
bool estaLibre(int angulo) {
  float d = medirEn(angulo);
  moverServo(SERVO_FRENTE);
  delay(ESPERA_SERVO_MS / 2);
  return (d < 0) || (d > DIST_PARED);
}

// =====================================================================
//  TU FUNCION DE PULSOS (igual que en la Pista A + un tiempo maximo)
// =====================================================================
int pulsos1 = 0;
int pulsos2 = 0;
int Anterior1 = 0;
int Anterior2 = 0;

// NUEVO: freno activo del L298N (EN en alto + IN1 = IN2) por FRENO_MS
void frenar() {
  if (FRENO_MS > 0) {
    digitalWrite(IZQ_IN1, LOW); digitalWrite(IZQ_IN2, LOW); analogWrite(IZQ_PWM, 255);
    digitalWrite(DER_IN1, LOW); digitalWrite(DER_IN2, LOW); analogWrite(DER_PWM, 255);
    delay(FRENO_MS);
  }
  moverMotores(0, 0);
}

void moversePulsos(int pulsos, int izq, int der){
  pulsos1 = 0;
  pulsos2 = 0;
  Anterior1 = 0;
  Anterior2 = 0;
  unsigned long inicio = millis();   // NUEVO: si se atora contra algo, no empuja para siempre
  unsigned long tMedida = 0;         // NUEVO: anti-choque
  int lecturasCerca = 0;
  while((pulsos > pulsos1) && (pulsos > pulsos2)){

    moverMotores(izq, der);

    int Actual1 = analogRead(Enc1) > 512;
    int Actual2 = analogRead(Enc2) > 512;

    if (Actual1 == 1 && Anterior1 == 0) {
      pulsos1++;
    }
    if (Actual2 == 1 && Anterior2 == 0) {
      pulsos2++;
    }

    Anterior1 = Actual1;
    Anterior2 = Actual2;

    if (millis() - inicio > TIMEOUT_MOV_MS) {
      Serial.println(F("!! Movimiento cortado por tiempo (robot atorado?)"));
      break;
    }

    // NUEVO: si hay una pared MUY cerca enfrente, deja de avanzar.
    // Pide 2 lecturas seguidas para que una lectura loca no lo frene.
    if (vigilarFrente && millis() - tMedida >= PERIODO_VIGILANCIA_MS) {
      tMedida = millis();
      float d = medirDistanciaRapida();
      if (d > 0 && d <= DIST_CHOQUE_CM) {
        lecturasCerca++;
        if (lecturasCerca >= 2) {
          Serial.print(F("!! Pared enfrente a ")); Serial.print(d);
          Serial.println(F(" cm: dejo de avanzar y sigo el recorrido"));
          break;
        }
      } else {
        lecturasCerca = 0;
      }
    }

    //Funcion para evitar las lineas (seccion 2)
  }
  frenar();          // NUEVO: freno activo en vez de dejarlo rodar
  detectarColor();   // cada vez que el robot se detiene, lee el piso y lo muestra en los LEDs
}
