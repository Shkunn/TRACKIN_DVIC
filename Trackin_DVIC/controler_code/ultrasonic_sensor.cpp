#include <Arduino.h>

// Code d'exemple pour un capteur à ultrasons HC-SR04.

/* Constantes pour les broches */
const byte TRIGGER_PIN = 2; // Broche TRIGGER
const byte ECHO_PIN = 3;    // Broche ECHO

const byte TRIGGER_PIN_B = 4; // Broche TRIGGER
const byte ECHO_PIN_B = 5;    // Broche ECHO

const byte TRIGGER_PIN_C = 7; // Broche TRIGGER
const byte ECHO_PIN_C = 6;    // Broche ECHO

const byte TRIGGER_PIN_D = 8; // Broche TRIGGER
const byte ECHO_PIN_D = 9;    // Broche ECHO
 
/* Constantes pour le timeout */
const unsigned long MEASURE_TIMEOUT = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_B = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_C = 25000UL; // 25ms = ~8m à 340m/s
const unsigned long MEASURE_TIMEOUT_D = 25000UL; // 25ms = ~8m à 340m/s

/* Vitesse du son dans l'air en mm/us */
const float SOUND_SPEED = 340.0 / 1000;

/** Fonction setup() */
void setup() {
   
  /* Initialise le port série */
  Serial.begin(115200);
   
  /* Initialise les broches */
  pinMode(TRIGGER_PIN, OUTPUT);
  digitalWrite(TRIGGER_PIN, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN, INPUT);

  pinMode(TRIGGER_PIN_B, OUTPUT);
  digitalWrite(TRIGGER_PIN_B, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN_B, INPUT);

  pinMode(TRIGGER_PIN_C, OUTPUT);
  digitalWrite(TRIGGER_PIN_C, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN_C, INPUT);

  pinMode(TRIGGER_PIN_D, OUTPUT);
  digitalWrite(TRIGGER_PIN_D, LOW); // La broche TRIGGER doit être à LOW au repos
  pinMode(ECHO_PIN_D, INPUT);
}
 
/** Fonction loop() */
void loop() {
  
  /* 1. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
  
  /* 2. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure = pulseIn(ECHO_PIN, HIGH, MEASURE_TIMEOUT);

  /* 1B. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_B, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_B, LOW);
  
  /* 2B. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_B = pulseIn(ECHO_PIN_B, HIGH, MEASURE_TIMEOUT_B);

  /* 1C. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_C, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_C, LOW);
  
  /* 2C. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_C = pulseIn(ECHO_PIN_C, HIGH, MEASURE_TIMEOUT_C);

  /* 1D. Lance une mesure de distance en envoyant une impulsion HIGH de 10µs sur la broche TRIGGER */
  digitalWrite(TRIGGER_PIN_D, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN_D, LOW);
  
  /* 2D. Mesure le temps entre l'envoi de l'impulsion ultrasonique et son écho (si il existe) */
  long measure_D = pulseIn(ECHO_PIN_D, HIGH, MEASURE_TIMEOUT_D);
   
  /* 3. Calcul la distance à partir du temps mesuré */
  float distance_mm = measure / 2.0 * SOUND_SPEED;
  float distance_mm_B = measure_B / 2.0 * SOUND_SPEED;
  float distance_mm_C = measure_C / 2.0 * SOUND_SPEED;
  float distance_mm_D = measure_D / 2.0 * SOUND_SPEED;
   
  /* Affiche les résultats en mm, cm et m */
  /*
  Serial.print(F("Distance: "));
  Serial.print(distance_mm);
  Serial.print(F("mm ("));
  Serial.print(distance_mm / 10.0, 2);
  Serial.print(F("cm, "));
  Serial.print(distance_mm / 1000.0, 2);
  Serial.print(F("m)"));

  Serial.print(F(" DistanceB: "));
  Serial.print(distance_mm_B);
  Serial.print(F("mm ("));
  Serial.print(distance_mm_B / 10.0, 2);
  Serial.print(F("cm, "));
  Serial.print(distance_mm_B / 1000.0, 2);
  Serial.print(F("m)"));

  Serial.print(F(" DistanceC: "));
  Serial.print(distance_mm_C);
  Serial.print(F("mm ("));
  Serial.print(distance_mm_C / 10.0, 2);
  Serial.print(F("cm, "));
  Serial.print(distance_mm_C / 1000.0, 2);
  Serial.print(F("m)"));

  Serial.print(F(" DistanceD: "));
  Serial.print(distance_mm_D);
  Serial.print(F("mm ("));
  Serial.print(distance_mm_D / 10.0, 2);
  Serial.print(F("cm, "));
  Serial.print(distance_mm_D / 1000.0, 2);
  Serial.println(F("m)"));*/
  String message = String(distance_mm)+"/"+String(distance_mm_B)+"/"+String(distance_mm_C)+"/"+String(distance_mm_D)+"/";
  Serial.println(message);
   
  /* Délai d'attente pour éviter d'afficher trop de résultats à la seconde */
  delay(100);
}