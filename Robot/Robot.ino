#include <Servo.h>
//#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
//#include <Wire.h>

#define debug //Commentez pour cacher les retour série

/**
 * Configuration du mode utilisé
 * Décommenter le mode demandé
 */
//#define LABMODE       //Mode labirinthe
//#define FORWARDMODE   //Mode tout droit
#define SEARCHMODE    //Mode Recherche et attaque de la cible

#define brEnable 13 //Pin d'activation des ponts
/**
 * Pins de controle des moteurs
 * 
 *  A     B     Effet
 *  HIGH  LOW   Sens horaire
 *  LOW   HIGH  Sans anti-horaire
 */
#define motor1A 3
#define motor1B 9
#define motor2A 5
#define motor2B 10

//Pins du capteur arrière
#define echoPinbk 7
#define trigPinbk 6
//Pins du capteur avant
#define echoPinfr 11
#define trigPinfr 8
#define servoPin 4  //Pin de controle du servomoteur

#define interruptPin 2  //Pin d'interruption (non utilisé)

#define startButtonPin 12 //Bouton de départ

#define freeDistance 25   //Distance minimum pour estimer une distance comme libre
#define deadTimeRotating 500  //Temps mort lors de la rotation du servo
#define deadTimeRunning 25    //Temps mort lors de chaque tour de boucle 
#define rightMotorCalibration 0   //Reduction du moteur de droite pour calibration

#define forwardAlternationTime 800 //Le temps minimum pour l'alternance de la direction quand le robot va tout droit
#define forwardAlternationAmount 10 //L'angle a ajouter pour compenser la rotation

#define turnDuration 2000 //Le temps mis pour tourner

#define leftAngle 100   //Angle du servo pour tourner a gauche
#define rightAngle 0    //Angle du servo pour tourner a droite
#define frontAngle 50   //Angle du servo pour aller tout droit

Servo direction;  //Le servo
int currentAngle = -1;  //L'angle actuel du servo

/**
 * Initialisation du robot et calibration
 */
void setup() {
  #if defined(debug)
    Serial.begin(115200); //Parametres sortie série
  #endif

  //Configuration des pins
  pinMode(brEnable,OUTPUT);
  pinMode(motor1A,OUTPUT);
  pinMode(motor1B,OUTPUT);
  pinMode(motor2A,OUTPUT);
  pinMode(motor2B,OUTPUT);
  
  direction.attach(servoPin,750,2250); //Configuration du servo
  pinMode(trigPinbk,OUTPUT);
  pinMode(echoPinbk,INPUT);
  pinMode(trigPinfr,OUTPUT);
  pinMode(echoPinfr,INPUT);

  pinMode(startButtonPin,INPUT);

  //Activation des moteurs
  digitalWrite(brEnable, HIGH);
  //Dance pour calibrer le servo
  dance();
  //Remise en face des roues
  turnForward();
}

/**
 * Boucle principale
 * Arrete le robot lors de la desactivation du bouton et 
 */
void loop() {
  if(digitalRead(startButtonPin)){
    stop();
    return;
  }
  #ifdef LABMODE
    turnLoop();
  #endif
  #ifdef FORWARDMODE
    forwardLoop();
  #endif
  #ifdef SEARCHMODE
    searchLoop();
  #endif
  
  
}

/**
 * Le programme de recherche et d'attaque
 */

bool findedObstacle = false;
  
void searchLoop(){
  if(findedObstacle){
    turnForward();
    goForward(75);
    return;
  }
  turnForward();
  if(getDistance(true) <= 100){
    findedObstacle = true;
  }
  rotation();
}

void rotation(){
  int deadTimeSlot = 400;
  turnRight();
  goForward(75);
  delay(deadTimeSlot);
  stop();
  turnLeft();
  goBackward(100);
  delay(deadTimeSlot);
  stop();
}


/**
 * Le programme permettant d'aller tout droit
 */
void forwardLoop(){
  turnForward();
  if(isFree(true)){
    goForward(100);
  } else {
    stop();
  }
  delay(deadTimeRunning);
}

long lastChangedTime = millis();

#define minDistance 40

void turnLoop(){
  turnForward();
  if(getDistance(true) < minDistance){
    stop();
    turnLeft();
    if(getDistance(true) > minDistance){
      goForward(75);
      delay(500);
    } else {
      turnRight();
      if(getDistance(true) > minDistance){
        goForward(75);
        delay(500);
      } else {
        goBackward(75);
        delay(500);
      }
    }
  } else {
    goForward(75);
  }
  lastChangedTime = millis();
}

/**
 * Arreter le robot
 */
void stop(){
  digitalWrite(motor1A,LOW);
  digitalWrite(motor1B,LOW);
  digitalWrite(motor2A,LOW);
  digitalWrite(motor2B,LOW);
}

int lastTimeTurnForward = -1; //Le dernier temps pour alterner la position de la tête

/**
 * Tourner la tête du robot en face
 */
void turnForward(){
  if(lastTimeTurnForward == -1){
    lastTimeTurnForward = millis();
  }
  if((millis() - lastTimeTurnForward) >= forwardAlternationTime){
    turnTo(frontAngle+forwardAlternationAmount);
    lastTimeTurnForward = millis();
  } else {
    turnTo(frontAngle);
  }
}

/**
 * Demarrer le robot en marche arrière
 * @param spd
 *  Vitesse du robot entre 0 et 100
 */
void goForward(int spd){
  #if defined(debug)
    Serial.print("Going forward at ");
    Serial.print(spd);
    Serial.println(" %");
  #endif
  int amount = (spd*255)/100;
  analogWrite(motor1A,amount);
  digitalWrite(motor1B,LOW);
  analogWrite(motor2A,amount);
  digitalWrite(motor2B,LOW);
}

/**
 * Demarrer le robot en marche arrière
 * @param spd
 *  Vitesse du robot entre 0 et 100
 */
void goBackward(int spd){
  #if defined(debug)
    Serial.print("Going backward at ");
    Serial.print(spd);
    Serial.println(" %");
  #endif
  int amount = (spd*255)/100;
  analogWrite(motor1B,amount);
  digitalWrite(motor1A,LOW);
  analogWrite(motor2B,amount);
  digitalWrite(motor2A,LOW);
}

/**
 * Tourner la tête du robot a droite
 */
void turnRight(){
  turnTo(rightAngle);
}

/**
 * Tourner la tête du robot a gauche
 */
void turnLeft(){
  turnTo(leftAngle);
}

/**
 * Tourner la tête du robot dans un angle spécifique
 * @param angle
 *  L'angle vers lequel tourner entre 0 et 180 deg
 */
void turnTo(int angle){
  if(currentAngle != angle){
    direction.write(angle);
    delay(deadTimeRotating);
    currentAngle = angle;
  }
}

/**
 * Tourner la tête du robot dans tout les sens pour le calibrer
 */
void dance(){
  #if defined(debug)
    Serial.println("Dancing...");
  #endif
  turnForward();
  turnRight();
  turnForward();
  turnLeft();
}

/**
 * Recupèrer la distance sur un capteur
 * @param front
 *  True  -> Regarder en face
 *  False -> Regarder derrière
 */
long getDistance(bool front){
  int trigPin = 0;
  int echoPin = 0;
  float duration, 
      distance;
  if(front){
    #if defined(debug)
      Serial.println("Polling distance on front");
    #endif
    trigPin = trigPinfr;
    echoPin = echoPinfr;
  } else {
    #if defined(debug)
      Serial.println("Polling distance on back");
    #endif
    trigPin = trigPinbk;
    echoPin = echoPinbk;
  }
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  if(distance > 200){
    distance = 200;
  }
  #if defined(debug)
    Serial.print("Resulted distance ");
    Serial.print(distance);
    Serial.println(" cm");
  #endif
  return distance;
}

/**
 * Verifie que le champ est libre a droite
 */
bool isLeftFree(){
  #if defined(debug)
    Serial.println("Checking free space at left");
  #endif
  turnLeft();
  return isFree(true);
}

/**
 * Verifie que le champ est libre a gauche
 */
bool isRightFree(){
  #if defined(debug)
    Serial.println("Checking free space at right");
  #endif
  turnRight();
  return isFree(true);
}

/**
 * Verifie que le champ est libre en face
 */
bool isForwardFree(){
  #if defined(debug)
    Serial.println("Checking free space at right");
  #endif
  turnForward();
  return isFree(true);
}

/**
 * Verifie que le champ est libre en face
 */
bool isBackFree(){
  #if defined(debug)
    Serial.println("Checking free space back");
  #endif
  return isFree(false);
}

/**
 * Verifie que le champ est libre sur un capteur
 * @param front
 *  True  -> Regarder en face
 *  False -> Regarder derrière
 */
bool isFree(bool front){
  long leftDistance = getDistance(front);
  return leftDistance > freeDistance;
}
