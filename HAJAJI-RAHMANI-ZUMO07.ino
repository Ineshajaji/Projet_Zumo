/*This demo requires a Zumo 32U4
* Configuration :
  5 capteur de lignes donc verifier que les 2 cavaliers sur la carte capteurs sont bien sur la position interieure
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include "gyro_use.h"
#include <math.h>

const uint16_t maxSpeed = 400;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4ProximitySensors proxSensors;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

void turncalibrate(short nbangle45)
{
  int16_t speedgauche = 160 * (nbangle45 > 0 ? -1 : 1);
  motors.setSpeeds(speedgauche, -speedgauche);
  turnSensorReset();
  int32_t angleC = turnAngle45 * nbangle45;
  while ((nbangle45 >= 0 ? turnAngle < angleC : turnAngle >= angleC))
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  turnSensorUpdate();
}

void calibrateSensors2()
{
  delay(100);
  lineSensors.calibrate();
  turncalibrate(2);
  turncalibrate(-4);
  turncalibrate(2);
}

void setup()
{
  Wire.begin();
  Serial1.begin(38400);
  buttonB.waitForButton();
  Serial1.println("BONJOUR !");
  delay(800);
  lineSensors.initFiveSensors();
  proxSensors.initThreeSensors();
  if (!imu.init())
  {
    ledRed(1);
    while (1);
  }
  imu.enableDefault();
  imu.configureForTurnSensing();
  turnSensorSetup();
  calibrateSensors2();
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying());
  encoders.getCountsAndResetRight();
  encoders.getCountsAndResetLeft();
}

//***********variables globales*************//
//variables pid
long ticksTotal = 0;
int16_t positionl;
int16_t lastError = 0;
int16_t lastDerivative = 0;
const int16_t baseSpeedFast = 350;
const int16_t baseSpeedMid  = 320;
const int16_t baseSpeedSlow = 290;
const int16_t kP = 7;
const int16_t kD = 26;
const int16_t kDiv = 32;
const int16_t deadband = 35;
int32_t lastErrorGyro = 0;
int32_t capRef = 0;
uint16_t count = 0;
//variables contournement et obstacle
bool contournementLance = false;
int cumulLeft = 0, cumulRight = 0;
const char endLineSong[]     = "L16 cdefedc8r8";
const char noObstacleSong[]  = "L16 g8a8b8>c8r8";
bool playedEndLine = false, obstacleReady = false, obstaclePasse = false;
bool playedIntersection = false, parkingFinal = false;
uint8_t postObstacleCount = 0;
uint8_t parkingStep = 0;
unsigned long chronoContournement = 0, tempsDebutParcours = 0, tempsFinalParcours = 0;
bool parcoursEnCours = false;
long ll, lastll;
long distdroit, distgauche;
uint8_t incomingByte, flaggo, automgest;
//*********************FIN*********************//

void PID()
{
  int16_t error = positionl;
  if (abs(error) < deadband) error = 0;
  int16_t derivative = error - lastError;
  lastError = error;
  derivative = (int16_t)((derivative + (int32_t)lastDerivative * 2) / 3);
  lastDerivative = derivative;
  int32_t speedDifference = (int32_t)error * 7 + (int32_t)derivative * 26;
  speedDifference /= 32;
  speedDifference = constrain(speedDifference, -(int32_t)260, (int32_t)260);
  int16_t curve = abs(error) + abs(derivative) * 2;
  int16_t currentBaseSpeed;
  if (curve < 180)
    currentBaseSpeed = baseSpeedFast;
  else if (curve < 700)
    currentBaseSpeed = baseSpeedMid;
  else
    currentBaseSpeed = baseSpeedSlow;
  int16_t leftSpeed = currentBaseSpeed + speedDifference;
  int16_t rightSpeed = currentBaseSpeed - speedDifference;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);
}

void PID_gyro()
{
  int32_t error = turnAngle - capRef;
  static int32_t errorFiltre = 0;
  errorFiltre = (errorFiltre * 7 + error) / 8;
  int16_t correction = errorFiltre / 50000;
  if (correction > -1 && correction < 1)
    correction = 0;
  correction = constrain(correction, -18, 18);
  int16_t baseGyroSpeed = 330;
  int16_t leftSpeed  = baseGyroSpeed + correction;
  int16_t rightSpeed = baseGyroSpeed - correction;
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
  motors.setSpeeds(leftSpeed, rightSpeed);
}

unsigned char Analyse(int *p)
{
  unsigned char i;
  unsigned char ret = 0;

  for (i = 0; i < 5; i++)
  {
    ret = ret << 1;
    ret = ret + (p[i] > (lineSensors.calibratedMinimumOn[i] * 2 +
                         lineSensors.calibratedMaximumOn[i]) / 3 ? 1 : 0);
  }
  return ret;
}

bool obstacleDetected()
{
  proxSensors.read();
  uint8_t left = proxSensors.countsFrontWithLeftLeds();
  uint8_t right = proxSensors.countsFrontWithRightLeds();
  const uint8_t seuil = 6;
  return (left >= seuil) && (right >= seuil);
}

bool zoneNoireDetectee(unsigned char sens_stat)
{
  return (sens_stat == 0x1F) ||
         (obstaclePasse && postObstacleCount >= 3 &&
          (sens_stat >= 0x1E || sens_stat == 0x0F));
}

bool confirmerCarreFinal()
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  while (encoders.getCountsRight() < 150)
  {
    motors.setSpeeds(140, 140);
  }
  motors.setSpeeds(0, 0);
  delay(20);
  lineSensors.read(lineSensorValues);
  unsigned char testApres = Analyse((int*)lineSensorValues);
  return (testApres == 0x1F);
}

void terminerParcoursEtParker()
{
  encoders.getCountsAndResetRight();
  while (encoders.getCountsRight() < 500)
  {
    motors.setSpeeds(200, 200);
  }
  motors.setSpeeds(0, 0);
  if (parcoursEnCours)
  {
    tempsFinalParcours = millis();
    parcoursEnCours = false;
    float tempsTotal = (tempsFinalParcours - tempsDebutParcours) / 1000.0;
    Serial1.print("Temps de parcours : ");
    Serial1.print(tempsTotal, 3);
    Serial1.println(" secondes");
  }
  turnSensorReset();
  motors.setSpeeds(-150, 150);
  while ((int32_t)turnAngle < (turnAngle45 * 4))
  {
    turnSensorUpdate();
  }
  motors.setSpeeds(0, 0);
  if (!playedEndLine)
  {
    buzzer.play("v15 L16 cdegc5");
    playedEndLine = true;
  }
  flaggo = 0;
  obstacleReady = false;
  count = 0;
  parkingFinal = true;
  automgest = 99;
  while (buzzer.isPlaying());
}

void loop()
{
  if (Serial1.available())
  {
    incomingByte = Serial1.read();

    switch (incomingByte & 0xff)
    {
      case 'c':
        calibrateSensors2();
        while ((Analyse((int*)lineSensorValues) & 0x1F) == 0)
        {
          lineSensors.read(lineSensorValues);
          motors.setSpeeds(-80, 80);
          delay(10);
        }
        motors.setSpeeds(0, 0);
        buzzer.play("L16 cdegreg4");
        while (buzzer.isPlaying());
        incomingByte = 0;
        break;

      case 'g':
        flaggo = 1;
        break;

      case 's':
        tempsFinalParcours = millis();
        flaggo = 0;
        automgest = 0;
        parcoursEnCours = false;
        motors.setSpeeds(0, 0);
        if (tempsDebutParcours > 0) {
            float tempsEcoule = (tempsFinalParcours - tempsDebutParcours) / 1000.0;
            Serial1.print("Temps final ('s') : ");
            Serial1.print(tempsEcoule, 3);
            Serial1.println(" s");
        }
        break;

      case 'x':
        Serial1.println(readBatteryMillivolts());
        incomingByte = 0;
        break;

      case 'd':
        Serial1.print("G=");
        Serial1.print(distgauche);
        Serial1.print("  D=");
        Serial1.println(distdroit);
        break;

      case 't':
        if (tempsDebutParcours == 0) {
          Serial1.println("Le robot n'a pas encore demarre.");
        } else {
          unsigned long duree = millis() - tempsDebutParcours;
          Serial1.print("Chrono : ");
          Serial1.print(duree);
          Serial1.println("ms");
        }
        break;

      case 'r':
        if (automgest == 0) {
          ticksTotal = 0;
          lastError = 0;
          motors.setSpeeds(120, 120);
          delay(300); // 300 millisecondes = 0.3 seconde
          motors.setSpeeds(0, 0);
          automgest = 10;
        }
        break;
      default:
        break;
    }
  }

  turnSensorUpdate();
  positionl = lineSensors.readLine(lineSensorValues) - 2000;
  unsigned char sens_stat = Analyse((int*)lineSensorValues);

  ll = micros();

  if ((ll - lastll) >= 10000)
  {
    lastll = ll;
    distdroit += encoders.getCountsAndResetRight();
    distgauche += encoders.getCountsAndResetLeft();
    uint8_t proxR = proxSensors.countsFrontWithRightLeds();
    uint8_t proxL = proxSensors.countsFrontWithLeftLeds();
    if (automgest > 0) {
    }
    switch (automgest)
    {
      case 0:
          if (flaggo == 1)
          {
              proxSensors.read();
              if (proxSensors.countsFrontWithRightLeds() < 5 && proxSensors.countsFrontWithLeftLeds() < 5)
              {
                  buzzer.play("L32 cde");
                  delay(200);
                  encoders.getCountsAndResetLeft();
                  encoders.getCountsAndResetRight();
                  tempsDebutParcours = millis();
                  parcoursEnCours = true;
                  obstacleReady = false;
                  playedEndLine = false;
                  playedIntersection = false;
                  obstaclePasse = false;
                  parkingFinal = false;
                  postObstacleCount = 0;
                  parkingStep = 0;
                  count = 0;
                  automgest = 1;
              }
          }
          break;

      case 1:
          if (zoneNoireDetectee(sens_stat))
          {
              if (confirmerCarreFinal())
              {
                  terminerParcoursEtParker();
                  return;
              }
              lineSensors.read(lineSensorValues);
              positionl = lineSensors.readLine(lineSensorValues) - 2000;
              sens_stat = Analyse((int*)lineSensorValues);

              if (obstaclePasse && !playedIntersection && postObstacleCount < 3)
              {
                  buzzer.play("L16 c8d8e8g8");
                  playedIntersection = true;
                  postObstacleCount++;
              }
          }
          count = 0;
          if (((sens_stat & 0x0E) == 0x0E) && (sens_stat != 0x1F))
          {
              if (!playedIntersection)
              {
                  if (!obstaclePasse || postObstacleCount < 3)
                  {
                      buzzer.play("L16 c8d8e8g8");
                      if (obstaclePasse) postObstacleCount++;
                  }
                  playedIntersection = true;
              }
              if (!obstaclePasse)
              {
                  obstacleReady = true;
              }
          }
          else
          {
              playedIntersection = false;
          }

          if (obstacleReady)
          {
              proxSensors.read();
              if (proxSensors.countsFrontWithLeftLeds() >= 5 && proxSensors.countsFrontWithRightLeds() >= 5)
              {
                  motors.setSpeeds(0, 0);
                  buzzer.play("L16 a8g8f8");
                  obstacleReady = false;
                  chronoContournement = millis();
                  automgest = 2;
                  Serial1.println("OBSTACLE");
                  return;
              }
          }
          if (sens_stat == 0x00) PID_gyro();
          else { PID(); capRef = turnAngle; }
          break;      
      case 2: 
           //une fois on detecte la boite on avance un petit temps
          motors.setSpeeds(250, 250); // AVANCE tout droit
          if (millis() - chronoContournement >= 700) 
          {
                motors.setSpeeds(0, 0);
                delay(80);
                turnSensorReset();
                contournementLance = false;
                automgest = 3;
                return;
          }
          break;
      case 3: 
          if (!contournementLance) 
          {
                turnSensorReset();
                // Pivot sur place 90° à droite
                motors.setSpeeds(250, -250); 
                contournementLance = true;
          }
          turnSensorUpdate();     
          if (turnAngle <= -(2 * turnAngle45)) 
          {
                // On prépare la recherche de ligne en courbe serrée
                // On baisse la vitesse globale et on augmente l'écart
                motors.setSpeeds(120, 280); 
                contournementLance = false;
                automgest = 4;
          }
          break;

      case 4: 
          // On maintient la courbe de recherche
          motors.setSpeeds(120, 280); 

          if ((sens_stat & 0x0E) != 0) // Dès qu'on croise la ligne
          { 
                motors.setSpeeds(0, 0); 
                delay(100);
                turnSensorReset();          
                obstacleReady = false;
                contournementLance = false; 
                automgest = 5; // On passe au micro-ajustement
          }
          break; 

      case 5:
          if (!contournementLance) 
          {
                turnSensorReset();
                // On pivote à droite pour bien se remettre face à la ligne
                motors.setSpeeds(150, -150); 
                contournementLance = true;
          }          
          turnSensorUpdate();             
          // On tourne de 64° environ (1.44 * 45)
          if (turnAngle <= -(turnAngle45 * 1.44)) 
          {
                motors.setSpeeds(0, 0);  
                delay(100); 
                // On nettoie tout pour que le Case 1 ne voit pas d'ancien obstacle
                lastError = 0;
                lastDerivative = 0;
                obstacleReady = false;
                playedIntersection = false;
                obstaclePasse = true;
                postObstacleCount = 0;
                contournementLance = false;                
                automgest = 1; // On force le passage au suivi de ligne                
                encoders.getCountsAndResetLeft();
                encoders.getCountsAndResetRight();
                buzzer.play("v10 L32 g"); 
                return; 
          }
          break;
      case 6:
          if (parkingStep == 0)
          {
              motors.setSpeeds(0, 0);
              encoders.getCountsAndResetRight();
              parkingStep = 1;
          }
          else if (parkingStep == 1)
          {
              motors.setSpeeds(170, 170);
              if (encoders.getCountsRight() >= 420)
              {
                  motors.setSpeeds(0, 0);
                  turnSensorReset();
                  parkingStep = 2;
              }
          }
          else if (parkingStep == 2)
          {
              motors.setSpeeds(-170, 170);
              turnSensorUpdate();
              if ((int32_t)turnAngle >= (turnAngle45 * 4))
              {
                  motors.setSpeeds(0, 0);
                  if (!playedEndLine) { buzzer.play(endLineSong); playedEndLine = true; }
                  flaggo = 0;
                  count = 0;
                  parkingStep = 0;
                  automgest = 7;
              }
          }
          break;
      case 7:
          motors.setSpeeds(0, 0);
          break;
            //phase finale: appui sur r   
      case 10:
          if ((sens_stat & 0x0E) == 0x0E || sens_stat == 0x1F)
          {
              motors.setSpeeds(0, 0);
              delay(150);
              chronoContournement = millis();
              automgest = 11;

          }
          else
          {
              if (sens_stat == 0x00) PID_gyro(); else PID();
          }
          break;
      case 11:
          motors.setSpeeds(200, 200);
          if (millis() - chronoContournement >= 400)
          {
              motors.setSpeeds(0, 0);
              delay(150);
              turnSensorReset();
              automgest = 12;
          }
          break;
      case 12:
          motors.setSpeeds(200, -200);
          turnSensorUpdate();
          if (turnAngle <= -(2 * turnAngle45))
          {
              motors.setSpeeds(0, 0);
              delay(150);
              chronoContournement = millis();
              automgest = 13;
          }
          break;
      case 13:
          motors.setSpeeds(250, 250);
          if (millis() - chronoContournement >= 1000)
          {
              motors.setSpeeds(0, 0);
              buzzer.play("! L16 V8 c e g > c");
              automgest = 0;
          }
          break;
          //parking final
      
      default:
          break;
    }
  }
}
