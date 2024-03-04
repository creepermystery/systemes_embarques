#include <LCD16x2.h>
#include <Wire.h>

LCD16x2 lcd;

// Pins
const int DIRECTION = 12;
const int BRAKE = 9;
const int PWM = 3;
const int ENCODER_A = 2;
const int ENCODER_B = 4;

// Constantes globales
const double cntToDeg = 360.0/(171.7877333333*24); // Rapport entre degrés et tours
const int antiBounceTime = 300;

// Variables globales
unsigned long antiBounce1 = millis();
unsigned long antiBounce2 = millis();
unsigned long antiBounce3 = millis();
unsigned long antiBounce4 = millis();
bool resA = 0;  // Etat binaire de la voie A de l'encodeur
bool resB = 0;  // Etat binaire de la voir B de l'encodeur
byte buttons = 0x0f;
volatile long countEncoder = 0; // Compteur de l'encodeur
int command = 0;
int prev_command = 0;

int modifier = 10;

void setup()
{
  Serial.begin(9600);

  pinMode(DIRECTION, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  digitalWrite(BRAKE, LOW);
  
  attachInterrupt(0, changeEncoder, CHANGE);

  Wire.begin();

  lcd.lcdClear();
  
  lcd.lcdGoToXY(1,1);
  lcd.lcdWrite("Angle:");
  
  lcd.lcdGoToXY(7,1);
  lcd.lcdWrite("0");
  
  lcd.lcdGoToXY(1,2);
  lcd.lcdWrite("+M");
  
  lcd.lcdGoToXY(5,2);
  lcd.lcdWrite("-M");
  
  lcd.lcdGoToXY(9,2);
  lcd.lcdWrite("M=");
  
  lcd.lcdGoToXY(11,2);
  lcd.lcdWrite(modifier);
  
  lcd.lcdGoToXY(15,2);
  lcd.lcdWrite("OK");
}

void changeEncoder ()
{
  // On lit l'état des deux voies de l'encodeur
  resA = digitalRead(ENCODER_A);
  resB = digitalRead(ENCODER_B);

  if (resA != resB) {
    countEncoder++;
  } else {
    countEncoder--;
  }
}

void runMotor ()
{
  if (command > countEncoder*cntToDeg)
  {
    digitalWrite(DIRECTION, HIGH);
    analogWrite(PWM, 255);
    while (command > floor(countEncoder*cntToDeg)) Serial.println(countEncoder*cntToDeg);
    analogWrite(PWM, 0);
  }
  else if (command < countEncoder*cntToDeg)
  {
    digitalWrite(DIRECTION, LOW);
    analogWrite(PWM, 255);
    while (command < floor(countEncoder*cntToDeg)) Serial.println(countEncoder*cntToDeg);
    analogWrite(PWM, 0);
  }
  else
  {
    analogWrite(PWM, 0);
  }
}

void loop()
{
  buttons = lcd.readButtons();

  if (!(buttons & 0x01) && (millis() - antiBounce1 > antiBounceTime)) // Si le bouton 1 ("UP") est enfoncé et que le debounce est passé
  {
    antiBounce1 = millis();
    if (command + modifier <= 180) command += modifier;  // On ajoute le modifier à la commande si sa valeur est différente de 180
    else command += -360 + modifier; // Si le résultat dépasse 180 deg, on effectue la soustraction pour que l'angle matche (= -180 + le restant du modifier)
  }
  if (!(buttons & 0x02) && (millis() - antiBounce2 > antiBounceTime)) // Si le bouton 2 ("DOWN") est enfoncé et que le debounce est passé
  {
    antiBounce2 = millis();
    if (command - modifier >= -180) command -= modifier; // On retire le modifier à la commande si sa valeur est différente de -180
    else command = 360 - abs(command) - modifier; // Si le résultat est sous -180 deg, on effectue l'addition pour que l'angle matche (= 180 - le restant du modifier)
  }
  if (!(buttons & 0x04) && (millis() - antiBounce3 > antiBounceTime)) // Si le bouton 3 ("modifier") est enfoncé et que le debounce est passé
  {
    antiBounce3 = millis();
    switch (modifier)
    {
        case 1:
        modifier = 10;
        break;
        case 10:
        modifier = 50;
        break;
        case 50:
        modifier = 100;
        break;
        case 100:
        modifier = 1;
        break;
        default:
        modifier = 10;
        break;
    }
    lcd.lcdGoToXY(11,2);
    lcd.lcdWrite("   ");
    lcd.lcdWrite(modifier);
  }
  if (!(buttons & 0x08) && (millis() - antiBounce4 > antiBounceTime)) // Si le bouton 4 ("OK") est enfoncé et que le debounce est passé
  {
    antiBounce4 = millis();
    runMotor();
  }
  
  if (command != prev_command)
  {
    lcd.lcdGoToXY(7,1);
    lcd.lcdWrite("    ");
    lcd.lcdWrite(command);
    prev_command = command;
  }
}
