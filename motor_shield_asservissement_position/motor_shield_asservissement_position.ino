// Pins
const int DIRECTION  = 12;
const int BRAKE  = 9;
const int PWM  = 3;
const int ENCODER_A = 2;
const int ENCODER_B = 4;

// Constantes globales
const double cntToDeg = 360.0/(172.0*24); // Rapport entre degrés et tours

// Variables globales
bool resA = 0;  // Etat binaire de la voie A de l'encodeur
bool resB = 0;  // Etat binaire de la voir B de l'encodeur
int demand = 270; // Angle de consigne
volatile long countEncoder = 0; // Compteur de l'encodeur

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

  delay(1000);
}

void loop()
{
  if (demand > countEncoder*cntToDeg)
  {
    digitalWrite(DIRECTION, HIGH);
    analogWrite(PWM, 255);
    while (demand > floor(countEncoder*cntToDeg)) Serial.println(countEncoder*cntToDeg);
    analogWrite(PWM, 0);
  }
  else if (demand < countEncoder*cntToDeg)
  {
    digitalWrite(DIRECTION, LOW);
    analogWrite(PWM, 255);
    while (demand < floor(countEncoder*cntToDeg)) Serial.println(countEncoder*cntToDeg);
    analogWrite(PWM, 0);
  }
  else
  {
    analogWrite(PWM, 0);
  }
}
