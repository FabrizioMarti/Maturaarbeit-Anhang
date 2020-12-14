#define MOT_A1_PIN 5
#define MOT_A2_PIN 6

void setup() {
  pinMode(MOT_A1_PIN, OUTPUT);
  pinMode(MOT_A2_PIN, OUTPUT);

  digitalWrite(MOT_A1_PIN, LOW);
  digitalWrite(MOT_A2_PIN, LOW);

  Serial.begin(9600);
}

void loop() {

  digitalWrite(MOT_A1_PIN, LOW);
  analogWrite(MOT_A2_PIN, 10);

  delay(2000);
    
}
