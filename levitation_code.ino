

#define TRIG_PIN 9
#define ECHO_PIN 10
#define COIL_PWM_PIN 5


float Kp = 40.0;
float Ki = 0.1;
float Kd = 15.0;


const int MAX_PWM = 255;
const int MIN_PWM = 0;
const int BIAS_PWM = 30;   


const unsigned long LOOP_MS = 30;


float setpoint = 8.0;   
float lastError = 0.0;
float integral = 0.0;
unsigned long lastTime = 0;


void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(COIL_PWM_PIN, OUTPUT);
  analogReference(DEFAULT);
  lastTime = millis();
  Serial.println("Ultrasonic levitation PID start...");
}


float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
  if (dur == 0) return -1.0; // no echo
  return dur * 0.034 / 2.0;  // cm
}

void writePWMSafe(int pwm) {
  pwm = constrain(pwm, MIN_PWM, MAX_PWM);
  analogWrite(COIL_PWM_PIN, pwm);
}


void loop() {
  unsigned long now = millis();
  if (now - lastTime < LOOP_MS) return;
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  
  float meas = readUltrasonicCM();
  if (meas < 0) {
    Serial.println("No echo, coil OFF");
    writePWMSafe(0);
    return;
  }

  
  float error = setpoint - meas;


  float P = Kp * error;
   
  integral += error * dt * Ki;
  integral = constrain(integral, -500, 500);  // anti-windup

  float derivative = (error - lastError) / dt;
  float D = Kd * derivative;

  float u = P + integral + D;
  u = P;
  lastError = error;

 
  float pwmFloat = BIAS_PWM + u;  
  int pwmOut = constrain((int)round(pwmFloat), MIN_PWM, MAX_PWM);


   if (pwmOut == MAX_PWM || pwmOut == MIN_PWM) {
     integral *= 0.95;
   }

  writePWMSafe(pwmOut);

  // --- Debug ---
  Serial.print("meas=");
  Serial.print(meas);
  Serial.print(" sp=");
  Serial.print(setpoint);
  Serial.print(" err=");
  Serial.print(error);
  Serial.print(" u=");
  Serial.print(u);
  Serial.print(" pwm=");
  Serial.println(pwmOut);
}
