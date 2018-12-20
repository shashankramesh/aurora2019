#define PWM 11
#define DIR 13
#define SLP 12 
#define sample_length 9
#define n_max 10000
#define enc_constant 0.03481 
#define Kp -13.41
#define Ki -282.434
#define Kd 0.0
#define w_max 15.0

//#include<Wire.h>

int nA = 1;
int nAi = 0;
boolean step_change_A = false;
unsigned long ti = 0;
float wd = 0;
int dt = 0;
int dir = 0;
int sign = 0;
float a = 0, b = 0, c = 0, ek = 0, ek_1 = 0, ek_2 = 0, pwm = 0;


void increment_A(){
  if(digitalRead(3) == HIGH)
  {
    sign = 1;
  }
  else
  {
    sign = -1;
  }
  nA++;
  if(nA > n_max)
  {
    nA = 1;
    step_change_A = true;
  }
}

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SLP, OUTPUT);
  pinMode(3, INPUT);

  digitalWrite(SLP, HIGH);

  attachInterrupt(0, increment_A, RISING);

  //Wire.begin();

  ti = micros();

  Serial.begin(115200);
  Serial.println("Enter wd");
  while(Serial.available() == 0);
  wd = Serial.parseInt();

  nAi = nA;
}

void loop() {
  if(abs(wd) < w_max)
  {
    //Change in time
    dt = micros() - ti;
    ti = micros();
  
    //Speed calculations
    int dn = nA - nAi + ((int)step_change_A*n_max);
    nAi = nA;
    if(step_change_A)
    {
      step_change_A = false;
    }
    float w = (float)sign*(float)enc_constant*(float)dn*1000000.0/(float)dt;
  
    //PID
    ek_2 = ek_1;
    ek_1 = ek;
    ek = wd - w;
    a = Kp + (Ki*(float)dt*0.000001/2.0) + (Kd*1000000.0/(float)dt);
    b = -Kp + (Ki*(float)dt*0.000001/2.0) -(2*Kd*1000000.0/(float)dt);
    c = Kd*1000000.0/(float)dt;
    
    pwm = pwm + a*ek + b*ek_1 + c*ek_2;
    
    int pwmi = pwm;
    if( pwmi < 0 )
    {
      digitalWrite(DIR, HIGH);
      pwmi = -pwmi;
    }
    else
    {
      digitalWrite(DIR, LOW);
    }
    constrain(pwmi, 0, 255);
  
    analogWrite(PWM, pwmi);
    Serial.print(w);
    Serial.print("  .......  ");
    Serial.println(dt);
    delay(20);
  }
  else
  {
    Serial.println("Saturation");
  }
}
