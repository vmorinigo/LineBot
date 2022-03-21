int Speed = 30;
int Kp_Speed= 50;
int Kp_right_inside= 30;
int Kp_right_outside= 50;
int Kp_left_inside = 30;
int Kp_left_outside = 50;

int Left_Error_Inside = 0;
int Left_Error_Outside = 0;
int Right_Error_Inside = 0;
int Right_Error_Outside = 0;
int Speed_Error = 0;

// Sensor Pins
int SENS1 = A3;
int SENS2 = A2;
int SENS3 = A1;
int SENS4 = A0;
int SENS5 = A7;
int SENS_Control = A4;

int SENS_MIN[5] = {1024, 1024, 1024, 1024, 1024};
int SENS_MAX[5] = {0, 0, 0, 0, 0};
int SENS_Values[5] = {0, 0, 0, 0, 0};

// Motor Pins
int MA1 = 4;
int MA2 = 3;
int PWMA = 5;
int MB1 = 8;
int MB2 = 9;
int PWMB = 6;
int STBY = 7;

// BTN pin
int BTN = 2;

// Calibration variable
bool CAL = false;


unsigned long SysTimer;

void setup() 

{ 
  // Setup Sensors
  pinMode(SENS1, INPUT);
  pinMode(SENS2, INPUT);
  pinMode(SENS3, INPUT);
  pinMode(SENS4, INPUT);
  pinMode(SENS5, INPUT);

  // sensor control pin
  pinMode(SENS_Control, OUTPUT);

  // Turn on the sensors
  digitalWrite(SENS_Control, HIGH);

  // Motor pin setup
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enables the motors
  digitalWrite(STBY, HIGH);

  // Set motor directions
  digitalWrite(MA1, LOW);
  digitalWrite(MA2, HIGH);
  digitalWrite(MB1, HIGH);
  digitalWrite(MB2, LOW);

  //Serial port
  Serial.begin(9600);

  SysTimer = millis();
}

void loop() 
{
  if(!digitalRead(BTN))
  {
    CAL = true;
  }
  // Calibration state
  if( !CAL)
  { 
    Serial.println("CAL");

    for(int i = 0; i < 300; i++)
    {
      // Read all sensors
      SENS_Values[0] = analogRead(SENS1);
      SENS_Values[1] = analogRead(SENS2);
      SENS_Values[2] = analogRead(SENS3);
      SENS_Values[3] = analogRead(SENS4);
      SENS_Values[4] = analogRead(SENS5);

      // Look Minimum/Maximum
      for(int j = 0; j < 5; j++)
      {
        // Looking for minimum
        if(SENS_Values[j] < SENS_MIN[j])
        {
          SENS_MIN[j] = SENS_Values[j];
        }
        // Looking for maximum
        if(SENS_Values[j] > SENS_MAX[j])
        {
          SENS_MAX[j] = SENS_Values[j];
        }
      }

      // Delay so it doesn't happen instantly
      delay(200);
    }
    
    
    // Has been calibrated
    //CAL = true;
  }

  //Following the line
  if(CAL)
  {
      // Read sensors
      SENS_Values[0] = analogRead(SENS1);
      SENS_Values[1] = analogRead(SENS2);
      SENS_Values[2] = analogRead(SENS3);
      SENS_Values[3] = analogRead(SENS4);
      SENS_Values[4] = analogRead(SENS5);

      // Calculate the Error
      Left_Error_Inside = (SENS_MAX[1]-SENS_Values[1]);
      Left_Error_Outside = (SENS_MAX[0]-SENS_Values[0]);
      Right_Error_Inside = (SENS_MAX[3]-SENS_Values[3]);
      Right_Error_Outside = (SENS_MAX[4]-SENS_Values[4]);
      Speed_Error = (SENS_MAX[2] -SENS_Values[2]);

      // Normalize the Error
      Left_Error_Inside = Left_Error_Inside/(SENS_MAX[1]-SENS_MIN[1]);
      Left_Error_Outside = Left_Error_Outside/(SENS_MAX[0]-SENS_MIN[0]);
      Right_Error_Inside =Right_Error_Inside/ (SENS_MAX[3]-SENS_MIN[3]);
      Right_Error_Outside = Right_Error_Outside/(SENS_MAX[4]-SENS_MIN[4]);
      Speed_Error = Speed_Error/(SENS_MAX[2] -SENS_MIN[2]);

      // Control the motor speeds
      analogWrite(PWMB, Kp_Speed*Speed_Error + Kp_right_inside*Right_Error_Inside + Kp_right_outside*Right_Error_Outside);
      analogWrite(PWMA, Kp_Speed*Speed_Error + Kp_left_inside*Left_Error_Inside + Kp_left_outside*Left_Error_Outside);
    }
      
      Serial.println(Kp_Speed*Speed_Error + Kp_right_inside*Right_Error_Inside + Kp_right_outside*Right_Error_Outside); //+ Kp_right_inside*Right_Error_Inside + Kp_right_outside*Right_Error_Outside);
      
}
