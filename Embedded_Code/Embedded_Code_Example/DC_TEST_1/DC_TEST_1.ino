//Motor A connected between A01 and A02

int STBY = 10; //standby

//Motor A
int PWMA = 20; // Speed Control
int AIN1 = 8; //Direction
int AIN2 = 7; //Direction

void setup() {
  // put your setup code here, to run once:
  pinMode(STBY, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  move(1, 255, 1); //motor 1, full speed, left
  // move(2, 255, 1); //motor 2, full speed, left

  delay(1000); //go for 1 second
  stop(); //stop
  // delay(250); //hold for 250ms until move again

  move(1, 128, 0); //motor 1, half speed, right
  // move(2, 128, 0); //motor 2, half speed, right

  delay(1000);
  stop();
  delay(250);
}


void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  // }else{
  //   digitalWrite(BIN1, inPin1);
  //   digitalWrite(BIN2, inPin2);
  //   analogWrite(PWMB, speed);
  }

  
}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}
