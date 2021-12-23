#include <IRremote.h>

#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11
#define carSpeed 255

////////// IR REMOTE CODES //////////
#define F 16736925  // FORWARD
#define B 16754775  // BACK
#define L 16720605  // LEFT
#define R 16761405  // RIGHT
#define S 16712445  // STOP
#define UNKNOWN_F 5316027     // FORWARD
#define UNKNOWN_B 2747854299  // BACK
#define UNKNOWN_L 1386468383  // LEFT
#define UNKNOWN_R 553536955   // RIGHT
#define UNKNOWN_S 3622325019  // STOP
#define KEY1 16738455
#define KEY2 16750695
#define KEY3 16756815
#define KEY4 16724175
#define KEY5 16718055
#define KEY6 16743045
#define KEY7 16716015
#define KEY8 16726215
#define KEY9 16734885
#define KEY0 16730805
#define KEY_HASH 16732845 // PLAY AND DELETE
#define KEY_STAR 16728765 // PLAY AND SAVE

double t3;

IRrecv irrecv(12);
decode_results results;
unsigned long val;
unsigned long preMillis;

bool playback = false;

bool save = false;


int instr[100];
int route1[100];
int route2[100];
int route3[100];
int route4[100];
int route5[100];


bool troute1 = true;
bool troute2 = true;
bool troute3 = true;
bool troute4 = true;
bool troute5 = true;


void mappy(int bulldog[]) {
  digitalWrite(A0, HIGH);
  int i = 0;
  bool byebye = false;
  bool turn = false;
  while (!byebye) {
    if (irrecv.decode(&results)){ 
      preMillis = millis();
      val = results.value;
      Serial.println(val);
      irrecv.resume();
      switch(val){
        case F: 
        case UNKNOWN_F: forward(); bulldog[i] = 5; i++; break;
        case B: 
        case UNKNOWN_B: back(); bulldog[i] = -5; i++; break;
        case L: 
        case UNKNOWN_L: left(); bulldog[i] = 1; turn = true; i++; break;
        case R: 
        case UNKNOWN_R: right(); bulldog[i] = 2; turn = true; i++; break;
        case S: 
        case UNKNOWN_S: byebye = true; break;
        default: break;
      }
    }
    else{
      if(millis() - preMillis > 500){
        stop();
        preMillis = millis();
      }
    }
  }
  digitalWrite(A0, LOW);
} 


void readMap(int stuff[]) {
  for (int i = 0; i < 100; i++) {
    readInstr(stuff[i]);
  }
}

void readInstr(int instr) {
  
  if (instr == 2) {
    right();
    right();
    right();
    right();
    right();
    right();
    right();
  }

  else if (instr == 1) {
    left();
    left();
    left();
    left();
    left();
    left();
    left();
  }

  else if (instr == 0) {
    stop();
  }

  else {
    t3 = millis();
    while ((millis() - t3) <= ((abs(instr))*133)) {
      if (instr < 0) {
        back();
      }
      else {
        forward();
      }
    }
  }
}
void forward() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void back() {
  analogWrite(ENA, carSpeed);
  analogWrite(ENB, carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() { //continuous turning
  t3 = millis(); //timer initialization
  while ((millis() - t3) <= 100) { //45 degree turn (1.75 secs = 360 degrees)
    analogWrite(ENA, carSpeed); 
    analogWrite(ENB, 32); //lower speed of right wheel
    digitalWrite(IN1, HIGH); //enable only forward motors
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void left() { //continuous turning
  t3 = millis(); //timer initialization 
  while ((millis() - t3) <= 100) { //45 degree turn (1.75 secs = 360 degrees)
    analogWrite(ENA, 32); //lower speed of left wheel
    analogWrite(ENB, carSpeed);
    digitalWrite(IN1, HIGH); //enable only forward motors
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void aaaa() { 
  t3 = millis();
  while ((millis() - t3) <= 1125) { 
    analogWrite(ENA, carSpeed); 
    analogWrite(ENB, 32);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void zero(int list[]) {
  for (int i = 0; i < 100; i++) {
    list[i] = 0;
  }
}

void equalize(int list1[], int list2[]) {
  for (int i = 0; i < 100; i++) {
    list1[i] = list2[i];
  }
}

void setup() {
  irrecv.enableIRIn();
}

void loop() {
  // put your main code here, to run repeatedly:
  while (!playback) {
    if (irrecv.decode(&results)){ 
      preMillis = millis();
      val = results.value;
      Serial.println(val);
      irrecv.resume();
      switch(val){
        case F: 
        case UNKNOWN_F: forward(); break;
        case KEY0: mappy(instr); break;
        case B: 
        case UNKNOWN_B: back(); break;
        case L: 
        case UNKNOWN_L: left(); break;
        case R: 
        case UNKNOWN_R: right(); break;
        case KEY_STAR: playback = true; save = true; break;
        case KEY_HASH: playback = true; break;
        case KEY1: readMap(route1); break;
        case KEY2: readMap(route2); break;
        case KEY3: readMap(route3); break;
        case KEY4: readMap(route4); break;
        case KEY5: readMap(route5); break;
        default: break;
      }
    }
      else{
      if(millis() - preMillis > 500){
        stop();
        preMillis = millis();
      }
    }
  }
  readMap(instr);
  if (save) {
    if (troute1) {
      equalize(route1, instr);
      troute1 = false;
    }
    else if (troute2) {
      equalize(route2, instr);
      troute2 = false;
    }
    else if (troute3) {
      equalize(route3, instr);
      troute3 = false;
    }
    else if (troute4) {
      equalize(route4, instr);
      troute4 = false;
    }
    else if (troute5) {
      equalize(route5, instr);
      troute5 = false;
    }
    else {
      Serial.println("bulldog rules the world");
    }
    zero(instr);
  }
  else {
    zero(instr);
  }
  playback = false;
  save = false;
}
