//ver 2

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>

#define sensor1               2
#define sensor2               3
#define IN1	                  6
#define IN2	                  7
#define ENA                   8
#define TRIG                  9
#define ECHO                  10
#define MAX_SPEED             255 
#define MIN_SPEED             0
#define Waiting_Time          3000 //ms
#define SPEED                 255
#define MAX_DISTANCE          50 //cm

void door_state_checking(void* pvParameters);
void door_control(void* pvParameters);
void detecting(void* pvParameters);
void close_door(int);
void open_door(int);
void stop();
int calculate_distance();

volatile bool closed = false;
volatile bool fully_openned = false;
volatile bool detected = false;

void setup() { 
  pinMode(5, OUTPUT);         // Enable pin for sensor 1-2
  digitalWrite(5, HIGH);
  pinMode(sensor1, INPUT);    
  pinMode(sensor2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  xTaskCreate(door_state_checking, "door_state_checking", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(door_control, "door_control", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(detecting, "detecting", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  Serial.begin(9600);       // for checking the state of the door
}

void loop() {}

void door_state_checking(void* pvParameters) {
  (void) pvParameters;
  
  while(1) {
    int distance = calculate_distance();
    switch (distance) {
      case 0 ... 4:
        closed = true;
        break;
      case MAX_DISTANCE ... 1000:
        fully_openned = true;
        break;
      default:
        closed = false;
        fully_openned = false;
        break;
    }
  }
}

void detecting(void* pvParameters) {
  (void) pvParameters;
  
  while(1) {
    while(digitalRead(sensor1) == LOW || digitalRead(sensor2) == LOW) {
      detected = true;
    }
      detected = false;
  }
}

void door_control(void* pvParameters) {
  (void) pvParameters;

  while(1) {
begincycle:    
    stop();
    while(!detected);
    if(detected){
      open_door(SPEED);
      Serial.println("Door is opening");
      while(!fully_openned);
      stop();
      Serial.println("Door is opened");
    }
    while(detected){};
    if(!detected){
      if(!closed){
        vTaskDelay(Waiting_Time / portTICK_PERIOD_MS);
        close_door(SPEED);
        Serial.println("Door is closing");
        while(!closed){
            if(detected){
              Serial.println("Something is detected");
              goto begincycle;
        }};
        stop();
        Serial.println("Door is closed");
      }else {
        stop();
        continue;
        }
    }
  }
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void open_door(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);
}

void close_door(int speed) {
  speed = constrain(speed, MIN_SPEED, MAX_SPEED);
  digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);
}

int calculate_distance(){
  int distance;
  digitalWrite(TRIG, LOW);
  vTaskDelay(1 / portTICK_PERIOD_MS);
  digitalWrite(TRIG, HIGH);
  vTaskDelay(5 / portTICK_PERIOD_MS);
  digitalWrite(TRIG, LOW);

  unsigned long duration = pulseIn(ECHO, HIGH);
  distance = int(duration/2/29.412);
  return distance;
}