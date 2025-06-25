#include <Servo.h>

Servo servos[5];  // 0: thumb twist, 1: thumb Y, 2: index Y, 3: middle Y, 4: little Y
const int servoPins[5] = {7, 3, 4, 5, 6};

String inputString = "";
const int MAX_INPUT_LEN = 64;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout( 50);
  for (int i = 1; i < 5; i++) {
    servos[i].attach(servoPins[i]);
    servos[i].write(40); // Neutral position
  }
   servos[0].attach(servoPins[0]);
    servos[0].write(180);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      if (inputString.length() > 0) {
        char buffer[MAX_INPUT_LEN];
        inputString.toCharArray(buffer, MAX_INPUT_LEN);

        char* tok = strtok(buffer, ",");
        int angles[5];
        int idx = 0;

        while (tok != nullptr && idx < 5) {
          angles[idx++] = atoi(tok);
          tok = strtok(nullptr, ",");
        }

        if (idx == 5) {
          for (int i = 0; i < 5; i++) {
            servos[i].write(constrain(angles[i], 0, 180));
          }
        }
      }
      inputString = "";
    } else {
      inputString += c;
    }
  }
}
