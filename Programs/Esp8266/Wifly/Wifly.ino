
#include "Wifly.h"

Wifly wifly;

void setup() {
  wifly.Init();
}

void loop() {
  wifly.Update();
}
