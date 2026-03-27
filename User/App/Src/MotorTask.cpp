#include "MotorTask.h"
#include "Foc.h"

FOC motor;

void MotorTask()
{
  motor.Tick();
}
