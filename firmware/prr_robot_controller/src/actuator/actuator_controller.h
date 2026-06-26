#ifndef ACTUATOR_CONTROLLER_H
#define ACTUATOR_CONTROLLER_H

void initializeActuators();

void moveActuators(
    int zDir,
    float zTimeS,
    float servo2Deg,
    float servo3Deg
);

void stopZAxis();

void toolAspirate();
void toolDispense();

void moveToolStepperSteps(long steps);

#endif