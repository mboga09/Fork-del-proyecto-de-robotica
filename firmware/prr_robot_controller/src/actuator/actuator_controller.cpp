#include "actuator_controller.h"

#include <Arduino.h>
#include <Servo.h>

// ---------------------------------------------------------
// Pines de actuadores principales
// ---------------------------------------------------------

#ifndef Z_SERVO_PIN
#define Z_SERVO_PIN 3
#endif

#ifndef SERVO_2_PIN
#define SERVO_2_PIN 6
#endif

#ifndef SERVO_3_PIN
#define SERVO_3_PIN 5
#endif

// ---------------------------------------------------------
// Pines ULN2003 para 28BYJ-48 herramienta
// ---------------------------------------------------------

#ifndef TOOL_IN1_PIN
#define TOOL_IN1_PIN 8
#endif

#ifndef TOOL_IN2_PIN
#define TOOL_IN2_PIN 9
#endif

#ifndef TOOL_IN3_PIN
#define TOOL_IN3_PIN 10
#endif

#ifndef TOOL_IN4_PIN
#define TOOL_IN4_PIN 11
#endif

// ---------------------------------------------------------
// Calibracion MG996R continuo para eje Z
// ---------------------------------------------------------

#define Z_STOP_US 1500

// Pulsos maximos simetricos alrededor del stop para mantener esfuerzo
// consistente en ambas direcciones. La compensacion por gravedad/friccion
// se hace en Python con z_up_speed_m_per_s y z_down_speed_m_per_s.
#define Z_UP_US   2000
#define Z_DOWN_US 1000

#define SERVO_SETTLE_MS 150

// ---------------------------------------------------------
// Calibracion herramienta 28BYJ-48
// ---------------------------------------------------------
//
// 28BYJ-48 con ULN2003 usando secuencia half-step.
// Usualmente:
//   4096 half-steps aprox 1 vuelta del eje de salida.
//
// TOOL_VOLUME_STEPS debe calibrarse experimentalmente para 1 ml.
//

#define TOOL_STEP_DELAY_MS 5

#define TOOL_VOLUME_STEPS 512L

#define TOOL_ASPIRATE_STEPS  (TOOL_VOLUME_STEPS)
#define TOOL_DISPENSE_STEPS (-TOOL_VOLUME_STEPS)

// ---------------------------------------------------------
// Objetos Servo
// ---------------------------------------------------------

static Servo zServo;
static Servo servo2;
static Servo servo3;

// ---------------------------------------------------------
// Estado stepper herramienta
// ---------------------------------------------------------

static int toolStepperPhase = 0;

static const uint8_t TOOL_SEQUENCE[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

// ---------------------------------------------------------
// Helpers stepper herramienta
// ---------------------------------------------------------

static void writeToolStepperPhase(int phase) {
    digitalWrite(TOOL_IN1_PIN, TOOL_SEQUENCE[phase][0]);
    digitalWrite(TOOL_IN2_PIN, TOOL_SEQUENCE[phase][1]);
    digitalWrite(TOOL_IN3_PIN, TOOL_SEQUENCE[phase][2]);
    digitalWrite(TOOL_IN4_PIN, TOOL_SEQUENCE[phase][3]);
}

static void releaseToolStepper() {
    digitalWrite(TOOL_IN1_PIN, LOW);
    digitalWrite(TOOL_IN2_PIN, LOW);
    digitalWrite(TOOL_IN3_PIN, LOW);
    digitalWrite(TOOL_IN4_PIN, LOW);
}

// ---------------------------------------------------------
// Inicializacion
// ---------------------------------------------------------

void initializeActuators() {
    zServo.attach(Z_SERVO_PIN);
    servo2.attach(SERVO_2_PIN);
    servo3.attach(SERVO_3_PIN);

    pinMode(TOOL_IN1_PIN, OUTPUT);
    pinMode(TOOL_IN2_PIN, OUTPUT);
    pinMode(TOOL_IN3_PIN, OUTPUT);
    pinMode(TOOL_IN4_PIN, OUTPUT);

    stopZAxis();

    servo2.write(45);
    servo3.write(90);

    releaseToolStepper();
}

// ---------------------------------------------------------
// Eje Z
// ---------------------------------------------------------

void stopZAxis() {
    zServo.writeMicroseconds(Z_STOP_US);
}

static void moveZAxis(int zDir, float zTimeS) {
    if (zDir == 0 || zTimeS <= 0.0f) {
        stopZAxis();
        return;
    }

    if (zDir > 0) {
        zServo.writeMicroseconds(Z_UP_US);
    } else {
        zServo.writeMicroseconds(Z_DOWN_US);
    }

    unsigned long durationMs = (unsigned long)(zTimeS * 1000.0f);
    delay(durationMs);

    stopZAxis();
}

// ---------------------------------------------------------
// Movimiento principal J1/J2/J3
// ---------------------------------------------------------

void moveActuators(
    int zDir,
    float zTimeS,
    float servo2Deg,
    float servo3Deg
) {
    servo2Deg = constrain(servo2Deg, 0.0f, 180.0f);
    servo3Deg = constrain(servo3Deg, 0.0f, 180.0f);

    servo2.write((int)servo2Deg);
    servo3.write((int)servo3Deg);

    delay(SERVO_SETTLE_MS);

    moveZAxis(zDir, zTimeS);
}

// ---------------------------------------------------------
// Herramienta 28BYJ-48
// ---------------------------------------------------------

void moveToolStepperSteps(long steps) {
    if (steps == 0) {
        releaseToolStepper();
        return;
    }

    int direction = steps > 0 ? 1 : -1;
    long stepCount = labs(steps);

    for (long i = 0; i < stepCount; i++) {
        toolStepperPhase += direction;

        if (toolStepperPhase > 7) {
            toolStepperPhase = 0;
        } else if (toolStepperPhase < 0) {
            toolStepperPhase = 7;
        }

        writeToolStepperPhase(toolStepperPhase);
        delay(TOOL_STEP_DELAY_MS);
    }

    releaseToolStepper();
}

void toolAspirate() {
    moveToolStepperSteps(TOOL_ASPIRATE_STEPS);
}

void toolDispense() {
    moveToolStepperSteps(TOOL_DISPENSE_STEPS);
}
