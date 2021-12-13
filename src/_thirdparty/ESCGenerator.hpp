#pragma once
#include "HardwareGenerator/pca9685.h"
#define PWM_MAX_US 2000.f
#define PWM_MIN_US 1000.f
#define PWM_RANGE 1000.f
#define PCA9685_PWM_MAX 3000.f
#define PCA9685_PWM_MIN 2300.f
#define PCA9685_RANGE 4096.f
#define PCA9685_DEFAULT_DEVICE "/dev/i2c-0"
#define PCA9685_DEFAULT_ADDRESS 0x40
#define PCA9685_ALL_PIN 16
#define ONESHOT125_RANGE_LOW 125.f
#define ONESHOT125_RANGE_HIGH 250.f

enum GeneratorType
{
    Hardware_PWM,
    Software_PWM,
    Hardware_ONESHOT125,
    Software_ONESHOT125,
};

class ESCGenerator
{
public:
    // Frequency is not update frequency
    inline ESCGenerator(GeneratorType generator, const char *device, uint8_t addr, int Frequency);
    // Range 1000us - 2000us, Will Caculate auto
    inline void ESCUpdate(int ID, int Range);

    inline void ESCClear(int ID);

    inline ~ESCGenerator()
    {
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        close(GeneratorFD);
    };

private:
    GeneratorType Generator;
    int PlFrequency = 0;
    int GeneratorFD = -1;

    int ThrotteMin;
    int ThrotteMax;
    int ThrotteRange;
    int Hardware_OffState = 0;
};

ESCGenerator::ESCGenerator(GeneratorType generator, const char *device, uint8_t addr, int Frequency)
{
    Generator = generator;
    PlFrequency = Frequency;
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
    {
        GeneratorFD = pca9685Setup(device,
                                   addr,
                                   PlFrequency);
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        ThrotteMin = PCA9685_PWM_MIN;
        ThrotteMax = PCA9685_PWM_MAX;
        ThrotteRange = ThrotteMax - ThrotteMin;
    }
    break;

    case GeneratorType::Hardware_ONESHOT125:
    {
        GeneratorFD = pca9685Setup(device,
                                   addr,
                                   PlFrequency);
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        int dt = 1.f / (float)PlFrequency * 1000000.f;
        ThrotteMin = PCA9685_RANGE * (ONESHOT125_RANGE_LOW / dt);
        ThrotteMax = PCA9685_RANGE * (ONESHOT125_RANGE_HIGH / dt);
        ThrotteRange = ThrotteMax - ThrotteMin;
    }
    break;

    case GeneratorType::Software_PWM:
        throw 0;
        break;

    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;

    default:
        throw 0;
        break;
    }

    if (GeneratorFD < 0)
        throw - 1;
}

void ESCGenerator::ESCUpdate(int ID, int Range)
{
    int Output = ThrotteMin + ((((float)Range - PWM_MIN_US) / PWM_RANGE) * (float)ThrotteRange);
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
        pca9685PWMWriteS(GeneratorFD, ID, Output);
        break;

    case GeneratorType::Hardware_ONESHOT125:
        pca9685PWMWriteS(GeneratorFD, ID, Output);
        break;

    case GeneratorType::Software_PWM:
        throw 0;
        break;

    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;

    default:
        throw 0;
        break;
    }
};

void ESCGenerator::ESCClear(int ID)
{
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
        pca9685PWMWrite(GeneratorFD, ID, 0, 0);
        break;

    case GeneratorType::Hardware_ONESHOT125:
        pca9685PWMWrite(GeneratorFD, ID, 0, 0);
        break;

    case GeneratorType::Software_PWM:
        throw 0;
        break;

    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;

    default:
        throw 0;
        break;
    }
}