/**
 * Encoder.h
 * @author: Nick Tremaroli and Sam Schoedel
 * Contains all of the strucutes and functions needed
 * to use an encoder on the Tiva
 */
#ifndef ENCODER_H
#define ENCODER_H

#include "HAL/SSI_TIVA.h"
#include "HAL/QEI_TIVA.h"


// The different types of encoders
enum encoderTypeEnum{SSI_Encoder, QEI_Encoder};
enum encoderBrandEnum{Gurley_Encoder, Orbis_Encoder, Motor_Encoder};

/**
 * Encoder
 * Contains all of the data and features of
 * an encoder connected to the Tiva
 */
struct Encoder
{
    enum encoderTypeEnum encoderType;
    enum encoderBrandEnum encoderBrand;
    uint32_t SSIBase;   // SSI0_Base, SSI1_Base, etc.
    uint32_t QEIBase;   // QEI0_Base, QEI1_Base, etc.
    uint16_t sampleRate;
    bool enabled;

    // Data received from reading from the encoder
    uint32_t raw;
    uint32_t rawPrev;
    int32_t rawVelF;
    int32_t rawVelPrev;
    int32_t rawVel;

    float angleRads;
    float angleDegrees;
};
typedef struct Encoder Encoder;

// Constructs an encoder object
Encoder encoderConstruct(uint32_t SSIBase, uint32_t QEIBase, enum encoderTypeEnum encoderType, uint16_t sampleRate, enum encoderBrandEnum encoderBrand);
// Enable or Disable the encoder
void enableEncoder(Encoder* encoder);
void disableEncoder(Encoder* encoder);
// Read data from the abs/motor encoder
void readAbsEncoder(Encoder* encoder);
void readAbsEncoderVelocity(Encoder* encoder);
void readMotorPosition(Encoder* encoder);
void readMotorVelocity(Encoder* encoder, int32_t sample_rate, int32_t countsPerRotation);

#endif /* ENCODER_H */
