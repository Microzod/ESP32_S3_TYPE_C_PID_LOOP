#include <encoderEncoderPCNT.h>

static const int        quadrature_encoder_pin_A    = 10;
static const int        quadrature_encoder_pin_B    = 9;
static const int        motor_encoder_pin_A         = 15;
static const int        motor_encoder_pin_B         = 16;

encoderEncoderPCNT encoder(quadrature_encoder_pin_A, quadrature_encoder_pin_B, motor_encoder_pin_A, motor_encoder_pin_B);

void setup()
{
    encoder.position();
    encoder.setPosition(pos);
    encoder.zero();
	
	int quadCount = encoder.getCount(false);
	int encA = encoder.getCountA(false);
	int encB = encoder.getCountB(false);
}

void loop()
{
    
}