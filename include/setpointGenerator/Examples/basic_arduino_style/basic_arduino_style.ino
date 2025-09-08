#include <Arduino.h>
#include "setpointGenerator.h"

// Four waveform generators for demonstration
setpointGenerator sineGen(0, 1023, 2.0f, WaveformType::SINE);
setpointGenerator squareGen(0, 1023, 2.0f, WaveformType::SQUARE);
setpointGenerator triangleGen(0, 1023, 2.0f, WaveformType::TRIANGLE);
setpointGenerator sawGen(0, 1023, 2.0f, WaveformType::SAWTOOTH);

void setup()
{
    Serial.begin(115200);

    // Customize square wave to spend 30% of period at min
    squareGen.setSquareDutycycle(0.3f);

    // Give triangle wave a +90° offset
    triangleGen.setPhaseOffsetDegrees(90.0f);

    // Sawtooth with 10-bit resolution for clarity
    sawGen.setDutycycleResolution(10);
}

void loop()
{
    // Get current duty values
    int sineVal     = sineGen.get();
    int squareVal   = squareGen.get();
    int triangleVal = triangleGen.get();
    int sawVal      = sawGen.get();

    // Print values in one line for Arduino Serial Plotter
    Serial.print("SINE:");
    Serial.print(sineVal);
    Serial.print("  SQUARE:");
    Serial.print(squareVal);
    Serial.print("  TRIANGLE:");
    Serial.print(triangleVal);
    Serial.print("  SAW:");
    Serial.println(sawVal);

    delay(20); // ~50 Hz sampling for nice plot
}
