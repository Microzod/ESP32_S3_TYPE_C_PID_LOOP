#pragma once
#include <map>
#include <string>
#include <vector>
#include <stdio.h>
#include <math.h>

class MeasurementReporter
{
public:
    void update(float setpoint, float feedback, float pwmOut, float dtSec)
    {
        float error = setpoint - feedback;
        float dErr = (dtSec > 0) ? (error - lastError) / dtSec : 0.0f;

        current["sp"]   = setpoint;
        current["fb"]   = feedback;
        current["err"]  = error;
        current["derr"] = dErr;
        current["pwm"]  = pwmOut;

        lastError = error;
    }

    void enable(const std::string& key)
    {
        if (std::find(active.begin(), active.end(), key) == active.end())
            active.push_back(key);
    }

    void disable(const std::string& key)
    {
        active.erase(std::remove(active.begin(), active.end(), key), active.end());
    }

    void print()
    {
        printf("MON:");
        for (size_t i = 0; i < active.size(); ++i)
        {
            const std::string& key = active[i];
            printf("%s=%.3f", key.c_str(), current[key]);
            if (i < active.size() - 1) printf(",");
        }
        printf("\n");
    }

    void enableAll()
    {
        active = { "sp", "fb", "err", "derr", "pwm" };
    }

    void disableAll()
    {
        active.clear();
    }

private:
    std::map<std::string, float> current;
    std::vector<std::string> active;
    float lastError = 0.0f;
};
