#pragma once

#include "nvs_flash.h"
#include "nvs.h"
#include "nvs_handle.hpp"
#include "esp_system.h"
#include "nvsSettings_definition.h"
#include "ESP32_L298N.h"
#include "encoderEncoderPCNT.h"


controlSettings nvsSettingsStorage;

void NVS_calibrateSpeed(ESP32_L298N& l298nAccess,encoderEncoderPCNT& encoderAccess,controlSettings& settings,uint32_t runUpToSpeedDelay,uint32_t measurementTime);
void NVS_transferSettingsStruct(controlSettings& settings);
void NVS_setStoredValues(controlSettings& settings);
void NVS_getStoredValues(controlSettings& settings);
void NVS_RestartCounting();         // Exists
void NVS_writePidCeofficients();
void NVS_readPidCeofficients();
void NVS_readMotorScaleValues(ESP32_L298N& l298nAccess, encoderEncoderPCNT& encoderAccess);
void NVS_readMotorCalibrationData(ESP32_L298N& l298nAccess, encoderEncoderPCNT& encoderAccess);

void NVS_calibrateSpeed(
    ESP32_L298N& l298nAccess,
    encoderEncoderPCNT& encoderAccess,
    controlSettings& settings,
    uint32_t runUpToSpeedDelay,
    uint32_t measurementTime
)
{
    uint32_t CAL_MS = runUpToSpeedDelay;// this is simply to give me time to switch to the serial monitor
    const uint32_t  fullDuty   = l298nAccess.pwm.getResolution();

    // 1) clear any leftover counts
    encoderAccess.getCountA(true);    // Clear counter A to zero.
    encoderAccess.getCountB(true);    // Clear counter A to zero.

    // 2) spin A & B at full, wait exactly CAL_MS
    l298nAccess.pwm.setDutyCycle(l298nAccess.pwm.cmprA, fullDuty);
    l298nAccess.pwm.setDutyCycle(l298nAccess.pwm.cmprB, fullDuty);
    delay(CAL_MS);
    int cntA = encoderAccess.getCountA(true);
    int cntB = encoderAccess.getCountB(true);
    l298nAccess.pwm.setDutyCycle(l298nAccess.pwm.cmprA, 0);
    l298nAccess.pwm.setDutyCycle(l298nAccess.pwm.cmprB, 0);
    settings.vA_full = cntA * (1000.0f / float(CAL_MS));   // ticks/sec
    settings.vB_full = cntB * (1000.0f / float(CAL_MS));   // ticks/sec

    // 4) compute your common max and scales
    float vmax_common = min(settings.vA_full, settings.vB_full);
    settings.scaleA      = (settings.vA_full > 0) ? vmax_common / settings.vA_full : 1.0f;
    settings.scaleB      = (settings.vB_full > 0) ? vmax_common / settings.vB_full : 1.0f;

    printf("cntA=%d cntB=%d vA_full=%.0f ticks/s, vB_full=%.0f ticks/s\nscaleA=%.3f scaleB=%.3f\n",
           cntA, cntB, settings.vA_full, settings.vB_full, settings.scaleA, settings.scaleB);
}

void NVS_transferSettingsStruct(controlSettings& settings)
{
    nvsSettingsStorage.Kp = settings.Kp;
    nvsSettingsStorage.Ki = settings.Ki;
    nvsSettingsStorage.Kd = settings.Kd;
    nvsSettingsStorage.LPF_ALPHA = settings.LPF_ALPHA;

    nvsSettingsStorage.scaleA = settings.scaleA;
    nvsSettingsStorage.scaleB = settings.scaleB;
    nvsSettingsStorage.needToCalibrate = settings.needToCalibrate;

    nvsSettingsStorage.Q = 1000000;
}

void NVS_setStoredValues(controlSettings& settings)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased, then retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening NVS file... ");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("storage", NVS_READWRITE, &err);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS file!\n", esp_err_to_name(err));
    }
    else
    {
        printf("File opened successfully\n");

        // Read
        printf("Writing value from NVS ... ");

        // Transforming float values into uint32_t values:
        uint32_t storedKp           = uint32_t(settings.Kp * settings.Q + 0.5f);
        uint32_t storedKi           = uint32_t(settings.Ki * settings.Q + 0.5f);
        uint32_t storedKd           = uint32_t(settings.Kd * settings.Q + 0.5f);
        uint32_t storedLPF_ALPHA    = uint32_t(settings.LPF_ALPHA * settings.Q + 0.5f);

        uint32_t storedScaleA       = uint32_t(settings.scaleA * settings.Q + 0.5f);
        uint32_t storedScaleB       = uint32_t(settings.scaleB * settings.Q + 0.5f);

        uint32_t storedNeedToCalibrate = settings.needToCalibrate;
        uint32_t storedQ = settings.Q;

        // Write values:
        err = handle->set_item("NVS_Kp", storedKp);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.Kp!\n" : "DONE Storing nvsSettingsStorage.Kp-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_Ki", storedKi);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.Ki!\n" : "DONE Storing nvsSettingsStorage.Ki-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_Kd", storedKd);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.Kd!\n" : "DONE Storing nvsSettingsStorage.Kd-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_LPF_ALPHA", storedLPF_ALPHA);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.LPF_ALPHA!\n" : "DONE Storing nvsSettingsStorage.LPF_ALPHA-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_scaleA", storedScaleA);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.scaleA!\n" : "DONE Storing nvsSettingsStorage.scaleA-------\n");
        //err = handle->commit();

		err = handle->set_item("NVS_scaleB", storedScaleB);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.scaleB!\n" : "DONE Storing nvsSettingsStorage.scaleB-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_needToCalibrate", storedNeedToCalibrate);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.Q!\n" : "DONE Storing nvsSettingsStorage.Q-------\n");
        //err = handle->commit();

        err = handle->set_item("NVS_Q", storedQ);
        printf((err != ESP_OK) ? "-------FAILED Storing nvsSettingsStorage.Q!\n" : "DONE Storing nvsSettingsStorage.Q-------\n");
        //err = handle->commit();

        // Commit written value.
        printf("Committing updates in NVS ... ");
        err = handle->commit();
        printf((err != ESP_OK) ? "Committing Updates FAILED!\n" : "Committing Updates DONE\n");

        switch (err)
        {
            case ESP_OK:
                printf("... Writing Values Complete\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("... The value is not initialized yet!\n");
                break;
            default :
                printf("... Error (%s) reading!\n", esp_err_to_name(err));
        }
        //nvs_close(my_handle);
    }
}

void NVS_getStoredValues(controlSettings& settings)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased, then retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("motorCal", NVS_READWRITE, &err);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS file!\n", esp_err_to_name(err));
    }

    printf("Finished Opening NVS file.\n");
    // read
    printf("Reading value from NVS ...\n");

    uint32_t storedKp = 0;
    uint32_t storedKi = 0;
    uint32_t storedKd = 0;
    uint32_t storedLPF_ALPHA = 0;
    uint32_t storedScaleA = 0;
    uint32_t storedScaleB = 0;
    uint32_t storedNeedToCalibrate = 0;
    uint32_t storedQ = 0;

    err = handle->get_item("NVS_Kp", storedKp);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.Kp!\n" : "DONE Reading nvsSettingsStorage.Kp-------\n");
    err = handle->get_item("NVS_Ki", storedKi);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.Ki!\n" : "DONE Reading nvsSettingsStorage.Ki-------\n");
    err = handle->get_item("NVS_Kd", storedKd);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.Kd!\n" : "DONE Reading nvsSettingsStorage.Kd-------\n");
    err = handle->get_item("NVS_LPF_ALPHA", storedLPF_ALPHA);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.LPF_ALPHA!\n" : "DONE Reading nvsSettingsStorage.LPF_ALPHA-------\n");
    err = handle->get_item("NVS_scaleA", storedScaleA);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.scaleA!\n" : "DONE Reading nvsSettingsStorage.scaleA-------\n");
    err = handle->get_item("NVS_scaleB", storedScaleB);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.scaleB!\n" : "DONE Reading nvsSettingsStorage.scaleB-------\n");
    err = handle->get_item("NVS_needToCalibrate", storedNeedToCalibrate);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.needToCalibrate!\n" : "DONE Reading nvsSettingsStorage.needToCalibrate-------\n");
    err = handle->get_item("NVS_Q", storedQ);
    printf((err != ESP_OK) ? "-------FAILED Reading nvsSettingsStorage.Q!\n" : "DONE Reading nvsSettingsStorage.Q-------\n");

    settings.Kp = float(storedKp) / float(nvsSettingsStorage.Q);
    settings.Ki = float(storedKi) / float(nvsSettingsStorage.Q);
    settings.Kd = float(storedKd) / float(nvsSettingsStorage.Q);
    settings.LPF_ALPHA = float(storedLPF_ALPHA) / float(nvsSettingsStorage.Q);
    settings.scaleA = float(storedScaleA) / float(nvsSettingsStorage.Q);
    settings.scaleB = float(storedScaleB) / float(nvsSettingsStorage.Q);
    settings.needToCalibrate = storedNeedToCalibrate;
    settings.Q = storedQ;
        
    switch (err)
    {
        case ESP_OK:
            printf("... Reading Values Complete\n");
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            printf("... The value is not initialized yet!\n");
            break;
        default :
            printf("... Error (%s) reading!\n", esp_err_to_name(err));
    }

    //nvs_close(my_handle);
    
}

void NVS_RestartCounting()
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        // NVS partition was truncated and needs to be erased, then retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening NVS file... ");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("motorCal", NVS_READWRITE, &err);
    if (err != ESP_OK)
    {
        printf("Error (%s) opening NVS file!\n", esp_err_to_name(err));
    }
    else
    {
        printf("Done\n");

        // Read
        printf("Reading restart counter from NVS ... ");
        nvsSettingsStorage.restart_counter = 0; // value will default to 0, if not set yet in NVS
        err = handle->get_item("restart_counter", nvsSettingsStorage.restart_counter);
        switch (err)
        {
            case ESP_OK:
                printf("Done\n");
                printf("Restart counter = %" PRIu32 "\n", nvsSettingsStorage.restart_counter);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }

        // Write
        nvsSettingsStorage.restart_counter++;
        printf("Updating restart_counter = %u in NVS ... ", nvsSettingsStorage.restart_counter);
        err = handle->set_item("restart_counter", nvsSettingsStorage.restart_counter);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Commit written value.
        printf("Committing restart_counter = %u in NVS ... ", nvsSettingsStorage.restart_counter);
        err = handle->commit();
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        //nvs_close(my_handle);
    }
}

void NVS_writePidCeofficients()
{
    // Initialize NVS
    //ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening NVS handle... ");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("motorCal", NVS_READWRITE, &err);
    if (err != ESP_OK)
	{
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
	else
	{
        printf("Done\n");

        // to save:
        //float nvsSettingsStorage.scaleB = …;              // e.g. 0.9487…
        //uint32_t stored = uint32_t(nvsSettingsStorage.scaleB * nvsSettingsStorage.Q + 0.5f);
        //ESP_ERROR_CHECK(nvs_set_u32(h, "scaleB_i", stored));

        //uint32_t stored_in;
        //ESP_ERROR_CHECK(nvs_get_u32(h, "scaleB_i", &stored_in));
        //float nvsSettingsStorage.scaleB = float(stored_in) / float(nvsSettingsStorage.Q);
        
        // Read
        uint32_t storedKp = uint32_t(nvsSettingsStorage.Kp * nvsSettingsStorage.Q + 0.5f);
        uint32_t storedKi = uint32_t(nvsSettingsStorage.Ki * nvsSettingsStorage.Q + 0.5f);
        uint32_t storedKd = uint32_t(nvsSettingsStorage.Kd * nvsSettingsStorage.Q + 0.5f);
        uint32_t storedLPF = uint32_t(nvsSettingsStorage.LPF_ALPHA * nvsSettingsStorage.Q + 0.5f);

        err = handle->set_item("Kp_i", storedKp);
        printf((err != ESP_OK) ? "-------nvsSettingsStorage.Kp Failed!\n" : "nvsSettingsStorage.Kp Done-------\n");

		err = handle->set_item("Ki_i", storedKi);
        printf((err != ESP_OK) ? "-------nvsSettingsStorage.Ki Failed!\n" : "nvsSettingsStorage.Ki Done-------\n");

        err = handle->set_item("Kd_i", storedKd);
        printf((err != ESP_OK) ? "-------nvsSettingsStorage.Kd Failed!\n" : "nvsSettingsStorage.Kd Done-------\n");

        err = handle->set_item("LPF_i", storedLPF);
        printf((err != ESP_OK) ? "-------LPF Failed!\n" : "LPF Done-------\n");

        // Commit written value.
        printf("Committing nvsSettingsStorage.Kp, nvsSettingsStorage.Ki, nvsSettingsStorage.Kd & LPF in NVS ... ");
        err = handle->commit();
        printf((err != ESP_OK) ? "Commit Failed!\n" : "Commit Done\n");
    }
}


void NVS_readPidCeofficients()
{
    // Initialize NVS
    //ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening NVS handle... ");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("motorCal", NVS_READWRITE, &err);
    if (err != ESP_OK)
	{
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
	else
	{
        printf("Done\n");

        // to save:
        //float nvsSettingsStorage.scaleB = …;              // e.g. 0.9487…
        //uint32_t stored = uint32_t(nvsSettingsStorage.scaleB * nvsSettingsStorage.Q + 0.5f);
        //ESP_ERROR_CHECK(nvs_set_u32(h, "scaleB_i", stored));

        //uint32_t stored_in;
        //ESP_ERROR_CHECK(nvs_get_u32(h, "scaleB_i", &stored_in));
        //float nvsSettingsStorage.scaleB = float(stored_in) / float(nvsSettingsStorage.Q);

        // Read
        printf("Reading PID Coefficients from NVS ... ");
        uint32_t stored_Kp_in = 0; // value will default to 0, if not set yet in NVS
		uint32_t stored_Ki_in = 0; // value will default to 0, if not set yet in NVS
        uint32_t stored_Kd_in = 0; // value will default to 0, if not set yet in NVS
        uint32_t stored_LPF_in = 0; // value will default to 0, if not set yet in NVS
        err = handle->get_item("Kp_i", stored_Kp_in);
		err = handle->get_item("Ki_i", stored_Ki_in);
        err = handle->get_item("Kd_i", stored_Kd_in);
        err = handle->get_item("LPF_i", stored_LPF_in);

        switch (err)
		{
            case ESP_OK:
                printf("Done\n");
                nvsSettingsStorage.Kp = float(stored_Kp_in) / float(nvsSettingsStorage.Q);
                nvsSettingsStorage.Ki = float(stored_Ki_in) / float(nvsSettingsStorage.Q);
                nvsSettingsStorage.Kd = float(stored_Kd_in) / float(nvsSettingsStorage.Q);
                nvsSettingsStorage.LPF_ALPHA = float(stored_LPF_in) / float(nvsSettingsStorage.Q);
                printf("nvsSettingsStorage.Kp = %.3f nvsSettingsStorage.Ki = %.3f nvsSettingsStorage.Kd = %.3f LPF = %.3f\n", nvsSettingsStorage.Kp, nvsSettingsStorage.Ki, nvsSettingsStorage.Kd, nvsSettingsStorage.LPF_ALPHA);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("Values Not Found, run NVS_writePidCeofficients()... \n");

		        //NVS_calibrateSpeed();
                /*
                uint32_t storedKp = uint32_t(nvsSettingsStorage.Kp * nvsSettingsStorage.Q + 0.5f);
                uint32_t storedKi = uint32_t(nvsSettingsStorage.Ki * nvsSettingsStorage.Q + 0.5f);
                uint32_t storedKd = uint32_t(nvsSettingsStorage.Kd * nvsSettingsStorage.Q + 0.5f);

                err = handle->set_item("Kp_i", storedKp);
                printf((err != ESP_OK) ? "-------nvsSettingsStorage.Kp Failed!\n" : "nvsSettingsStorage.Kp Done-------\n");
                

		        err = handle->set_item("Ki_i", storedKi);
                printf((err != ESP_OK) ? "-------nvsSettingsStorage.Ki Failed!\n" : "nvsSettingsStorage.Ki Done-------\n");

                err = handle->set_item("Kd_i", storedKd);
                printf((err != ESP_OK) ? "-------nvsSettingsStorage.Kd Failed!\n" : "nvsSettingsStorage.Kd Done-------\n");

                // Commit written value.
                printf("Committing nvsSettingsStorage.Kp, nvsSettingsStorage.Ki & nvsSettingsStorage.Kd in NVS ... ");
                err = handle->commit();
                printf((err != ESP_OK) ? "Commit Failed!\n" : "Commit Done\n");
                break;
                */
                break;
        }
    }
}

void NVS_readMotorScaleValues(ESP32_L298N& l298nAccess, encoderEncoderPCNT& encoderAccess)
{
    // Initialize NVS
    //ESP_ERROR_CHECK(nvs_flash_erase());
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    // Handle will automatically close when going out of scope or when it's reset.
    std::unique_ptr<nvs::NVSHandle> handle = nvs::open_nvs_handle("motorCal", NVS_READWRITE, &err);
    if (err != ESP_OK)
	{
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    }
	else
	{
        printf("Done\n");

        // to save:
        //float nvsSettingsStorage.scaleB = …;              // e.g. 0.9487…
        //uint32_t stored = uint32_t(nvsSettingsStorage.scaleB * nvsSettingsStorage.Q + 0.5f);
        //ESP_ERROR_CHECK(nvs_set_u32(h, "scaleB_i", stored));

        //uint32_t stored_in;
        //ESP_ERROR_CHECK(nvs_get_u32(h, "scaleB_i", &stored_in));
        //float nvsSettingsStorage.scaleB = float(stored_in) / float(nvsSettingsStorage.Q);

        // Read
        printf("Reading nvsSettingsStorage.scaleB from NVS ... ");
        uint32_t stored_A_in = 0; // value will default to 0, if not set yet in NVS
		uint32_t stored_B_in = 0; // value will default to 0, if not set yet in NVS
        err = handle->get_item("scaleA_i", stored_A_in);
		err = handle->get_item("scaleB_i", stored_B_in);

        switch (err)
		{
            case ESP_OK:
                printf("Done\n");
                nvsSettingsStorage.scaleA = float(stored_A_in) / float(nvsSettingsStorage.Q);
                nvsSettingsStorage.scaleB = float(stored_B_in) / float(nvsSettingsStorage.Q);
                printf("nvsSettingsStorage.scaleA = %.3f nvsSettingsStorage.scaleB = %.3f\n", nvsSettingsStorage.scaleA, nvsSettingsStorage.scaleB);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("running calibration routine ... \n");

		        NVS_calibrateSpeed(l298nAccess, encoderAccess, nvsSettingsStorage, 5000, 5000);

                uint32_t storedA = uint32_t(nvsSettingsStorage.scaleA * nvsSettingsStorage.Q + 0.5f);
                uint32_t storedB = uint32_t(nvsSettingsStorage.scaleB * nvsSettingsStorage.Q + 0.5f);

                err = handle->set_item("scaleA_i", storedA);
                printf((err != ESP_OK) ? "-------nvsSettingsStorage.scaleA Failed!\n" : "nvsSettingsStorage.scaleA Done-------\n");
                err = handle->commit();

		        err = handle->set_item("scaleB_i", storedB);
                printf((err != ESP_OK) ? "-------nvsSettingsStorage.scaleB Failed!\n" : "nvsSettingsStorage.scaleB Done-------\n");

                // Commit written value.
                printf("Committing updates in NVS ... ");
                err = handle->commit();
                printf((err != ESP_OK) ? "Update Failed!\n" : "Update Done\n");
                break;
        }
    }
}

void NVS_readMotorCalibrationData(ESP32_L298N& l298nAccess, encoderEncoderPCNT& encoderAccess)
{
    // 1) Init NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // 2) Open your namespace
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open("motorCal", NVS_READWRITE, &h));

    nvsSettingsStorage.scaleA = 0.0f;
    nvsSettingsStorage.scaleB = 0.0f;
    size_t sz = sizeof(nvsSettingsStorage.scaleA);

    // 3) Try to read nvsSettingsStorage.scaleA
    err = nvs_get_blob(h, "nvsSettingsStorage.scaleA", &nvsSettingsStorage.scaleA, &sz);
    if (err == ESP_ERR_NVS_NOT_FOUND || err == ESP_ERR_NVS_INVALID_LENGTH)
    {
        // first-time or corrupted: run your calibration
        NVS_calibrateSpeed(l298nAccess, encoderAccess, nvsSettingsStorage, 5000, 5000);      // fills vA_full, vB_full
        //float vmax = fminf(vA_full, vB_full);
        //nvsSettingsStorage.scaleA = vmax / vA_full;
        //nvsSettingsStorage.scaleB = vmax / vB_full;

        // write them both
        ESP_ERROR_CHECK(nvs_set_blob(h, "nvsSettingsStorage.scaleA", &nvsSettingsStorage.scaleA, sizeof(nvsSettingsStorage.scaleA)));
        ESP_ERROR_CHECK(nvs_set_blob(h, "nvsSettingsStorage.scaleB", &nvsSettingsStorage.scaleB, sizeof(nvsSettingsStorage.scaleB)));
        ESP_ERROR_CHECK(nvs_commit(h));
        printf("Calibrated and saved: nvsSettingsStorage.scaleA=%.3f  nvsSettingsStorage.scaleB=%.3f\n", nvsSettingsStorage.scaleA, nvsSettingsStorage.scaleB);
    }
    else
    {
        // nvsSettingsStorage.scaleA was found and loaded, now load nvsSettingsStorage.scaleB
        ESP_ERROR_CHECK(err);     // make sure it really was OK
        sz = sizeof(nvsSettingsStorage.scaleB);
        ESP_ERROR_CHECK(nvs_get_blob(h, "nvsSettingsStorage.scaleB", &nvsSettingsStorage.scaleB, &sz));
        printf("Loaded from NVS: nvsSettingsStorage.scaleA=%.3f  nvsSettingsStorage.scaleB=%.3f\n", nvsSettingsStorage.scaleA, nvsSettingsStorage.scaleB);
    }

    nvs_close(h);

    // … now use nvsSettingsStorage.scaleA/nvsSettingsStorage.scaleB in your control loop …
}