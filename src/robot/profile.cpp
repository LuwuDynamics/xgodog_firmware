#include "robot/state.h"
#include "motion/config.h"
#include <cstdio>

static void formatFirmwareVersion(uint8_t model, uint8_t wheel, char (&version)[11])
{
    const char *prefix = model == XGOMINI ? (wheel == 1 ? "MW" : "M") : "L";
    // The profile starts zero-filled. Both protocols send ten bytes.
    snprintf(version, sizeof(version), "%s%s", prefix, XGO_FIRMWARE_VERSION);
}

RobotProfile RobotProfile_Create(const uint8_t model, const uint8_t wheel){
    RobotProfile profile = {};
    BodyParam_t &bodyConfig = profile.body;
    if(model == XGOMINI){
        bodyConfig.width = 45.0f/2;
        bodyConfig.length = 150.0f/2;
        bodyConfig.L_b = 60.0f;

        bodyConfig.L_c = 75.0f;
        bodyConfig.L_d = 31.6f;
        bodyConfig.z_init = 100.0f;
        bodyConfig.NORMAL_T = SEC_2_TICK(NORMAL_PERIOD);
        bodyConfig.MID_T = SEC_2_TICK(MID_PERIOD);
        bodyConfig.SLOW_T = SEC_2_TICK(SLOW_PERIOD);
        bodyConfig.FAST_T = SEC_2_TICK(FAST_PERIOD);

        bodyConfig.xmax = 30;
        bodyConfig.xmin = -30;
        bodyConfig.ymax = 18;
        bodyConfig.ymin = -18;
        bodyConfig.zmax = 120;
        bodyConfig.zmin = 70;

        bodyConfig.rolmax = 18;
        bodyConfig.pitmax = 18;
        bodyConfig.yawmax = 18;
        bodyConfig.rolmin = -18;
        bodyConfig.pitmin = -18;
        bodyConfig.yawmin = -18;

        bodyConfig.vx_max = 33;
        bodyConfig.vx_min = -33;

        bodyConfig.vy_max = 15;
        bodyConfig.vy_min = -15;

        bodyConfig.vyaw_max = 12;
        bodyConfig.vyaw_min = -12;

        if(wheel == 1){
            bodyConfig.L_c = 68.5f;
            bodyConfig.ymax *= 0.9;
            bodyConfig.ymin *= 0.9;
            bodyConfig.zmax *= 0.95;
            bodyConfig.vx_max *= 0.8;
            bodyConfig.vx_min *= 0.8;
            bodyConfig.vy_max *= 1.2;
            bodyConfig.vy_min *= 1.2;
            bodyConfig.vyaw_max *= 1.2;
            bodyConfig.vyaw_min *= 1.2;
        }
    }else{
        bodyConfig.width = 45.0f/2;
        bodyConfig.length = 133.9f/2;
        bodyConfig.L_b = 54.9f;
        bodyConfig.L_c = 66.8f;
        bodyConfig.L_d = 28.1f;
        bodyConfig.z_init = 85.0f;
        bodyConfig.NORMAL_T = SEC_2_TICK(0.45);
        bodyConfig.MID_T = SEC_2_TICK(0.55);
        bodyConfig.SLOW_T = SEC_2_TICK(0.60);
        bodyConfig.FAST_T = SEC_2_TICK(0.36);
        bodyConfig.xmax = 20;
        bodyConfig.xmin = -20;
        bodyConfig.ymax = 16;
        bodyConfig.ymin = -16;
        bodyConfig.zmax = 110;
        bodyConfig.zmin = 65;
        bodyConfig.rolmax = 15;
        bodyConfig.pitmax = 15;
        bodyConfig.yawmax = 15;
        bodyConfig.rolmin = -15;
        bodyConfig.pitmin = -15;
        bodyConfig.yawmin = -15;

        bodyConfig.vx_max = 22;
        bodyConfig.vx_min = -14.3f;

        bodyConfig.vy_max = 12.5f;
        bodyConfig.vy_min = -12.5f;

        bodyConfig.vyaw_max = 8;
        bodyConfig.vyaw_min = -8;
    }
    profile.stepHeight = (wheel == 1) ? STEP_HEIGHT_WHEEL : STEP_HEIGHT;
    formatFirmwareVersion(model, wheel, profile.firmwareVersion);
    return profile;
}
