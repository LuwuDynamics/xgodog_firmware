#include "board/board.h"
#include "board/calibration.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "robot/types.h"
#include "robot/state.h"
#include "motor/motor.h"
#include "action/action.h"
#include "motion/gait.h"
#include "sensor/attitude.h"
#include <Adafruit_NeoPixel.h>

static void updateOutputs(void);
static void updatePowerSwitch(float sw_state_adc);
static void updateBattery(void);
static void handleLowBattery(void);
static void updateErrorIndicators();
static void updateErrors();
static void initStatusLeds(void);
static void updateStatusLeds(void);

#define GPIO_TASK_STACK_SIZE 2048
#define GPIO_TASK_PERIOD_MS  20
#define GPIO_TASK_PRIORITY   1

uint8_t digital_in_state = 0;
uint8_t analog_in_state = 0;
uint8_t anaCtrl = 0;
uint8_t digCtrl = 0;

void Init_GPIO(void) {
    /* 输入：校准键、开关状态、电池ADC、扩展数字/模拟输入 */
    pinMode(calibrate_pin, INPUT_PULLUP);
    pinMode(sw_state_pin, INPUT_PULLDOWN);
    pinMode(battery_adc, INPUT_PULLDOWN);
    pinMode(ana_input_pin, INPUT_PULLDOWN);
    pinMode(digital_input_pin, INPUT_PULLDOWN);

    pinMode(sw_ctrl_pin, OUTPUT);
    pinMode(sw_led_pin, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(ana_output_pin, OUTPUT);
    pinMode(volt_5_pin, OUTPUT);
    pinMode(digital_output_pin, OUTPUT);
    pinMode(2, OUTPUT);  /* WS2812 */

    /* 输出初始电平（swLED/swCtrl 由 switch 模块默认值） */
    digitalWrite(ana_output_pin, anaCtrl);
    digitalWrite(sw_led_pin, swLED);
    digitalWrite(sw_ctrl_pin, swCtrl);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(volt_5_pin, HIGH);
    digitalWrite(digital_output_pin, LOW);
}

/* 将各模块控制量同步到硬件引脚 */
static void updateOutputs(void) {
    digitalWrite(ana_output_pin, anaCtrl);
    digitalWrite(sw_ctrl_pin, swCtrl);
    digitalWrite(sw_led_pin, swLED);
    digitalWrite(digital_output_pin, digCtrl);
}

/* 周期任务：采样输入 → 标定/开关/电量逻辑 → 刷新输出 */
static void gpioTask(void* pvParameters) {
    (void)pvParameters;
    initStatusLeds();
    for (;;) {
        uint8_t cal_btn = (uint8_t)digitalRead(calibrate_pin);
        digital_in_state = (uint8_t)digitalRead(digital_input_pin);
        analog_in_state = (uint8_t)(analogRead(ana_input_pin) / 4097.0f * 255.0f);
        float sw_adc = (float)analogRead(sw_state_pin);
        updateStatusLeds();
        Calibration_Update(cal_btn);
        updatePowerSwitch(sw_adc);
        updateBattery();
        updateOutputs();
        updateErrors();
        vTaskDelay(pdMS_TO_TICKS(GPIO_TASK_PERIOD_MS));
    }
}

void GPIO_Task_Start(void) {
    xTaskCreate(gpioTask, "GPIO", GPIO_TASK_STACK_SIZE, NULL, GPIO_TASK_PRIORITY, NULL);
}

uint8_t swLED = 1;
uint8_t swCtrl = 1;

static uint8_t sw_button_count = 0;
static uint8_t sw_button_stage = 0;   /* 0=未触发 1=已触发长按 */
static uint16_t sw_listen_count = 0;

#define SHUTDOWN_HOLD_SAMPLES 30

static void updatePowerSwitch(float sw_state_adc) {
    /* ADC<300 视为按下，连续 30 次进入关机阶段 */
    if (sw_state_adc < 300.0f&&sw_button_stage == 0) {
        if (sw_button_count++ >= SHUTDOWN_HOLD_SAMPLES) {
            sw_button_stage = 1;
        }
    }else{
        sw_button_count = 0;
    }
    if(sw_button_stage == 0){
        if (isBattLow == 0 && isIMUInit != 0) {
            swLED = 1;
            swCtrl = 1;
            isReceiveMsg = true;
        }
        return;
    }
    sw_listen_count++;
    if(sw_listen_count%10==0){ // 20 ms GPIO polling interval
        swLED = !swLED;
    }
    if(sw_listen_count == 10){
        isReceiveMsg = false;
        Reset(2);
    }else if(sw_listen_count == 20){ // 20 ms GPIO polling interval
        Falldown();
    }else if(sw_listen_count == 90){
        EnableAllMotor(0);
    }else if(sw_listen_count == 100){
        Switch_Off();
    }
}

void Switch_Off(void) {
    swCtrl = 0;
    digitalWrite(sw_ctrl_pin, swCtrl);  /* 立即拉低，后续由 updateOutputs 维持 */
}

#define BATTERY_VOLT_BUFF_NUM 15
#define BATTERY_LOW_COUNT 200 // 20 ms GPIO polling interval

float battery = 50.0f;
uint16_t isBattLow = 0;

/* 单节电压→容量查表；Lite 与 Mini 使用各自的容量曲线，由运行时识别的机型选择。
 * 电压索引：单节 2.80V + index*0.01V（总压=2*单节）。
 * 欠压直接使用曲线低电点对应的总压：Mini 6.84V、Lite 6.70V，不依赖显示百分比。
 */
static const float kLiteBatteryCapacity[141] = {0.0016429,0.0017394,0.001846,0.0019594,0.0020728,0.0021862,0.0022995,0.0024391,0.0025809,0.0027228,0.0028666,0.0030224,0.0031783,0.0033341,0.0035078,0.003692,0.0038762,0.0040835,0.0043045,0.0045244,0.0047315,0.0049387,0.0051684,0.005438,0.0057046,0.0059622,0.0062208,0.006526,0.0068355,0.0071766,0.0075216,0.0078722,0.008298,0.0087121,0.0090791,0.0095793,0.01003,0.010536,0.011149,0.01171,0.012432,0.013262,0.01398,0.014931,0.015931,0.016998,0.018374,0.019857,0.021625,0.023247,0.025511,0.028303,0.030551,0.035077,0.039669,0.044953,0.050083,0.05601,0.063214,0.07007,0.078285,0.088135,0.096813,0.10582,0.117,0.13482,0.15248,0.17237,0.18602,0.20233,0.22087,0.23634,0.25226,0.2704,0.28586,0.30344,0.32068,0.33715,0.35582,0.37702,0.40097,0.42713,0.45238,0.47755,0.50047,0.51791,0.53419,0.54788,0.55931,0.57168,0.58308,0.59511,0.60797,0.62103,0.63435,0.64584,0.65784,0.66955,0.68119,0.69173,0.70293,0.71279,0.72294,0.73445,0.74348,0.75308,0.76306,0.77315,0.78319,0.79231,0.80211,0.81189,0.82109,0.83031,0.84011,0.84978,0.8585,0.86747,0.87656,0.88564,0.89394,0.90321,0.9114,0.91921,0.92735,0.93473,0.94217,0.94899,0.95582,0.96125,0.96668,0.97182,0.97611,0.98014,0.9838,0.98736,0.99044,0.99352,0.9966,0.99968,1};
static const float kMiniBatteryCapacity[141] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0045317,0.023414,0.042296,0.060423,0.078046,0.098187,0.11631,0.13646,0.1566,0.17674,0.19688,0.2145,0.2359,0.2573,0.27492,0.29507,0.31521,0.33535,0.35448,0.37362,0.39376,0.41138,0.43152,0.45166,0.46979,0.48993,0.50881,0.52769,0.54783,0.56697,0.5861,0.60826,0.6284,0.65106,0.67321,0.69587,0.72256,0.74924,0.77845,0.80967,0.84542,0.87462,0.8993,0.91944,0.93505,0.94612,0.95519,0.96299,0.96928,0.97412,0.97852,0.98187,0.98489,0.98752,0.98976,0.99152,0.99345,0.99457,0.99565,0.99655,0.99745,0.99802,0.99811,0.99819,0.99828,0.99836,0.99845,0.99854,0.99862,0.99871,0.99879,0.99888,0.99897,0.99905,0.99914,0.99923,0.99931,0.9994,0.99948,0.99957,0.99966,0.99974,0.99983,0.99991,1,1,1};

static float volt_buff[BATTERY_VOLT_BUFF_NUM] = {0.0f};
static float volt_avg = 0.0f;
static uint8_t volt_buff_count = 0;
static uint16_t batt_low_count = 0;
static bool batteryVoltageReady = false;
static bool batteryWasBusy = true;
static uint32_t batteryQuietSinceMs = 0;
constexpr uint32_t kBatteryRestMs = 1000;
constexpr float kBatteryStableRangeV = 0.1f;

static bool batteryMotionActive() {
    // Commands only: target changes also cover joint/foot/arm positioning
    // and delayed folding. No measured servo/wheel speed gates SOC.
    static uint16_t previousTargets[MOTOR_NUM] = {};
    bool busy = !motorInitializationComplete || isIMUInit == 0 || inCalibration
        || Action_Ctx.id != 0 || Action_Ctx.loop || Action_IsRecovering() || GaitIsActive()
        || fabsf(body.vx) > 0.001f || fabsf(body.vy) > 0.001f || fabsf(imu.yaw_speed) > 0.001f
        || body.x_T != 0 || body.y_T != 0 || body.z_T != 0
        || body.rol_T != 0 || body.pit_T != 0 || body.yaw_T != 0;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        busy |= motor[i].des_pos != previousTargets[i];
        previousTargets[i] = motor[i].des_pos;
    }
    for (const auto &w : wheel)
        busy |= w.order_vel != 0 || fabsf(w.des_vel) > 0.001f;
    return busy;
}

static float batteryVoltageThreshold(uint8_t index)
{
    /* 与旧 2 * Battery_Volt[index] 的 float 舍入逐项一致。 */
    return 2.0f * ((280.0f + (float)index) / 100.0f);
}

static uint8_t batteryCapacityIndex(float packVoltage)
{
    if (packVoltage <= batteryVoltageThreshold(0))
        return 0;
    if (!isfinite(packVoltage))
        return 140;

    int index = (int)ceilf((packVoltage - 5.6f) * 50.0f);
    if (index < 0) index = 0;
    if (index > 140) index = 140;

    /* 修正浮点边界；通常零次，最多在相邻桶间移动。 */
    while (index > 0 && packVoltage <= batteryVoltageThreshold((uint8_t)(index - 1)))
        --index;
    while (index < 140 && packVoltage > batteryVoltageThreshold((uint8_t)index))
        ++index;
    return (uint8_t)index;
}

/* 低电后分步执行：停收指令→趴下→卸载→停电机关电，并闪灯 */
static void handleLowBattery(void) {
    // Last table entries at/below the original 5% Lite / 16.6% Mini limits.
    // Voltage sampling and this debounce continue even while SOC is frozen.
    const float lowVoltage = batteryVoltageThreshold(xgo_version == XGOLITE ? 55 : 62);
    if (batteryVoltageReady && volt_avg <= lowVoltage && isBattLow == 0) {
        if (batt_low_count++ >= BATTERY_LOW_COUNT) {
            isBattLow = 1;
        }
    }
    if (!batteryVoltageReady || volt_avg > lowVoltage) {
        batt_low_count = 0;
    }

    if (isBattLow != 0) {
        isReceiveMsg = false;
        if (isBattLow == 1) {
            Reset(2);
        } else if (isBattLow == 50) {
            Falldown();
        } else if (isBattLow == 120) {
            EnableAllMotor(0);
        } else if (isBattLow >= 150) {
            Switch_Off();
        }
        if (isBattLow%5 == 0) { // 闪烁频率
            swLED = !swLED;
        }
        /* 先执行当前阶段再递增，确保阶段 1 的 Reset 不被跳过。 */
        if (isBattLow < 150)
            ++isBattLow;
    }
}

static void updateBattery(void) {
    const uint32_t nowMs = (uint32_t)millis();
    const bool busy = batteryMotionActive();
    if (busy || batteryWasBusy) batteryQuietSinceMs = nowMs;
    batteryWasBusy = busy;

    const bool isLite = xgo_version == XGOLITE;
    const float voltageGain = isLite ? 1.06f : 1.03f;
    volt_buff[volt_buff_count++] = analogRead(battery_adc) / 4096.0f * 3.3f * 3.0f * voltageGain;
    if (volt_buff_count == BATTERY_VOLT_BUFF_NUM) {
        volt_buff_count = 0;
        volt_avg = 0.0f;
        float minimumV = volt_buff[0], maximumV = volt_buff[0];
        for (const float voltage : volt_buff) {
            volt_avg += voltage / (float)BATTERY_VOLT_BUFF_NUM;
            if (voltage < minimumV) minimumV = voltage;
            if (voltage > maximumV) maximumV = voltage;
        }
        batteryVoltageReady = true;
        // The 1 s quiet interval also discards every loaded sample from this
        // 300 ms window. Stability uses this window, never an old loaded value.
        if (!busy && isBattLow == 0 && nowMs - batteryQuietSinceMs >= kBatteryRestMs
            && maximumV - minimumV <= kBatteryStableRangeV) {
            const float *capacity = isLite ? kLiteBatteryCapacity : kMiniBatteryCapacity;
            battery = 0.97f*battery + 0.03f*(100.0f * capacity[batteryCapacityIndex(volt_avg)]);
            if (battery > 100.0f) battery = 100.0f;
            else if (battery < 0.0f) battery = 0.0f;
        }
    }
    // Protection and its shutdown stages always run at the 20 ms GPIO cadence.
    handleLowBattery();
}

static void updateMotorError(){
    if(millis() - motorRevTime > 3000){
        isMotorErr = 1;
    }else{
        isMotorErr = 0;
    }
}

static void updateImuError(){
    // A motionless robot can legitimately produce identical angles forever.
    isIMUErr = imu_health::fresh((uint32_t)millis()) ? 0 : 1;
}

static void updateErrorIndicators(){
    if(isIMUErr){
        digitalWrite(LED_RED,LOW);
    }else{
        digitalWrite(LED_RED,HIGH);
    }
    if(isMotorErr){
        digitalWrite(LED_BLUE,LOW);
    }else{
        digitalWrite(LED_BLUE,HIGH);
    }
}

static void updateErrors(){
    static bool startupComplete = false;
    if (millis() > 4000) startupComplete = true;
    if(startupComplete){
        updateMotorError();
        updateImuError();
        updateErrorIndicators();
    }
}

static const uint8_t WS2812_PIN = 2;
static const uint8_t WS2812_NUM = 3;
static Adafruit_NeoPixel strip(WS2812_NUM, WS2812_PIN, NEO_GRB + NEO_KHZ800);

static void initStatusLeds(void)
{
    strip.begin();
    strip.setBrightness(128);
    strip.show();
}

static void updateStatusLeds(void)
{
    for (uint8_t i = 0; i < WS2812_NUM; ++i) {
        strip.setPixelColor(i, strip.Color(user_rgb[3*i], user_rgb[3*i+1], user_rgb[3*i+2]));
    }
    strip.show();
}
