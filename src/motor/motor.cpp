#include "motor/motor.h"
#include "robot/state.h"
#include "robot/math.h"
#include "board/calibration.h"
#include <Arduino.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

namespace {
constexpr uint8_t kBigEndian = 0;
constexpr uint8_t kLittleEndian = 1;
constexpr uint8_t kWheelVelocityAddress = 0x20;
constexpr uint8_t kPositionGainAddress = 21;
constexpr uint8_t k009PositionKp = 32;
constexpr uint8_t k009PositionKd = 16;
constexpr uint8_t kLite009PositionKp = 15;
constexpr uint8_t kLite009PositionKd = 15;
constexpr uint8_t k009MaxTorqueAddress = 0x10;
constexpr uint8_t k009EepromLockAddress = 0x30;
constexpr uint8_t kGripperId = 51;
constexpr uint16_t kGripperTorqueLimit = 400;

bool usesLite009Drive(uint8_t version) {
    return xgo_version == XGOLITE && version == MOTOR_009;
}
}

namespace {
// Wire codec shared by servo and wheel transactions.
// The 26 supported motors use at most seven bytes per sync-write record.
namespace motor_bus {
static const uint16_t kSyncDataMax = 26u * 7u;
static const uint16_t kPayloadMax = kSyncDataMax + 2u;
static const uint16_t kFrameMax = kPayloadMax + 6u;
static_assert(kFrameMax <= 255u, "Motor Bus lengths must fit the deployed byte API");

inline void packWord(uint8_t *out, uint16_t value, uint8_t endian)
{
    const uint8_t low = endian == 0 ? 1 : 0;
    out[low] = (uint8_t)value;
    out[1u - low] = (uint8_t)(value >> 8);
}

inline uint16_t unpackWord(const uint8_t *data, uint8_t endian)
{
    const uint8_t low = endian == 0 ? 1 : 0;
    return (uint16_t)((uint16_t)data[low] | ((uint16_t)data[1u - low] << 8));
}

// Prefix and payload are separate spans so sync writes do not need an extra
// concatenation buffer. Both command forms share bounds and checksum handling.
inline uint8_t encodeFrame(uint8_t *frame, uint8_t id, uint8_t instruction,
                          const uint8_t *prefix, uint8_t prefixLength,
                          const uint8_t *payload, uint8_t payloadLength)
{
    const uint16_t length = (uint16_t)prefixLength + payloadLength;
    if (length > kPayloadMax || (prefixLength != 0 && prefix == 0) ||
        (payloadLength != 0 && payload == 0))
        return 0;

    frame[0] = 0xff;
    frame[1] = 0xff;
    frame[2] = id;
    frame[3] = (uint8_t)(length + 2u);
    frame[4] = instruction;
    uint8_t checksum = (uint8_t)(id + frame[3] + instruction);
    uint8_t offset = 5;
    for (uint8_t i = 0; i < prefixLength; ++i) {
        frame[offset++] = prefix[i];
        checksum = (uint8_t)(checksum + prefix[i]);
    }
    for (uint8_t i = 0; i < payloadLength; ++i) {
        frame[offset++] = payload[i];
        checksum = (uint8_t)(checksum + payload[i]);
    }
    frame[offset++] = (uint8_t)~checksum;
    return offset;
}

class FeedbackParser {
public:
    void reset()
    {
        state_ = HeaderFirst;
        index_ = 0;
        length_ = 0;
        checksum_ = 0;
    }

    // Returns exactly one completed feedback record, with chunk boundaries
    // invisible to the state machine. Accepted lengths match deployed motors.
    bool consume(uint8_t byte, uint8_t endian, MotorState_t &result)
    {
        switch (state_) {
        case HeaderFirst:
            if (byte == 0xff) state_ = HeaderSecond;
            break;
        case HeaderSecond:
            state_ = byte == 0xff ? Id : HeaderFirst;
            break;
        case Id:
            id_ = byte;
            checksum_ = byte;
            state_ = Length;
            break;
        case Length:
            checksum_ = (uint8_t)(checksum_ + byte);
            if (byte == 8 || byte == 9 || byte == 11) {
                length_ = byte;
                state_ = Error;
            } else {
                state_ = HeaderFirst;
            }
            break;
        case Error:
            checksum_ = (uint8_t)(checksum_ + byte);
            index_ = 0;
            state_ = Data;
            break;
        case Data:
            data_[index_++] = byte;
            checksum_ = (uint8_t)(checksum_ + byte);
            if (index_ == length_ - 2u) state_ = Checksum;
            break;
        case Checksum:
            state_ = HeaderFirst;
            if ((uint8_t)~checksum_ != byte) return false;
            decode(endian, result);
            return true;
        }
        return false;
    }

private:
    enum State : uint8_t { HeaderFirst, HeaderSecond, Id, Length, Error, Data, Checksum };
    State state_ = HeaderFirst;
    uint8_t id_ = 0;
    uint8_t length_ = 0;
    uint8_t index_ = 0;
    uint8_t checksum_ = 0;
    uint8_t data_[9] = {};

    void decode(uint8_t endian, MotorState_t &result) const
    {
        result = {};
        result.id = id_;
        result.order_len = length_;
        const uint8_t *data = data_ + (length_ == 8 ? 0 : 3);
        if (length_ == 9) {
            // Wheel odometry is always little endian. Unsigned shifts avoid
            // signed overflow for negative positions; convert the sign explicitly.
            const uint32_t raw = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                                 ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
            result.position = raw <= 0x7fffffffu ? (int)raw : -1 - (int)(~raw);
            return;
        }
        const uint16_t position = unpackWord(data, endian);
        result.position = position <= 0x7fffu ? (int)position : (int)position - 65536;
        const uint16_t velocity = unpackWord(data + 2, endian);
        const short speed = (short)(velocity & 0x7fffu);
        const bool wheelStatus = length_ == 11 &&
            (id_ == 10 || id_ == 20 || id_ == 30 || id_ == 40);
        // Wheel speed is signed int16; servos use sign + magnitude.
        // Treating 0xffec (-20 rpm) as sign + magnitude gives -32748 rpm.
        result.velocity = wheelStatus
            ? (short)(velocity <= 0x7fffu ? (int)velocity : (int)velocity - 65536)
            : (velocity & 0x8000u) ? (short)-speed : speed;
        const uint16_t torque = unpackWord(data + 4, endian);
        const short magnitude = (short)(torque & 0x03ffu);
        result.torque = (torque & 0x0400u) ? (short)-magnitude : magnitude;
    }
};
} // namespace motor_bus

}

static motor_bus::FeedbackParser parser;
static StaticSemaphore_t busMutexStorage;
static SemaphoreHandle_t busMutex = NULL;
static uint32_t busQuietUntilMs = 0;
static bool busQuietPending = false;

// Frames may also be submitted by the host's raw enable command. A recursive
// lock lets a complete request/reply retain ownership across its frame write.
class BusLock {
public:
    BusLock() {
        configASSERT(busMutex != NULL);
        xSemaphoreTakeRecursive(busMutex, portMAX_DELAY);
    }
    ~BusLock() { xSemaphoreGiveRecursive(busMutex); }
    BusLock(const BusLock &) = delete;
    BusLock &operator=(const BusLock &) = delete;
};

static void waitForBusQuiet()
{
    if (!busQuietPending) return;
    while (static_cast<int32_t>(millis() - busQuietUntilMs) < 0)
        vTaskDelay(pdMS_TO_TICKS(1));
    busQuietPending = false;
}

MotorConfig_t g_motorConfig = {
    .M_N = 4095.0,
    .M_A = 360.0,
    .M_V = 4095.0,
    .ENDIAN = kLittleEndian,
    .ADDR_POS = 42,
    .ADDR_POS_READ = 0x38,
    .ADDR_ENABLE = 40,
};

// All frames use the same lock. Read transactions retain it until the response
// is complete, so raw host writes cannot interleave with a motor reply.
static void writeFrame(uint8_t id, uint8_t instruction,
                       const uint8_t *prefix, uint8_t prefixLength,
                       const uint8_t *payload, uint8_t payloadLength)
{
    uint8_t frame[motor_bus::kFrameMax];
    const uint8_t length = motor_bus::encodeFrame(frame, id, instruction,
                                                 prefix, prefixLength, payload, payloadLength);
    if (length == 0) return;
    const BusLock lock;
    waitForBusQuiet();
    Serial2.write(frame, length);
}

static void packWord(uint8_t *pDat, uint16_t nVal) {
    motor_bus::packWord(pDat, nVal, g_motorConfig.ENDIAN);
}

static void writeInstruction(uint8_t ID, uint8_t Fun, const uint8_t *nDat, uint8_t nLen)
{
    writeFrame(ID, Fun, NULL, 0, nDat, nLen);
}

static void writeByte(uint8_t ID, uint8_t addr, uint8_t data){
    uint8_t payload[2];
    payload[0] = addr;
    payload[1] = data;
    writeInstruction(ID, 0x03, payload, 2);
}

static void writeWord(uint8_t ID, uint8_t addr, uint16_t data){
    uint8_t payload[3];
    payload[0] = addr;
    packWord(payload+1, data);
    writeInstruction(ID, 0x03, payload, 3);
}

static void readMotor(uint8_t ID, uint8_t addr, uint8_t nLen){
    uint8_t payload[2];
    payload[0] = addr;
    payload[1] = nLen;
    writeInstruction(ID, 0x02, payload, 2);
}

static void writeMotorEnable(uint8_t ID, uint8_t state, uint8_t version){ // 修改PD参数变成带阻尼模式
    uint8_t payload[3];
    if (usesLite009Drive(version)) {
        writeByte(ID, g_motorConfig.ADDR_ENABLE, state == 1 ? 1 : 0);
        return;
    }
    if(version != MOTOR_RC05P){
        payload[0] = kPositionGainAddress;
        if(state == 1){
            payload[1] = version == MOTOR_3032 ? 24 : k009PositionKp;
            payload[2] = version == MOTOR_3032 ? 0 : k009PositionKd;
        }else{
            if(version==0x01){
                payload[1] = 0;
                payload[2] = 0;
            }else{
                payload[1] = 0;
                payload[2] = 30;
            }
        }
        writeInstruction(ID, 0x03, payload, 3);
    }else{
        writeByte(ID, g_motorConfig.ADDR_ENABLE, state);
    }
}

void Motor_WriteEnable(uint8_t id, uint8_t state, uint8_t version)
{
    const BusLock lock;
    writeMotorEnable(id, state, version);
    Serial2.flush(true);
    // The caller keeps the original single-write/no-state-change semantics.
    // Defer the next transaction until the write acknowledgement has cleared.
    busQuietUntilMs = millis() + 5u;
    busQuietPending = true;
}

static void syncWrite(uint8_t addr, uint8_t nLen, uint8_t *nDat, uint8_t totalLen){
    if (nDat == NULL || totalLen > motor_bus::kSyncDataMax)
        return;
    const uint8_t prefix[] = {addr, nLen};
    writeFrame(0xFE, 0x83, prefix, sizeof(prefix), nDat, totalLen);
}


static void UpdateMotorState(MotorState_t state);
static void SetTorLimit(uint8_t id, uint16_t limit);
static void waitWithFeedback(uint32_t milliseconds);
static void readFeedback(uint8_t id, uint8_t address, uint8_t size,
                         uint8_t responseLength, uint32_t cycleStart);

static const uint8_t kStartupLegSpeed = 60;

static uint32_t s_enablePending[8] = {0};
static uint32_t s_enableDesired[8] = {0};
static portMUX_TYPE s_enableMux = portMUX_INITIALIZER_UNLOCKED;
// Enable writes retain the 20 ms settling delay and three 5 ms retries without
// suspending target transmission and feedback for the entire bus.
static struct {
    uint32_t deadlineMs = 0;
    uint8_t id = 0;
    uint8_t state = 0;
    uint8_t writesLeft = 0;
    bool active = false;
} enableRequest;
static uint8_t s_midMoveNextMotor = MOTOR_NUM;
static uint32_t s_midMoveDeadlineMs = 0;
static portMUX_TYPE s_midMoveMux = portMUX_INITIALIZER_UNLOCKED;

static void ResetMotorEnableRequests(void)
{
    portENTER_CRITICAL(&s_enableMux);
    for (uint8_t i = 0; i < 8; ++i) {
        s_enablePending[i] = 0;
        s_enableDesired[i] = 0;
    }
    portEXIT_CRITICAL(&s_enableMux);
    enableRequest = {};
}

void RequestMotorEnable(uint8_t id, uint8_t state)
{
    const uint8_t word = id >> 5;
    const uint32_t mask = 1u << (id & 31u);
    portENTER_CRITICAL(&s_enableMux);
    if (state != 0)
        s_enableDesired[word] |= mask;
    else
        s_enableDesired[word] &= ~mask;
    s_enablePending[word] |= mask;
    portEXIT_CRITICAL(&s_enableMux);
}

static bool TakeMotorEnableRequest(uint8_t *id, uint8_t *state)
{
    if (id == NULL || state == NULL)
        return false;
    bool found = false;
    portENTER_CRITICAL(&s_enableMux);
    for (uint8_t word = 0; word < 8; ++word) {
        const uint32_t pending = s_enablePending[word];
        if (pending == 0)
            continue;
        const uint8_t bit = (uint8_t)__builtin_ctz(pending);
        const uint32_t mask = 1u << bit;
        *id = (uint8_t)(word * 32u + bit);
        *state = (s_enableDesired[word] & mask) != 0;
        s_enablePending[word] &= ~mask;
        found = true;
        break;
    }
    portEXIT_CRITICAL(&s_enableMux);
    return found;
}

static const uint8_t kMotorIds[MOTOR_NUM] = {11,12,13,
                        21,22,23,
                        31,32,33,
                        41,42,43,
                        51,52,53,54};

static const float kMotorDirections[MOTOR_NUM] = {
     -1, 1, 1,
      1,-1,-1,
      1,-1, 1,
     -1, 1,-1,
      1, -1, 1,-1
} ;

static const MotorConfig_t kMotorConfig3032 = {
    .M_N = 4095.0,
    .M_A = 360.0,
    .M_V = 4095.0,
    .ENDIAN = kLittleEndian,
    .ADDR_POS = 42,
    .ADDR_POS_READ = 0x38,
    .ADDR_ENABLE = 40,
};

static const MotorConfig_t kMotorConfig009 = {
    .M_N = 1023.0,
    .M_A = 300.0,
    .M_V = 1023.0,
    .ENDIAN = kBigEndian,
    .ADDR_POS = 42,
    .ADDR_POS_READ = 0x38,
    .ADDR_ENABLE = 40,
};

static const MotorConfig_t kMotorConfigRc05p = {
    .M_N = 1023.0,
    .M_A = 360.0,
    .M_V = 360.0,
    .ENDIAN = kLittleEndian,
    .ADDR_POS = 0x1E,
    .ADDR_POS_READ = 0x24,
    .ADDR_ENABLE = 0x18,
};

static const float kMiniJointLimits[MOTOR_NUM][2] = {
    {-70,60},{-70,120},{-35,35},
    {-70,60},{-70,120},{-35,35},
    {-70,60},{-70,120},{-35,35},
    {-70,60},{-70,120},{-35,35},
    {-80, 80},{-55, 96},{-93, 86},{-180, 180}
};

static const float kLiteJointLimits[MOTOR_NUM][2] = {
    {-70,120},{-85,95},{-40,55},
    {-70,120},{-85,95},{-40,55},
    {-70,120},{-85,95},{-40,55},
    {-70,120},{-85,95},{-40,55},
    {-70, 70},{-70, 75},{-93, 75},{-180, 180}
};

static void Init_Motor_ZeroPos(){
    uint16_t zero_pos[MOTOR_NUM];
    FlashZeroPosRead(zero_pos, MOTOR_NUM);
    for(int i=0;i<MOTOR_NUM;i++){
        if(zero_pos[i]<10||zero_pos[i]>g_motorConfig.M_N){
            zero_pos[i] = g_motorConfig.M_N/2;
        }
        if(i<12 && i%3 == 0){
            if(hasWheel == 1){
                zero_pos[i] -= motor[i].sign*5.0*g_motorConfig.M_N/g_motorConfig.M_A;
            }else{
                zero_pos[i] -= motor[i].sign*11.8*g_motorConfig.M_N/g_motorConfig.M_A;
            }
        }
        motor[i].zero_pos = zero_pos[i];
    }
}

void SetLegMotorSpeed(uint8_t data){
    uint16_t speed = (uint16_t)(data/255.0*g_motorConfig.M_V);
    for(int i=0;i<12;i++){
        motor[i].speed = speed;
    }
    motor[12].speed = 0;
}

void SetArmMotorSpeed(uint8_t data){
    uint16_t speed = (uint16_t)(data/255.0*g_motorConfig.M_V);
    motor[12].speed = 0;
    motor[13].speed = speed;
    motor[14].speed = speed;
    motor[15].speed = speed;
}

static void Init_ExMotor(){
    for(int i=0;i<10;i++){
        ex_motor[i].id = 91+i;
        ex_motor[i].des_pos = g_motorConfig.M_N/2;
        ex_motor[i].zero_pos = g_motorConfig.M_N/2;
        ex_motor[i].max_pos = g_motorConfig.M_N;
        ex_motor[i].min_pos = 0;
        ex_motor[i].enable = 1;
    }
}

static void Init_Wheel(){
    for(int i=0;i<4;i++){
        wheel[i].id = 10*(i+1);
        wheel[i].fb_pos = 0;
        wheel[i].order_vel = 0;
        wheel[i].fb_vel = 0;
        wheel[i].odom = 0;
        wheel[i].enable = 1;
        wheel[i].sign = -sign.nppn[i];
    }
}

void Init_Motor(){
    // setup calls Init_Motor before creating any communication task. Subsequent
    // model-discovery resets reuse the same statically allocated lock.
    if (busMutex == NULL)
        busMutex = xSemaphoreCreateRecursiveMutexStatic(&busMutexStorage);
    configASSERT(busMutex != NULL);
    motorInitializationComplete = false;
    ResetMotorEnableRequests();

    const float (*angle_limit)[2] = (xgo_version == XGOMINI) ? kMiniJointLimits : kLiteJointLimits;

    parser.reset();
    for(int i=0;i<MOTOR_NUM;i++){
        motor[i].id = kMotorIds[i];
        motor[i].des_pos = g_motorConfig.M_N/2;
        motor[i].zero_pos = g_motorConfig.M_N/2 ;
        motor[i].max_pos = g_motorConfig.M_N;
        motor[i].min_pos = 0;
        motor[i].fb_pos = g_motorConfig.M_N/2;
        motor[i].fb_vel = 0;
        motor[i].fb_tor = 0;
        motor[i].enable = 1;
        motor[i].speed = g_motorConfig.M_V/2;
        motor[i].des_angle = 0;
        motor[i].fb_angle = 0;
        motor[i].max_angle = angle_limit[i][1];
        motor[i].min_angle = angle_limit[i][0];
        motor[i].sign = kMotorDirections[i];
    }
    motor[15].fb_pos = 0;
    motor[12].speed = 0;

    Init_Motor_ZeroPos();
    Init_ExMotor();
    Init_Wheel();
    SetTorLimit(kGripperId, kGripperTorqueLimit);
    SetLegMotorSpeed(kStartupLegSpeed);
    SetArmMotorSpeed(60);
}

void UpdateMotorDesPos(void){
    for(int i=0;i<MOTOR_NUM;i++){
        if(motor[i].enable == 0){
            motor[i].des_angle = motor[i].fb_angle;
        }
        motor[i].des_angle = Saturation(motor[i].des_angle, motor[i].max_angle, motor[i].min_angle);
        const float offset = motor[i].des_angle * g_motorConfig.M_N / g_motorConfig.M_A * motor[i].sign;
        if (!std::isfinite(offset)) continue; // Keep the last encoded target on invalid input.
        // Truncate the signed offset before adding calibration, preserving the
        // deployed quantization. Clamp before narrowing: unsigned wrap would
        // send an under-range target to the opposite mechanical endpoint.
        const float bounded = Saturation(offset, 65535.f, -65535.f);
        const int32_t raw = static_cast<int32_t>(bounded) + motor[i].zero_pos;
        const int32_t low = motor[i].min_pos < motor[i].max_pos ? motor[i].min_pos : motor[i].max_pos;
        const int32_t high = motor[i].min_pos < motor[i].max_pos ? motor[i].max_pos : motor[i].min_pos;
        motor[i].des_pos = static_cast<uint16_t>(raw < low ? low : raw > high ? high : raw);
    }
}

static void UpdateMotorState(MotorState_t state){
    uint8_t version = state.position & 0xFF;
    // 初次上电读取版本号，根据版本号初始化电机
    if(motor_version == MOTOR_UNKNOWN){
        if(version == 0x09){
            motor_version = MOTOR_3032;
            g_motorConfig = kMotorConfig3032;
            xgo_version = XGOMINI;
        }else if(version == 0x05){
            motor_version = MOTOR_009;
            g_motorConfig = kMotorConfig009;
            xgo_version = XGOLITE;
        }else if(version == 0x0B){
            motor_version = MOTOR_RC05P;
            g_motorConfig = kMotorConfigRc05p;
            xgo_version = XGOMINI;
        }
        if(motor_version != MOTOR_UNKNOWN){
            Init_Motor();
            Init_Config();
        }
    }

    for(int i=0;i<MOTOR_NUM;i++){
        if(motor[i].id == state.id){
            motorRevTime = millis();
            motor[i].fb_pos = state.position;
            motor[i].fb_vel = state.velocity * g_motorConfig.M_A / 360.0 * motor[i].sign;
            motor[i].fb_tor = state.torque * motor[i].sign;
            motor[i].fb_angle = (float)(state.position-motor[i].zero_pos) * g_motorConfig.M_A / g_motorConfig.M_N * motor[i].sign;
            break;
        }
    }

    for(int i=0;i<4;i++){
        if(wheel[i].id == state.id){
            if(state.order_len == 0x0B){
                wheel[i].fb_pos = state.position;
                wheel[i].fb_vel = (float)(state.velocity*0.022*PI/30.0*wheel[i].sign);
            }else{
                wheel[i].odom = int(state.position/16.0*wheel[i].sign);
            }
            break;
        }
    }

}

static void SendWheelVel(){
    uint8_t order[3 + 4 * 3];
    uint8_t count = 0;
    order[0] = 0xAA;
    order[1] = 0;
    order[2] = 0;
    for(int i=0;i<4;i++){
        if(wheel[i].enable == 1){
			order[3+count*3] = wheel[i].id;
			order[3+count*3+1] = (uint8_t)(wheel[i].order_vel & 0xFF);
			order[3+count*3+2] = (uint8_t)(wheel[i].order_vel >> 8);
            count++;
        }
    }
    if(count == 0){
        return;
    }
    syncWrite(kWheelVelocityAddress, 2, order, (count+1)*3);
}

// 周期写 0x18=0x01，防止轮子欠压失能；每 0.2s 只发一个轮，轮询四轮
static void KeepWheelEnable(void){
    static uint32_t last_ms = 0;
    static uint8_t idx = 0;
    uint32_t now = millis();
    if (now - last_ms < 200)
        return;
    last_ms = now;

    const BusLock lock;
    writeByte(wheel[idx].id, 0x18, 0x01);
    Serial2.flush(true);
    busQuietUntilMs = millis() + 3u;
    busQuietPending = true;
    if (++idx >= 4)
        idx = 0;
}

// One record layout is used by legs, arm and extension motors. RC05P omits
// the intermediate word used by 009/3032.
static uint8_t *packMotorTarget(uint8_t *out, const Motor_t &target,
                                uint16_t speed, bool compact)
{
    *out++ = target.id;
    packWord(out, target.des_pos);
    out += 2;
    if (!compact) {
        packWord(out, 0);
        out += 2;
    }
    packWord(out, speed);
    return out + 2;
}

static void SendMotorDesPos(){
    const bool lite009 = usesLite009Drive(motor_version);
    if (lite009 && inCalibration) return;

    static const uint8_t kMotorRecordMax = 7;
    uint8_t order[(MOTOR_NUM + 10) * kMotorRecordMax];
    uint8_t *next = order;
    const bool compact = motor_version == MOTOR_RC05P;
    // Snapshot readiness once: every leg record in this frame uses the same
    // startup policy, even if the IMU task completes while we pack the frame.
    const bool imuInitializing = isIMUInit == 0;
    const uint16_t startupLegSpeed = imuInitializing
        ? (uint16_t)(kStartupLegSpeed / 255.0 * g_motorConfig.M_V) : 0;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        // A position write can re-engage a released 009 servo.
        if (motor[i].enable != 1 && (compact || lite009)) continue;
        const uint16_t speed = (imuInitializing && i < 12)
            ? startupLegSpeed : motor[i].speed;
        next = packMotorTarget(next, motor[i], speed, compact);
    }
    for (int i = 0; i < 10; ++i) {
        if (ex_motor[i].enable != 1 && compact) continue;
        next = packMotorTarget(next, ex_motor[i], 0, compact);
    }
    if (next != order)
        syncWrite(g_motorConfig.ADDR_POS, compact ? 4 : 6, order, (uint8_t)(next - order));
}

void EnableAllMotor(uint8_t state){
    RequestMotorEnable(0xFE, state);
}

static void configureLite009Servo(uint8_t id) {
    const BusLock lock;
    writeByte(id, k009EepromLockAddress, 0);
    waitWithFeedback(5);
    if (id == kGripperId) {
        writeWord(id, k009MaxTorqueAddress, kGripperTorqueLimit);
        waitWithFeedback(5);
    }
    const uint8_t gains[] = {kPositionGainAddress, kLite009PositionKp, kLite009PositionKd};
    for (int retry = 0; retry < 3; ++retry) {
        writeInstruction(id, 0x03, gains, sizeof(gains));
        waitWithFeedback(5);
    }
    writeByte(id, k009EepromLockAddress, 1);
    waitWithFeedback(5);
}

// Restore drive for each 009/3032 leg and arm ID at startup.
static void restoreLegArmDrive(void){
    if ((motor_version != MOTOR_009 && motor_version != MOTOR_3032) || inCalibration)
        return;
    const bool lite009 = usesLite009Drive(motor_version);
    for (int i = 0; i < MOTOR_NUM; i++) {
        const uint8_t id = motor[i].id;
        if (lite009) {
            if (inCalibration) return;
            configureLite009Servo(id);
        }
        for (int j = 0; j < 3; j++) {
            const BusLock lock;
            writeMotorEnable(id, 1, motor_version);
            waitWithFeedback(5);
        }
        if (motor_version == MOTOR_3032) {
            uint8_t deadband[] = {26, 3, 3}; // Start address, then both directions.
            const BusLock lock;
            writeInstruction(motor[i].id, 0x03, deadband, sizeof(deadband));
            waitWithFeedback(5);
        }
    }
}

static void SetTorLimit(uint8_t ID,uint16_t limit){
    if(motor_version != MOTOR_3032){
        return;
    }
    const BusLock lock;
    writeWord(ID, 0x30, limit);
    Serial2.flush(true);
    busQuietUntilMs = millis() + 5u;
    busQuietPending = true;
}

void MotorMoveToMid(void){
    xgo_mode = XGO_MODE_MOTOR;
    const uint32_t now = millis();
    portENTER_CRITICAL(&s_midMoveMux);
    motor[0].des_angle = 0;
    s_midMoveNextMotor = 1;
    s_midMoveDeadlineMs = now + 300u;
    portEXIT_CRITICAL(&s_midMoveMux);
}

void UpdateMotorMoveToMid(void){
    const uint32_t now = millis();

    portENTER_CRITICAL(&s_midMoveMux);
    if(s_midMoveNextMotor < MOTOR_NUM &&
       (int32_t)(now - s_midMoveDeadlineMs) >= 0){
        motor[s_midMoveNextMotor++].des_angle = 0;
        s_midMoveDeadlineMs = now + 300u;
    }
    portEXIT_CRITICAL(&s_midMoveMux);
}

static void ReadMotorVersion(void){
    readFeedback(11, 0x03, 6, 8, micros());
}

static void probeRotationAndWheels(void){
    readFeedback(10, 0x24, 6, 11, micros());
    waitWithFeedback(10);
    readFeedback(54, 0x38, 6, 8, micros());
}

static void SyncReadMotor(void)
{
    uint8_t payload[2 + MOTOR_NUM];
    payload[0] = 0x38;
    payload[1] = 0x06;
    for (uint8_t i = 0; i < MOTOR_NUM; ++i) payload[2 + i] = motor[i].id;
    writeInstruction(0xFE, 0x82, payload, sizeof(payload));
}

namespace {
// Leave 3 ms of scheduling margin for the 20 ms bus refresh requirement.
constexpr uint32_t kCycleMs = 17;
constexpr uint32_t kFeedbackBudgetUs = 18000;
constexpr uint32_t kReplyTimeoutUs = 6000;
constexpr uint32_t kRunningReplyTimeoutUs = 2500;
constexpr uint32_t kSyncReplyTimeoutUs = 6000;
// One blocking read can span a tick; include request airtime as well. A new
// request starts only when its complete reply window fits before the deadline.
constexpr uint32_t kReplyGuardUs = 1200;
constexpr uint32_t kTaskStack = 3072;
constexpr uint8_t kTaskPriority = 4;
constexpr uint8_t kTaskCore = 1;
static_assert(pdMS_TO_TICKS(kCycleMs) > 0, "RTOS tick must support the motor cycle");
}

// Block for the first byte, then fetch the available chunk in one driver call.
// Preserve unconsumed bytes when a chunk contains several complete replies.
static uint8_t feedbackBytes[64];
static size_t feedbackRead = 0, feedbackSize = 0;
static bool receiveFeedback(MotorState_t &state)
{
    if (feedbackRead == feedbackSize) {
        feedbackRead = 0;
        feedbackSize = Serial2.readBytes(feedbackBytes, 1);
        if (feedbackSize == 0) return false;
        const size_t available = min((size_t)Serial2.available(), sizeof(feedbackBytes) - 1);
        if (available) feedbackSize += Serial2.readBytes(feedbackBytes + 1, available);
    }
    while (feedbackRead < feedbackSize) {
        if (parser.consume(feedbackBytes[feedbackRead++], g_motorConfig.ENDIAN, state)) {
            UpdateMotorState(state);
            return true;
        }
    }
    return false;
}

static void waitWithFeedback(uint32_t milliseconds)
{
    const uint32_t startedAt = millis();
    MotorState_t state;
    while (millis() - startedAt < milliseconds) receiveFeedback(state);
}

static void serviceEnableRequests(uint32_t cycleStart)
{
    const uint32_t now = millis();
    if (enableRequest.active) {
        if (static_cast<int32_t>(now - enableRequest.deadlineMs) < 0) return;
        if (enableRequest.writesLeft != 0) {
            if (micros() - cycleStart > kFeedbackBudgetUs - 6000u) return;
            const BusLock lock;
            waitForBusQuiet();
            if (micros() - cycleStart > kFeedbackBudgetUs - 6000u) return;
            writeMotorEnable(enableRequest.id, enableRequest.state, motor_version);
            --enableRequest.writesLeft;
            enableRequest.deadlineMs = now + 5u;
            // Preserve the deployed write/reply quiet time. Feedback continues
            // to be consumed, and this bounded wait fits inside the cycle.
            waitWithFeedback(5);
            return;
        }
        enableRequest.active = false;
    }

    uint8_t id, state;
    if (!TakeMotorEnableRequest(&id, &state)) return;
    bool found = id == 0xfe;
    for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
        if (id == 0xfe || motor[i].id == id) {
            motor[i].enable = state;
            found = true;
            if (id != 0xfe) break;
        }
    }
    if (!found) return;
    enableRequest.id = id;
    enableRequest.state = state;
    enableRequest.writesLeft = 3;
    enableRequest.deadlineMs = now + 20u;
    enableRequest.active = true;
}

static void drainFeedback()
{
    MotorState_t state;
    // A fixed bound also handles noise without monopolizing the motor task.
    for (uint16_t i = 0; i < 256 && (feedbackRead < feedbackSize || Serial2.available() > 0); ++i)
        receiveFeedback(state);
}

static bool feedbackTimeLeft(uint32_t cycleStart, uint32_t timeoutUs)
{
    return micros() - cycleStart + timeoutUs + kReplyGuardUs < kFeedbackBudgetUs;
}

static void readFeedback(uint8_t id, uint8_t address, uint8_t size,
                         uint8_t responseLength, uint32_t cycleStart)
{
    const BusLock lock;
    waitForBusQuiet();
    const uint32_t timeoutUs = motorInitializationComplete ? kRunningReplyTimeoutUs : kReplyTimeoutUs;
    if (!feedbackTimeLeft(cycleStart, timeoutUs)) return;
    drainFeedback();
    readMotor(id, address, size);
    Serial2.flush(true);
    const uint32_t startedAt = micros();
    MotorState_t state;
    while (micros() - startedAt < timeoutUs) {
        if (receiveFeedback(state) && state.id == id && state.order_len == responseLength)
            return;
    }
}

static void readSynchronousFeedback(uint32_t cycleStart)
{
    const BusLock lock;
    waitForBusQuiet();
    if (!feedbackTimeLeft(cycleStart, kSyncReplyTimeoutUs)) return;
    drainFeedback();
    SyncReadMotor();
    Serial2.flush(true);
    const uint32_t startedAt = micros();
    uint16_t received = 0;
    MotorState_t state;
    while (received != 0xffffu && micros() - startedAt < kSyncReplyTimeoutUs) {
        if (!receiveFeedback(state) || state.order_len != 8) continue;
        for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
            if (state.id == motor[i].id) {
                received |= static_cast<uint16_t>(1u << i);
                break;
            }
        }
    }
}

static void pollFeedback(uint32_t cycleStart)
{
    static uint8_t next = 0;
    const bool synchronous = motor_version == MOTOR_3032;
    if (synchronous && feedbackTimeLeft(cycleStart, kSyncReplyTimeoutUs))
        readSynchronousFeedback(cycleStart);

    // Interleave four servo reads with each wheel's state and odometry. If a
    // missing/slow device exhausts this cycle's budget, resume here next cycle
    // so neither the last servos nor the wheels are starved.
    const uint8_t count = synchronous ? (hasWheel ? 8 : 0) : (hasWheel ? 24 : MOTOR_NUM);
    const uint32_t timeoutUs = kRunningReplyTimeoutUs;
    for (uint8_t i = 0; i < count && feedbackTimeLeft(cycleStart, timeoutUs); ++i) {
        if (next >= count) next = 0;
        const uint8_t slot = next++;
        const bool isWheel = synchronous || (hasWheel && slot % 6 >= 4);
        if (isWheel) {
            const uint8_t wheelIndex = synchronous ? slot / 2 : slot / 6;
            const bool odometry = synchronous ? slot % 2 != 0 : slot % 6 == 5;
            readFeedback(wheel[wheelIndex].id, odometry ? 0x47 : 0x24,
                         odometry ? 4 : 6, odometry ? 9 : 11, cycleStart);
        } else {
            const uint8_t motorIndex = hasWheel ? (slot / 6) * 4 + slot % 6 : slot;
            readFeedback(motor[motorIndex].id, g_motorConfig.ADDR_POS_READ, 6, 8, cycleStart);
        }
        serviceEnableRequests(cycleStart);
    }
}

static void motorTask(void *)
{
    Serial2.setTimeout(1);
    while (motor_version == MOTOR_UNKNOWN) {
        Serial2.begin(1000000, SERIAL_8N1, 13, 14);
        ReadMotorVersion();
        waitWithFeedback(100);
        Serial2.begin(500000, SERIAL_8N1, 13, 14);
        ReadMotorVersion();
        waitWithFeedback(100);
    }

    if (motor_version == MOTOR_RC05P || motor_version == MOTOR_3032) {
        Serial2.begin(1000000, SERIAL_8N1, 13, 14);
        for (uint8_t i = 0; i < 10; ++i) {
            waitWithFeedback(10);
            probeRotationAndWheels();
        }
        if (wheel[0].fb_pos != 0) hasWheel = 1;
    } else {
        Serial2.begin(500000, SERIAL_8N1, 13, 14);
    }

    waitWithFeedback(100);
    Init_Config();
    Init_Motor();
    // Settle targets before restoring drive after an MCU/calibration restart.
    waitWithFeedback(300);
    restoreLegArmDrive();
    motorInitializationComplete = true;

    const TickType_t period = pdMS_TO_TICKS(kCycleMs);
    TickType_t lastWake = xTaskGetTickCount();
    for (;;) {
        const uint32_t cycleStart = micros();
        serviceEnableRequests(cycleStart);
        SendMotorDesPos();
        if (hasWheel) {
            SendWheelVel();
        }
        Serial2.flush(true);
        pollFeedback(cycleStart);
        // Keep-alive acknowledgements settle in the idle part of the cycle.
        if (hasWheel) KeepWheelEnable();

        // Absolute scheduling, with no catch-up burst after an overrun.
        const TickType_t now = xTaskGetTickCount();
        if (static_cast<TickType_t>(now - lastWake) >= period) lastWake = now;
        MotorState_t state;
        while (static_cast<TickType_t>(xTaskGetTickCount() - lastWake) + 1u < period) {
            serviceEnableRequests(cycleStart);
            receiveFeedback(state);
        }
        vTaskDelayUntil(&lastWake, period);
    }
}

void MotorTasks_Start()
{
    xTaskCreatePinnedToCore(motorTask, "Motor", kTaskStack, NULL,
                            kTaskPriority, NULL, kTaskCore);
}
