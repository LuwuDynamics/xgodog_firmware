#include "protocol/protocol_internal.h"
#include "robot/state.h"
#include "motion/commands.h"
#include "robot/math.h"
#include "board/ble.h"
#include "board/board.h"
#include "motor/motor.h"
#include "action/action.h"
#include <Arduino.h>
#include <string.h>

// Main addresses are decimal. The write/read tables below define each
// supported subfield and its encoding; wire addresses are not inferred ranges.

static float legLateralInput[4];
static float yawTrackingOffset = 0;

struct ProtocolWriteContext {
    uint8_t port;
    const uint8_t *payload;
    uint8_t payloadLen;
};
using WriteHandler = void (*)(const ProtocolWriteContext &context);

struct FloatFieldDef {
	uint8_t sub_addr;
	float  *value;
};

struct Uint8FieldDef {
	uint8_t  sub_addr;
	uint8_t *value;
};

struct FloatFieldMeta {
	const FloatFieldDef *fields;
	uint8_t count;
};

struct Uint8FieldMeta {
	const Uint8FieldDef *fields;
	uint8_t count;
};

static void assignSubFloat(uint8_t sub, float val, const FloatFieldDef *fields, uint8_t n)
{
	for (uint8_t j = 0; j < n; j++) {
		if (fields[j].sub_addr != sub) continue;
		*fields[j].value = val;
		return;
	}
}

static bool parseSubFloat(const uint8_t *data, uint8_t len,
                          const FloatFieldDef *fields, uint8_t n)
{
	if (len == 0 || len % 5 != 0) return false;
	for (uint8_t i = 0; i + 5 <= len; i += 5) {
		const uint8_t sub = data[i];
		const float val = Byte2Float((uint8_t *)(data + i + 1));
		assignSubFloat(sub, val, fields, n);
	}
	return true;
}

static bool parseSubUint8(const uint8_t *data, uint8_t len,
                          const Uint8FieldDef *fields, uint8_t n)
{
	if (len == 0 || len % 2 != 0) return false;
	for (uint8_t i = 0; i + 2 <= len; i += 2) {
		const uint8_t sub = data[i];
		const uint8_t val = data[i + 1];
		for (uint8_t j = 0; j < n; j++) {
			if (fields[j].sub_addr != sub) continue;
			*fields[j].value = val;
			break;
		}
	}
	return true;
}

// Each write address has one optional field list and a handler that applies
// command side effects after those fields have been assigned.
struct MajorDef {
    uint8_t main_addr;
    const FloatFieldMeta *floatFields;
    const Uint8FieldMeta *byteFields;
    WriteHandler post;
};

#define FLOAT_META(name, tbl) \
	static const FloatFieldMeta name = { tbl, (uint8_t)(sizeof(tbl) / sizeof((tbl)[0])) }

#define UINT8_META(name, tbl) \
	static const Uint8FieldMeta name = { tbl, (uint8_t)(sizeof(tbl) / sizeof((tbl)[0])) }

static const FloatFieldDef kFieldsMotion[] = {
	{1, &body.vx},
	{2, &body.vy},
	{3, &imu.yaw_speed},
};

static const FloatFieldDef kFieldsYawTrack[] = {
	{1, &yawTrackingOffset},
};

static const FloatFieldDef kFieldsPose[] = {
	{1, &body.x},
	{2, &body.y},
	{3, &body.z},
	{4, &body.rol},
	{5, &body.pit},
	{6, &body.yaw},
};

static const FloatFieldDef kFieldsPeriod[] = {
	{1, &body.x_T},
	{2, &body.y_T},
	{3, &body.z_T},
	{4, &body.rol_T},
	{5, &body.pit_T},
	{6, &body.yaw_T},
};

static const FloatFieldDef kFieldsLeg[] = {
	{1, &leg[0].x_base},  {2, &legLateralInput[0]},  {3, &leg[0].z_base},
	{4, &leg[1].x_base},  {5, &legLateralInput[1]},  {6, &leg[1].z_base},
	{7, &leg[2].x_base},  {8, &legLateralInput[2]},  {9, &leg[2].z_base},
	{10, &leg[3].x_base}, {11, &legLateralInput[3]}, {12, &leg[3].z_base},
};

static const Uint8FieldDef kFieldsIo[] = {
	{1, &anaCtrl},
	{2, &digCtrl},
};

static const FloatFieldDef kFieldsWheel[] = {
	{1, &wheel[0].des_vel},
	{2, &wheel[1].des_vel},
	{3, &wheel[2].des_vel},
	{4, &wheel[3].des_vel},
};

static const FloatFieldDef kFieldsArmJoint[] = {
	{1, &motor[13].des_angle},
	{2, &motor[14].des_angle},
	{3, &motor[15].des_angle},
};

static const FloatFieldDef kFieldsArmPos[] = {
	{1, &arm_x},
	{2, &arm_z},
	{3, &arm_r},
	{4, &arm_theta},
	{5, &motor[15].des_angle},
};

static const FloatFieldDef kFieldsDebug[] = {
	{1, &debug_var[0]},
	{2, &debug_var[1]},
	{3, &debug_var[2]},
	{4, &debug_var[3]},
	{5, &debug_var[4]},
	{6, &debug_var[5]},
	{7, &debug_var[6]},
	{8, &debug_var[7]},
	{9, &debug_var[8]},
	{10, &debug_var[9]},
};

FLOAT_META(kMetaMotion, kFieldsMotion);
FLOAT_META(kMetaYawTrack, kFieldsYawTrack);
FLOAT_META(kMetaPose, kFieldsPose);
FLOAT_META(kMetaPeriod, kFieldsPeriod);
FLOAT_META(kMetaLeg, kFieldsLeg);
FLOAT_META(kMetaWheel, kFieldsWheel);
FLOAT_META(kMetaArmJoint, kFieldsArmJoint);
FLOAT_META(kMetaArmPos, kFieldsArmPos);
FLOAT_META(kMetaDebug, kFieldsDebug);
UINT8_META(kMetaIo, kFieldsIo);

static bool payloadHasSub(const ProtocolWriteContext &context, uint8_t sub)
{
	for (uint8_t i = 0; i + 5 <= context.payloadLen; i += 5) {
		if (context.payload[i] == sub)
			return true;
	}
	return false;
}

static bool setMotorById(uint8_t id, float val)
{
	for (int j = 0; j < MOTOR_NUM; j++) {
		if (motor[j].id != id) continue;
		motor[j].des_angle = val;
		return true;
	}
	for (int j = 0; j < 10; j++) {
		if (ex_motor[j].id != id) continue;
		ex_motor[j].des_angle = val;
		return true;
	}
	return false;
}

static void syncExMotorPos(uint8_t id)
{
	for (uint8_t i = 0; i < 10; i++) {
		if (ex_motor[i].id != id) continue;
		uint16_t pos = (uint16_t)(ex_motor[i].des_angle * g_motorConfig.M_N /
		                          g_motorConfig.M_A) + ex_motor[i].zero_pos;
		ex_motor[i].des_pos = Saturation(pos, ex_motor[i].max_pos, ex_motor[i].min_pos);
		break;
	}
}

/* main 1~7：机身状态（各 main 仅 sub 1） */
static void postFeedback(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	RobotCommand_SetFeedback(context.port, context.payload[1]);
}

static void postCalibration(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	RobotCommand_SetCalibration(context.payload[1]);
}

static void postBaud(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1 || context.payload[1] > 5) return;
	RobotCommand_SetBaud(context.port, context.payload[1]);
}

static void postBleName(const ProtocolWriteContext &context)
{
	if (context.payloadLen < 2 || context.payload[0] != 1) return;
	const uint8_t nameLen = context.payload[1];
	if (nameLen == 0 || nameLen > 30 ||
	    context.payloadLen != (uint8_t)(2 + nameLen))
		return;
	uint8_t bleNameRecord[32] = {0};
	bleNameRecord[0] = nameLen;
	memcpy(bleNameRecord + 1, context.payload + 2, nameLen);
	BLENameWrite(bleNameRecord);
}

static void postTestMode(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	SetTestMode(context.payload[1]);
}

static void postBodyMode(const ProtocolWriteContext &)
{
	xgo_mode = XGO_MODE_BODY;
}

static void postMotion(const ProtocolWriteContext &)
{

	RobotCommand_ApplyMotion(ROBOT_MOTION_ALL);
}

static void postYawTrack(const ProtocolWriteContext &)
{
	yaw_track_flag = 1;
	imu.yaw_des = imu.yaw_origin + yawTrackingOffset;
	imu.stable = 0;
	xgo_mode = XGO_MODE_BODY;
}

static void postYawOrigin(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1 || context.payload[1] != 1) return;
	RobotCommand_SetYawOrigin();
}

static void postAction(const ProtocolWriteContext &context)
{
	for (uint8_t i = 0; i + 1 < context.payloadLen; i += 2) {
		if (context.payload[i] == 1)
			SetAction(context.payload[i + 1]);
		else if (context.payload[i] == 2) {
			RobotCommand_SetActionLoop(context.payload[i + 1]);
		}
	}
}

static void postGait(const ProtocolWriteContext &context)
{
	for (uint8_t i = 0; i + 1 < context.payloadLen; i += 2) {
		if (context.payload[i] == 1)
			RobotCommand_SetGaitPeriod(context.payload[i + 1]);
		else if (context.payload[i] == 2)
			RobotCommand_SetGaitType(context.payload[i + 1]);
	}
}

static void postLegBase(const ProtocolWriteContext &context)
{
	static const uint8_t kLegYSub[] = {2, 5, 8, 11};
	for (uint8_t i = 0; i < 4; i++) {
		if (payloadHasSub(context, kLegYSub[i]))
			leg[i].y_base = legLateralInput[i] - sign.nppn[i] * bodyConfig.L_d;
	}
	xgo_mode = XGO_MODE_LEG;
}

static void postMotorControl(const ProtocolWriteContext &context)
{
	if (context.payloadLen == 0 || context.payloadLen % 5 != 0) return;
	bool gotArmJoint = false;
	for (uint8_t i = 0; i + 5 <= context.payloadLen; i += 5) {
		const uint8_t id = context.payload[i];
		const float val = Byte2Float((uint8_t *)(context.payload + i + 1));
		if (id == 52 || id == 53)
			gotArmJoint = true;
		if (!setMotorById(id, val))
			continue;
		if (id >= 91 && id <= 100)
			syncExMotorPos(id);
	}
	if (gotArmJoint)
		armCtrlMode = 0;
	xgo_mode = XGO_MODE_MOTOR;
}

static void postMotorEnable(const ProtocolWriteContext &context)
{
	uint8_t i = 0;
	while (i < context.payloadLen) {
		const uint8_t sub = context.payload[i];
		if (sub == 1) {
			if (i + 2 >= context.payloadLen) return;
			Motor_WriteEnable(context.payload[i + 1], context.payload[i + 2], motor_version);
			i += 3;
		} else if (sub == 2) {
			if (i + 1 >= context.payloadLen) return;
			if (context.payload[i + 1] > 0x25) return;
			parseEnableMotor(context.payload[i + 1]);
			i += 2;
		} else {
			return;
		}
	}
	xgo_mode = XGO_MODE_MOTOR;
}

static void postServoSpeed(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	SetLegMotorSpeed(context.payload[1]);
}

static void postServoCenter(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1 || context.payload[1] != 1) return;
	MotorMoveToMid();
}

static void postGripper(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	uint8_t val = context.payload[1];
	if (val > 100) val = 100;
	motor[12].des_angle = motor[12].min_angle +
		(motor[12].max_angle - motor[12].min_angle) * val / 100.0f;
}

static void postArmJoint(const ProtocolWriteContext &)
{
	armCtrlMode = 0;
}

static void postArmPos(const ProtocolWriteContext &context)
{
	armCtrlMode = 1;
	if (payloadHasSub(context, 3) || payloadHasSub(context, 4)) {
		RobotCommand_ApplyArmPolar();
	}
}

static void postArmPosMode(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	armPosMode = context.payload[1];
}

static void postArmSpeed(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	SetArmMotorSpeed(context.payload[1]);
}

static void postImuStable(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	RobotCommand_SetImuStable(context.payload[1]);
}

static void postRgb(const ProtocolWriteContext &context)
{
	uint8_t i = 0;
	while (i + 4 <= context.payloadLen) {
		const uint8_t sub = context.payload[i];
		if (sub < 1 || sub > 4)
			return;
		const uint8_t idx = (uint8_t)(sub - 1);
		user_rgb[idx * 3] = context.payload[i + 1];
		user_rgb[idx * 3 + 1] = context.payload[i + 2];
		user_rgb[idx * 3 + 2] = context.payload[i + 3];
		i += 4;
	}
}

static void postMoveMode(const ProtocolWriteContext &context)
{
	if (context.payloadLen != 2 || context.payload[0] != 1) return;
	if (context.payload[1] != MOVE_MODE_HYBRID && context.payload[1] != MOVE_MODE_WHEEL &&
	    context.payload[1] != MOVE_MODE_LEG)
		return;
	RobotCommand_ApplyMoveMode(context.payload[1]);
}

static void postWheelVel(const ProtocolWriteContext &)
{
	for (int j = 0; j < 4; j++) {
		if (abs(wheel[j].des_vel) < 0.05f)
			wheel[j].des_vel = 0;
	}
	MoveMode = MOVE_MODE_LEG;
}

static const MajorDef kMajorTable[] = {
    {3,    nullptr,         nullptr,   postFeedback},
    {4,    nullptr,         nullptr,   postCalibration},
    {5,    nullptr,         nullptr,   postBaud},
    {6,    nullptr,         nullptr,   postBleName},
    {7,    nullptr,         nullptr,   postTestMode},
    {10,   &kMetaPose,      nullptr,   postBodyMode},
    {11,   &kMetaPeriod,    nullptr,   postBodyMode},
    {20,   &kMetaMotion,    nullptr,   postMotion},
    {21,   nullptr,         nullptr,   postAction},
    {22,   &kMetaYawTrack,  nullptr,   postYawTrack},
    {23,   nullptr,         nullptr,   postGait},
    {24,   nullptr,         nullptr,   postYawOrigin},
    {30,   &kMetaLeg,       nullptr,   postLegBase},
    {40,   nullptr,         nullptr,   postMotorControl},
    {41,   nullptr,         nullptr,   postMotorEnable},
    {43,   nullptr,         nullptr,   postServoSpeed},
    {44,   nullptr,         nullptr,   postServoCenter},
    {51,   nullptr,         nullptr,   postGripper},
    {52,   &kMetaArmJoint,  nullptr,   postArmJoint},
    {53,   &kMetaArmPos,    nullptr,   postArmPos},
    {55,   nullptr,         nullptr,   postArmPosMode},
    {56,   nullptr,         nullptr,   postArmSpeed},
    {61,   nullptr,         nullptr,   postImuStable},
    {62,   nullptr,         nullptr,   postRgb},
    {63,   nullptr,         &kMetaIo,  nullptr},
    {70,   nullptr,         nullptr,   postMoveMode},
    {71,   &kMetaWheel,     nullptr,   postWheelVel},
    {128,  &kMetaDebug,     nullptr,   nullptr},
};

void CommandProtocol_Write(uint8_t port, uint8_t main_addr,
                            const uint8_t *data, uint8_t len)
{
	for (uint8_t i = 0; i < sizeof(kMajorTable) / sizeof(kMajorTable[0]); i++) {
		if (main_addr != kMajorTable[i].main_addr) continue;
		const ProtocolWriteContext context = {
			port,
			data,
			len,
		};
        const MajorDef &def = kMajorTable[i];
        if (def.floatFields != nullptr &&
            !parseSubFloat(data, len, def.floatFields->fields, def.floatFields->count))
            return;
        if (def.byteFields != nullptr &&
            !parseSubUint8(data, len, def.byteFields->fields, def.byteFields->count))
            return;
        if (def.post != nullptr)
            def.post(context);
		return;
	}
}

/* ---------- 读协议 cmd=0x04 / 响应 cmd=0x14 ---------- */

constexpr uint8_t kReadPayloadCapacity = 240;

using ReadHandler = void (*)(const uint8_t *subs, uint8_t len,
                            uint8_t *out, uint8_t *outLen);

struct ReadMajorDef {
	uint8_t            main_addr;
	const FloatFieldMeta *floatRd;
	const Uint8FieldMeta *u8Rd;
	ReadHandler  custom;
};

static uint8_t appendSubFloat(uint8_t *out, uint8_t outLen, uint8_t sub, float val)
{
	if (outLen + 5 > kReadPayloadCapacity) return outLen;
	out[outLen] = sub;
	Float2Byte(out + outLen + 1, val);
	return (uint8_t)(outLen + 5);
}

static uint8_t appendSubU8(uint8_t *out, uint8_t outLen, uint8_t sub, uint8_t val)
{
	if (outLen + 2 > kReadPayloadCapacity) return outLen;
	out[outLen] = sub;
	out[outLen + 1] = val;
	return (uint8_t)(outLen + 2);
}

static void readSubsFloat(const uint8_t *subs, uint8_t len,
                          const FloatFieldDef *fields, uint8_t n,
                          uint8_t *out, uint8_t *outLen)
{
	for (uint8_t i = 0; i < len; i++) {
		for (uint8_t j = 0; j < n; j++) {
			if (fields[j].sub_addr != subs[i]) continue;
			*outLen = appendSubFloat(out, *outLen, subs[i], *fields[j].value);
			break;
		}
	}
}

static void readSubsUint8(const uint8_t *subs, uint8_t len,
                          const Uint8FieldDef *fields, uint8_t n,
                          uint8_t *out, uint8_t *outLen)
{
	for (uint8_t i = 0; i < len; i++) {
		for (uint8_t j = 0; j < n; j++) {
			if (fields[j].sub_addr != subs[i]) continue;
			*outLen = appendSubU8(out, *outLen, subs[i], *fields[j].value);
			break;
		}
	}
}

static const FloatFieldDef kFieldsLegRead[] = {
	{1, &leg[0].x_base},  {2, &leg[0].y_base},  {3, &leg[0].z_base},
	{4, &leg[1].x_base},  {5, &leg[1].y_base},  {6, &leg[1].z_base},
	{7, &leg[2].x_base},  {8, &leg[2].y_base},  {9, &leg[2].z_base},
	{10, &leg[3].x_base}, {11, &leg[3].y_base}, {12, &leg[3].z_base},
};

static const FloatFieldDef kFieldsWheelRead[] = {
	{1, &wheel[0].fb_vel},
	{2, &wheel[1].fb_vel},
	{3, &wheel[2].fb_vel},
	{4, &wheel[3].fb_vel},
};

static const FloatFieldDef kFieldsImu[] = {
	{1, &imu.rol},
	{2, &imu.pit},
	{3, &imu.yaw},
	{4, &imu.acc[0]},
	{5, &imu.acc[1]},
	{6, &imu.acc[2]},
	{7, &imu.gyro[0]},
	{8, &imu.gyro[1]},
	{9, &imu.gyro[2]},
};

static const FloatFieldDef kFieldsArmJointRead[] = {
	{1, &motor[13].fb_angle},
	{2, &motor[14].fb_angle},
	{3, &motor[15].fb_angle},
};

static const FloatFieldDef kFieldsArmPosRead[] = {
	{1, &arm_x},
	{2, &arm_z},
	{3, &arm_r},
	{4, &arm_theta},
	{5, &motor[15].fb_angle},
};

static const FloatFieldDef kFieldsGripperRead[] = {
	{1, &motor[12].fb_angle},
};

static const Uint8FieldDef kFieldsIoRead[] = {
	{1, &analog_in_state},
	{2, &digital_in_state},
};

static const Uint8FieldDef kFieldsArmPosModeRead[] = {
	{1, &armPosMode},
};

FLOAT_META(kMetaLegRead, kFieldsLegRead);
FLOAT_META(kMetaWheelRead, kFieldsWheelRead);
FLOAT_META(kMetaImu, kFieldsImu);
FLOAT_META(kMetaArmJointRead, kFieldsArmJointRead);
FLOAT_META(kMetaArmPosRead, kFieldsArmPosRead);
FLOAT_META(kMetaGripperRead, kFieldsGripperRead);
UINT8_META(kMetaIoRead, kFieldsIoRead);
UINT8_META(kMetaArmPosModeRead, kFieldsArmPosModeRead);

static void readMainVersion(const uint8_t *subs, uint8_t len,
                          uint8_t *out, uint8_t *outLen)
{
	static const uint8_t kVerLen = 10;
	for (uint8_t i = 0; i < len; i++) {
		if (subs[i] != 1) continue;
		if (*outLen + 2 + kVerLen > kReadPayloadCapacity) continue;
		out[*outLen] = 1;
		out[*outLen + 1] = kVerLen;
		memcpy(out + *outLen + 2, firmwareVersion, kVerLen);
		*outLen = (uint8_t)(*outLen + 2 + kVerLen);
	}
}

static void readMainBattery(const uint8_t *subs, uint8_t len,
                            uint8_t *out, uint8_t *outLen)
{
	for (uint8_t i = 0; i < len; i++) {
		if (subs[i] == 1)
			*outLen = appendSubU8(out, *outLen, 1, (uint8_t)battery);
	}
}

static void readMotorAngles(const uint8_t *subs, uint8_t len,
                       uint8_t *out, uint8_t *outLen)
{
	if (len == 1 && subs[0] == 0xFF) {
		for (int j = 0; j < MOTOR_NUM; j++)
			*outLen = appendSubFloat(out, *outLen, motor[j].id, motor[j].fb_angle);
		return;
	}
	for (uint8_t i = 0; i < len; i++) {
		const uint8_t id = subs[i];
		if (id == 0xFF) continue;
		bool found = false;
		for (int j = 0; j < MOTOR_NUM; j++) {
			if (motor[j].id != id) continue;
			*outLen = appendSubFloat(out, *outLen, id, motor[j].fb_angle);
			found = true;
			break;
		}
		if (found) continue;
		if (id >= 91 && id <= 100) {
			for (int j = 0; j < 10; j++) {
				if (ex_motor[j].id != id) continue;
				*outLen = appendSubFloat(out, *outLen, id, ex_motor[j].fb_angle);
				break;
			}
		}
	}
}

static const ReadMajorDef kReadMajorTable[] = {
	{1,  nullptr,            nullptr,         readMainVersion},
	{2,  nullptr,            nullptr,         readMainBattery},
	{10, &kMetaPose,         nullptr,         nullptr},
	{11, &kMetaPeriod,       nullptr,         nullptr},
	{20, &kMetaMotion,       nullptr,         nullptr},
	{30, &kMetaLegRead,      nullptr,         nullptr},
	{40, nullptr,            nullptr,         readMotorAngles},
	{51, &kMetaGripperRead,  nullptr,         nullptr},
	{52, &kMetaArmJointRead, nullptr,         nullptr},
	{53, &kMetaArmPosRead,   nullptr,         nullptr},
	{55, nullptr,            &kMetaArmPosModeRead, nullptr},
	{60, &kMetaImu,          nullptr,         nullptr},
	{63, nullptr,            &kMetaIoRead,    nullptr},
	{71, &kMetaWheelRead,    nullptr,         nullptr},
	{128, &kMetaDebug,       nullptr,         nullptr},
};

void CommandProtocol_Read(uint8_t port, uint8_t main_addr,
                           const uint8_t *subs, uint8_t len)
{
	if (subs == nullptr || len == 0) return;

	uint8_t frame[PROTOCOL_FRAME_MAX_SIZE];
	uint8_t *out = frame + 5;
	uint8_t outLen = 0;

	for (uint8_t i = 0; i < sizeof(kReadMajorTable) / sizeof(kReadMajorTable[0]); i++) {
		if (main_addr != kReadMajorTable[i].main_addr) continue;
		const ReadMajorDef *def = &kReadMajorTable[i];
		if (def->custom != nullptr)
			def->custom(subs, len, out, &outLen);
		if (def->floatRd != nullptr)
			readSubsFloat(subs, len, def->floatRd->fields, def->floatRd->count,
			              out, &outLen);
		if (def->u8Rd != nullptr)
			readSubsUint8(subs, len, def->u8Rd->fields, def->u8Rd->count,
			              out, &outLen);
		if (outLen > 0) {
			uint8_t frameLen = 0;
			if (ProtocolFrame_FinalizeResponse(0x14, main_addr, outLen,
			                                   frame, sizeof(frame), &frameLen))
				ProtocolTransport_Send(port, frame, frameLen);
		}
		return;
	}
}
