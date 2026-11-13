#include "protocol/protocol_internal.h"
#include "robot/state.h"
#include "motion/commands.h"
#include "robot/math.h"
#include "board/ble.h"
#include <Arduino.h>
#include "board/board.h"
#include "motor/motor.h"
#include "action/action.h"

namespace {
struct ScaledField {
    float *value;
    const float *minimum;
    const float *maximum;
};

// Contiguous deployed addresses 0x30..0x38. Reads and writes use the same
// conversion metadata, so model-specific ranges cannot drift between them.
const ScaledField kBodyFields[] = {
    {&body.vx, &bodyConfig.vx_min, &bodyConfig.vx_max},
    {&body.vy, &bodyConfig.vy_min, &bodyConfig.vy_max},
    {&imu.yaw_speed, &imu.yaw_speed_min, &imu.yaw_speed_max},
    {&body.x, &bodyConfig.xmin, &bodyConfig.xmax},
    {&body.y, &bodyConfig.ymin, &bodyConfig.ymax},
    {&body.z, &bodyConfig.zmin, &bodyConfig.zmax},
    {&body.rol, &bodyConfig.rolmin, &bodyConfig.rolmax},
    {&body.pit, &bodyConfig.pitmin, &bodyConfig.pitmax},
    {&body.yaw, &bodyConfig.yawmin, &bodyConfig.yawmax},
};
static_assert(sizeof(kBodyFields) / sizeof(kBodyFields[0]) == 9,
              "legacy scalar address range changed");

int motorIndex(uint8_t address)
{
    if (address >= 0x50 && address <= 0x5B) return address - 0x50;
    switch (address) {
        case 0x71: return 12;
        case 0x5D: return 13;
        case 0x5E: return 14;
        case 0x70: return 15;
        default: return -1;
    }
}

uint8_t readScaledMotorAngle(uint8_t index)
{
    return float_to_u8(motor[index].fb_angle, motor[index].min_angle,
                       motor[index].max_angle);
}
} // namespace

// Register addresses use byte-scaled values. Unknown reads and requested
// scalar lengths retain existing response-buffer behavior.
void RegisterProtocol_Write(uint8_t port, const MsgPacket &pkt)
{
	if (pkt.length < 8) return;
	uint8_t addr = pkt.message[1];
	uint8_t data_len = pkt.length - 8;
	if (data_len == 0) return;
	if(addr==0x03){
		RobotCommand_SetActionLoop(pkt.message[2]);
	}
	else if(addr==0x02){
		RobotCommand_SetBaud(port, pkt.message[2]);
	}
	else if(addr==0x04){
		RobotCommand_SetCalibration(pkt.message[2]);
	}

	else if(addr==0x06){
		if(pkt.message[2]==1){
			RobotCommand_SetYawOrigin();
		}
	}

	else if(addr==0x08){
		RobotCommand_SetFeedback(port, pkt.message[2]);
	}

	else if(addr==0x09){
		RobotCommand_SetGaitType(pkt.message[2]);
	}
	else if(addr == 0x13){
		if(data_len > 30) return;
		uint8_t bleNameRecord[32] = {0};
		bleNameRecord[0] = data_len;
		for (int i = 0; i < data_len; i++) {
			bleNameRecord[1 + i] = pkt.message[2 + i];
		}
		BLENameWrite(bleNameRecord);
	}
	else if(addr==0x20){
		xgo_mode = XGO_MODE_MOTOR;
		if(data_len==1){
			if(pkt.message[2]<=0x25)
			{
				parseEnableMotor(pkt.message[2]);
			}
		}
		if(data_len%2==0){
			for(int i=0;i<data_len/2;i++){
				Motor_WriteEnable(pkt.message[2+i*2], !pkt.message[2+i*2+1], motor_version);
			}
		}
	}
	else if(addr>=0x30&&addr<=0x38) {
        const ScaledField &field = kBodyFields[addr - 0x30];
        *field.value = u8_to_float(pkt.message[2], *field.minimum, *field.maximum);
        if (addr <= 0x32) {
            static const uint8_t motionAxes[] = {
                ROBOT_MOTION_VX, ROBOT_MOTION_VY, ROBOT_MOTION_YAW
            };
            RobotCommand_ApplyMotion(motionAxes[addr - 0x30]);
        } else {
            xgo_mode = XGO_MODE_BODY;
        }
    }
	else if(addr>=0x39&&addr<=0x3B){
		if(addr==0x39)	body.rol_T=pkt.message[2];
		else if(addr==0x3A) body.pit_T=pkt.message[2];
		else body.yaw_T=pkt.message[2];
		xgo_mode = XGO_MODE_BODY;
	}
	else if(addr>=0x80&&addr<=0x82){
		if(addr==0x80)	body.x_T=pkt.message[2];
		else if(addr==0x81) body.y_T=pkt.message[2];
		else body.z_T=pkt.message[2];
		xgo_mode = XGO_MODE_BODY;
	}

	else if(addr==0x3D){
		if (!RobotCommand_SetGaitPeriod(pkt.message[2]))
			xgo_mode = XGO_MODE_BODY; /* preserve legacy invalid-value side effect */
		SetLegMotorSpeed(0);
	}
	else if(addr==0X3E){
		SetAction(pkt.message[2]);
	}
	else if(addr==0x3F){
		if(data_len < 2) return;
		yaw_track_flag = 1;
		const int16_t yawOffset = static_cast<int16_t>(pkt.message[2] << 8 | pkt.message[3]);
		imu.yaw_des = imu.yaw_origin + yawOffset;
	}
	else if(addr>=0x40&&addr<=0x4B){
		const uint8_t writeLen = min(data_len, (uint8_t)(0x4C - addr));
		for(int i=0;i<writeLen;i++){
			if ((addr+i-0x40)%3==0)			leg[(addr+i-0x40)/3].x_base = u8_to_float(pkt.message[2+i],bodyConfig.xmin,bodyConfig.xmax);
			else if((addr+i-0x41)%3==0)		leg[(addr+i-0x41)/3].y_base = u8_to_float(pkt.message[2+i],bodyConfig.ymin,bodyConfig.ymax)-sign.nppn[(addr+i-0x41)/3]*bodyConfig.L_d;
			else							leg[(addr+i-0x42)/3].z_base = u8_to_float(pkt.message[2+i],bodyConfig.zmin,bodyConfig.zmax);
		}
		xgo_mode = XGO_MODE_LEG;
	}
	else if(addr>=0x50&&addr<0x5C){
		const uint8_t writeLen = min(data_len, (uint8_t)(0x5C - addr));
		for(int i=0;i<writeLen;i++){
			motor[addr-0x50+i].des_angle = u8_to_float(pkt.message[2+i],motor[addr-0x50+i].min_angle,motor[addr-0x50+i].max_angle);
		}
		xgo_mode = XGO_MODE_MOTOR;
	}
	else if(addr==0x5C){
		SetLegMotorSpeed(pkt.message[2]);
	}
	else if(addr==0x5D||addr==0x5E||addr==0x70){
		if(addr==0x5D||addr==0x5E){
			armCtrlMode = 0;
		}
		if (addr==0x5D)
		{
			motor[13].des_angle = u8_to_float(pkt.message[2],motor[13].min_angle,motor[13].max_angle);
		}else if(addr==0x5E){
			motor[14].des_angle = u8_to_float(pkt.message[2],motor[14].min_angle,motor[14].max_angle);
		}else if(addr==0x70){
			motor[15].des_angle = u8_to_float(pkt.message[2],motor[15].min_angle,motor[15].max_angle);
		}
	}else if(addr==0x71){
		motor[12].des_angle = u8_to_float(pkt.message[2],motor[12].min_angle,motor[12].max_angle);
	}
	else if(addr==0x5F){
		if(pkt.message[2]==0x01){
			MotorMoveToMid();
		}
	}
	else if(addr==0x61){
		RobotCommand_SetImuStable(pkt.message[2]);
	}
	else if(addr >= 0x69&&addr <= 0x6C){
        if(data_len == 3){
            const uint8_t ledIndex = addr - 0x69;
            user_rgb[ledIndex*3] = pkt.message[2];
            user_rgb[ledIndex*3+1] = pkt.message[3];
            user_rgb[ledIndex*3+2] = pkt.message[4];
        }else if(data_len >= 12){
            for(int i=0;i<12;i++){
                user_rgb[i] = pkt.message[i+2];
            }
        }
    }

	else if(addr==0x72){
		if(pkt.message[2]==0x00){
			armPosMode = 0;
		}
		else if(pkt.message[2]==0x01){
			armPosMode = 1;
		}
	}
	else if(addr==0x73||addr==0x74){
		armCtrlMode = 1;
		if(addr==0x73) arm_x = u8_to_float(pkt.message[2],ARM_X_MIN,ARM_X_MAX);
		else if(addr==0x74) arm_z = u8_to_float(pkt.message[2],ARM_Z_MIN,ARM_Z_MAX);
	}
	else if(addr==0x75){
		SetArmMotorSpeed(pkt.message[2]);
	}
	else if(addr==0x76||addr==0x77){
		if(addr == 0x76){
			arm_theta = pkt.message[2]/255.0 * 200.0/180.0*PI + 70.0/180.0*PI;
		}else{
			arm_r = pkt.message[2]/255.0*(140.0-80.0) + 80.0;
		}
		RobotCommand_ApplyArmPolar();
	}
	else if(addr==0x90){
		if(pkt.message[2]==0x00||pkt.message[2]==0x01){
			anaCtrl = pkt.message[2];
		}
	}else if(addr==0x91){
		if(pkt.message[2]==0x00||pkt.message[2]==0x01){
			digCtrl = pkt.message[2];
		}
	}
	else if(addr>=0xA1&&addr<=0xA9){
		if(data_len < 2) return;
		const uint8_t extensionIndex = addr - 0xA1;
		ex_motor[extensionIndex].des_pos = ((uint16_t)pkt.message[2]<<8 | pkt.message[3]);
	}
	else if(addr==0xB0){
		if(pkt.message[2]==MOVE_MODE_HYBRID||
			pkt.message[2]==MOVE_MODE_WHEEL||
			pkt.message[2]==MOVE_MODE_LEG||
			pkt.message[2]==0x03){
			RobotCommand_ApplyMoveMode(pkt.message[2]);
		}
	}
	else if(addr==0xB1){
		if(data_len < 4) return;
		MoveMode = MOVE_MODE_LEG;
		for(int i=0;i<4;i++){
            const int offset = int(pkt.message[2+i]) - 0x80;
            // The positive half has 127 steps, the negative half has 128.
            // Both endpoints reach the drive limit; 0x80 is exactly zero.
            wheel[i].des_vel = (float(offset) / (offset > 0 ? 127.0f : 128.0f))
                * (WHEEL_ORDER_MAX / WHEEL_OPEN_LOOP_GAIN);
		}
	}
	else if(addr==0xF0){
		SetTestMode(pkt.message[2]);
	}
}

void RegisterProtocol_Read(MsgRead &rb)
{
    // Preserve the requested length (and previous tail bytes) for scalar reads.
    if (rb.addr >= 0x30 && rb.addr <= 0x38) {
        const ScaledField &field = kBodyFields[rb.addr - 0x30];
        rb.message[0] = float_to_u8(*field.value, *field.minimum, *field.maximum);
        return;
    }
    const int index = motorIndex(rb.addr);
    if (index >= 0) {
        if (rb.addr == 0x50 && rb.length != 1) {
            rb.length = MOTOR_NUM;
            for (uint8_t i = 0; i < MOTOR_NUM; ++i)
                rb.message[i] = readScaledMotorAngle(i);
        } else {
            rb.message[0] = readScaledMotorAngle((uint8_t)index);
        }
        return;
    }
    const float *const attitude[] = {&imu.rol, &imu.pit, &imu.yaw};
    if (rb.addr >= 0x62 && rb.addr <= 0x64) {
        rb.length = 4;
        Float2Byte(rb.message, *attitude[rb.addr - 0x62]);
        return;
    }
    if (rb.addr >= 0x66 && rb.addr <= 0x68) {
        rb.length = 2;
        const short angle = (short)*attitude[rb.addr - 0x66];
        rb.message[0] = angle >> 8;
        rb.message[1] = angle & 0xff;
        return;
    }
	switch(rb.addr)
	{
		case 0x00:
			rb.message[0] = 0x00;
			break;
		case 0x01:
			rb.length = 1;
			rb.message[0] = (uint8_t)battery;
			break;
		case 0x02:
			rb.message[0] = XGOMINI;
			break;
		case 0x07:
			rb.length=10;
			for(int i=0;i<rb.length;i++)
			{
				rb.message[i] = firmwareVersion[i];
			}
			break;
		case 0x3F:
			rb.length = 1;
			rb.message[0] = yaw_track_flag;
			break;
		case 0x61:
			rb.message[0] = imu.stable;
			break;
		case 0x92:
			rb.message[0] = analog_in_state;
			break;
		case 0x93:
			rb.message[0] = digital_in_state;
			break;
		case 0xB1:
			rb.length=16;
			for(int i=0;i<4;i++){
				Float2Byte(rb.message+4*i, wheel[i].fb_vel);
			}
			break;
		case 0xB2:
			rb.length=16;
			for(int i=0;i<4;i++){
				Int2Byte(rb.message+4*i, wheel[i].odom);
			}
			break;

		default:
			break;
	}
}
