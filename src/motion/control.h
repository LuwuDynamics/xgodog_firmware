#pragma once
#include <stdint.h>

// The control task owns motion state and advances it once per measured tick.
void controlTask(void *parameter);
// A measured interval, bounded to 20 ms; elapsedUs==0 leaves state untouched.
void MotionControl_Step(uint32_t elapsedUs);
// Reset_Body requests private-state cleanup; the control task applies it before
// its next body update. Repeated requests coalesce without a command queue.
void MotionControl_RequestReset();
// Retain an idle motion command even if it is replaced before the next tick.
void MotionControl_NotifyMotionCommand();
