#pragma once
#include <stdint.h>

// Reset mode: 0 keeps the current action, 1 clears it, 2 also stops the loop.
void Reset(uint8_t mode);
// The existing 600 ms Reset phase, independent of servo feedback.
bool Action_IsRecovering();
void Falldown();
void FoldArm();
void SetAction(uint8_t id);
// Called once per control step; does not block on action or arm delays.
void ActionManager();

// Protocol-controlled factory exercise; 0 disables it.
void SetTestMode(uint8_t mode);
