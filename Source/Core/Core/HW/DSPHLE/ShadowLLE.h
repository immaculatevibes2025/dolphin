// Copyright 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

// Shadow LLE: runs a second, hidden low-level DSP alongside DSP HLE, fed the same
// CPU->DSP mails and control writes, so both produce audio for the same frames.
// The shadow never touches emulated state: its interrupts are dropped, its DMA
// writes to main memory are discarded (except audio output, which is captured),
// and its mails to the CPU are acknowledged internally.
//
// Output (User/Dump/Audio):
//   shadow_hle.wav  - what DSP HLE produced, frame by frame
//   shadow_lle.wav  - what the real ucode produced for the same frames
// Enable with [DSP] ShadowLLE = True in Dolphin.ini. GameCube AX ucodes only.

#pragma once

#include <memory>

#include "Common/CommonTypes.h"

namespace DSPHLE::Shadow
{
bool IsEnabled();

// Called from DSPHLE.
void Initialize();
void Shutdown();
void OnControlWrite(u16 value);
void OnMailWrite(u32 mail);
void OnUpdate(int cycles);

// Called by an HLE ucode when it writes a finished output frame.
// samples: interleaved big-endian s16 R,L pairs as written to emulated memory.
void OnHLEOutput(u32 addr, const s16* samples_be, u32 num_frames);

// True while the shadow core is executing (used to redirect DSP::Host calls).
bool IsRunning();
// Called from DSP::Host::DMAFromDSP while the shadow runs.
void OnShadowDMAFromDSP(const u16* src, u32 addr, u32 size);
}  // namespace DSPHLE::Shadow
