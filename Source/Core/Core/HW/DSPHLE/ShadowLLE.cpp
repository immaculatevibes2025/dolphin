// Copyright 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/DSPHLE/ShadowLLE.h"

#include <array>
#include <string>
#include <vector>

#include "Common/CommonPaths.h"
#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Logging/Log.h"
#include "Common/Swap.h"
#include "Core/Config/MainSettings.h"
#include "Core/DSP/DSPCore.h"

namespace DSPHLE::Shadow
{
namespace
{
constexpr u32 kSampleRate = 32000;
constexpr int kMaxCyclesPerMail = 200000;

class WavWriter
{
public:
  bool Open(const std::string& path)
  {
    m_file.Open(path, "wb");
    if (!m_file.IsOpen())
      return false;
    m_bytes = 0;
    WriteHeader();
    return true;
  }
  void Write(const s16* lr_le, u32 num_frames)
  {
    if (!m_file.IsOpen())
      return;
    m_file.WriteBytes(lr_le, num_frames * 4);
    m_bytes += num_frames * 4;
    if (++m_writes % 200 == 0)  // keep the header valid about once a second
      Finalize();
  }
  void Close()
  {
    if (!m_file.IsOpen())
      return;
    Finalize();
    m_file.Close();
  }

private:
  void Put32(u8* p, u32 v)
  {
    p[0] = u8(v);
    p[1] = u8(v >> 8);
    p[2] = u8(v >> 16);
    p[3] = u8(v >> 24);
  }
  void WriteHeader()
  {
    u8 h[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
                16,  0,   0,   0,   1, 0, 2, 0};
    Put32(h + 4, 36 + m_bytes);
    Put32(h + 24, kSampleRate);
    Put32(h + 28, kSampleRate * 4);
    h[32] = 4;
    h[34] = 16;
    h[36] = 'd';
    h[37] = 'a';
    h[38] = 't';
    h[39] = 'a';
    Put32(h + 40, m_bytes);
    m_file.WriteBytes(h, sizeof(h));
  }
  void Finalize()
  {
    const u64 pos = m_file.Tell();
    m_file.Seek(0, File::SeekOrigin::Begin);
    WriteHeader();
    m_file.Seek(pos, File::SeekOrigin::Begin);
    m_file.Flush();
  }

  File::IOFile m_file;
  u32 m_bytes = 0;
  u32 m_writes = 0;
};

bool LoadRom(u16* rom, const std::string& filename, u32 size_in_bytes)
{
  std::string bytes;
  if (!File::ReadFileToString(filename, bytes) || bytes.size() != size_in_bytes)
    return false;
  const u16* words = reinterpret_cast<const u16*>(bytes.c_str());
  for (u32 i = 0; i < size_in_bytes / 2; ++i)
    rom[i] = Common::swap16(words[i]);
  return true;
}

struct State
{
  std::unique_ptr<DSP::DSPCore> core;
  bool running = false;
  WavWriter hle_wav, lle_wav;
  // Output addresses HLE wrote recently; shadow DMAs to these are audio frames.
  std::array<u32, 8> recent_out{};
  u32 recent_pos = 0;
  u64 hle_frames = 0, lle_frames = 0;
};

State* s_state = nullptr;

void AckShadowMail()
{
  // Pretend the CPU read any mail the shadow sent, so its ucode doesn't stall.
  auto& core = *s_state->core;
  for (int i = 0; i < 4 && (core.PeekMailbox(DSP::Mailbox::DSP) & 0x80000000); ++i)
  {
    core.ReadMailboxHigh(DSP::Mailbox::DSP);
    core.ReadMailboxLow(DSP::Mailbox::DSP);
  }
}

void Run(int cycles)
{
  s_state->running = true;
  s_state->core->RunCycles(cycles);
  s_state->running = false;
  AckShadowMail();
}
}  // namespace

bool IsEnabled()
{
  return Config::Get(Config::MAIN_DSP_SHADOW_LLE);
}

bool IsRunning()
{
  return s_state && s_state->running;
}

void Initialize()
{
  Shutdown();
  if (!IsEnabled())
    return;

  DSP::DSPInitOptions opts;
  std::string irom = File::GetUserPath(D_GCUSER_IDX) + DSP_IROM;
  std::string coef = File::GetUserPath(D_GCUSER_IDX) + DSP_COEF;
  if (!File::Exists(irom))
    irom = File::GetSysDirectory() + GC_SYS_DIR DIR_SEP DSP_IROM;
  if (!File::Exists(coef))
    coef = File::GetSysDirectory() + GC_SYS_DIR DIR_SEP DSP_COEF;
  if (!LoadRom(opts.irom_contents.data(), irom, DSP::DSP_IROM_BYTE_SIZE) ||
      !LoadRom(opts.coef_contents.data(), coef, DSP::DSP_COEF_BYTE_SIZE))
  {
    ERROR_LOG_FMT(DSPHLE, "ShadowLLE: could not load DSP ROMs, shadow disabled");
    return;
  }
  opts.core_type = DSP::DSPInitOptions::CoreType::Interpreter;

  s_state = new State;
  s_state->core = std::make_unique<DSP::DSPCore>();
  if (!s_state->core->Initialize(opts))
  {
    ERROR_LOG_FMT(DSPHLE, "ShadowLLE: core init failed, shadow disabled");
    delete s_state;
    s_state = nullptr;
    return;
  }
  s_state->core->Reset();

  const std::string dir = File::GetUserPath(D_DUMPAUDIO_IDX);
  File::CreateFullPath(dir);
  s_state->hle_wav.Open(dir + "shadow_hle.wav");
  s_state->lle_wav.Open(dir + "shadow_lle.wav");
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: enabled, writing {}shadow_hle.wav / shadow_lle.wav", dir);
}

void Shutdown()
{
  if (!s_state)
    return;
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: shutting down, {} HLE frames, {} LLE frames",
                 s_state->hle_frames, s_state->lle_frames);
  s_state->hle_wav.Close();
  s_state->lle_wav.Close();
  s_state->core->Shutdown();
  delete s_state;
  s_state = nullptr;
}

void OnControlWrite(u16 value)
{
  if (!s_state)
    return;
  auto& core = *s_state->core;
  core.GetInterpreter().WriteControlRegister(value);
  if (value & DSP::CR_EXTERNAL_INT)
  {
    s_state->running = true;
    core.CheckExternalInterrupt();
    core.CheckExceptions();
    s_state->running = false;
  }
}

void OnMailWrite(u32 mail)
{
  if (!s_state)
    return;
  auto& core = *s_state->core;
  core.WriteMailboxHigh(DSP::Mailbox::CPU, u16(mail >> 16));
  core.WriteMailboxLow(DSP::Mailbox::CPU, u16(mail));
  // Let the shadow consume the mail before the next one can overwrite it.
  for (int spent = 0; spent < kMaxCyclesPerMail; spent += 2000)
  {
    if (!(core.PeekMailbox(DSP::Mailbox::CPU) & 0x80000000))
      break;
    Run(2000);
  }
}

void OnUpdate(int cycles)
{
  if (!s_state || cycles <= 0)
    return;
  Run(cycles);
}

void OnHLEOutput(u32 addr, const s16* samples_be, u32 num_frames)
{
  if (!s_state)
    return;
  s_state->recent_out[s_state->recent_pos++ % s_state->recent_out.size()] = addr;
  std::vector<s16> lr(num_frames * 2);
  for (u32 i = 0; i < num_frames; ++i)
  {
    // Memory layout is R,L big-endian; WAV wants L,R little-endian.
    lr[2 * i + 0] = s16(Common::swap16(u16(samples_be[2 * i + 1])));
    lr[2 * i + 1] = s16(Common::swap16(u16(samples_be[2 * i + 0])));
  }
  s_state->hle_wav.Write(lr.data(), num_frames);
  ++s_state->hle_frames;
}

void OnShadowDMAFromDSP(const u16* src, u32 addr, u32 size)
{
  if (!s_state)
    return;
  bool is_output = false;
  for (u32 a : s_state->recent_out)
    is_output |= (a != 0 && a == addr);
  if (!is_output || size % 4 != 0)
    return;
  // src is DSP words in host order: R,L pairs.
  const u32 num_frames = size / 4;
  std::vector<s16> lr(num_frames * 2);
  for (u32 i = 0; i < num_frames; ++i)
  {
    lr[2 * i + 0] = s16(src[2 * i + 1]);
    lr[2 * i + 1] = s16(src[2 * i + 0]);
  }
  s_state->lle_wav.Write(lr.data(), num_frames);
  ++s_state->lle_frames;
}
}  // namespace DSPHLE::Shadow
