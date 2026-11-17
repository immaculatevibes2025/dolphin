// Copyright 2026 Dolphin Emulator Project
// SPDX-License-Identifier: GPL-2.0-or-later

#include "Core/HW/DSPHLE/ShadowLLE.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <array>
#include <string>
#include <vector>

#include <fmt/format.h>

#include "Common/CommonPaths.h"
#include "Common/FileUtil.h"
#include "Common/IOFile.h"
#include "Common/Logging/Log.h"
#include "Common/Swap.h"
#include "Core/Config/MainSettings.h"
#include "Core/DSP/DSPCore.h"
#include "Core/DSP/DSPTables.h"
#include "Core/DSP/Interpreter/DSPInterpreter.h"

namespace DSPHLE::Shadow
{
namespace
{
constexpr u32 kSampleRate = 32000;
// Short wait: if the shadow is busy it takes the mail later on its own, and a
// long wait here slows emulation (audible as crackling).
constexpr int kMaxCyclesPerMail = 4000;
constexpr u32 kFrameBytes = 5 * 32 * 4;  // one AX frame: 160 stereo s16 samples

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

class BusWriter
{
public:
  bool Open(const std::string& path)
  {
    m_file.Open(path, "wb");
    m_bytes = 0;
    if (m_file.IsOpen())
      Header();
    return m_file.IsOpen();
  }
  void Write(const s32* interleaved, u32 frames)
  {
    if (!m_file.IsOpen())
      return;
    m_file.WriteBytes(interleaved, frames * kCh * 4);
    m_bytes += frames * kCh * 4;
    if (++m_writes % 200 == 0)
      Fix();
  }
  void Close()
  {
    if (!m_file.IsOpen())
      return;
    Fix();
    m_file.Close();
  }

private:
  static constexpr u32 kCh = 6;
  static void P32(u8* p, u32 v)
  {
    for (int i = 0; i < 4; ++i)
      p[i] = u8(v >> (8 * i));
  }
  void Header()
  {
    u8 h[44] = {'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16, 0, 0, 0, 1, 0, kCh, 0};
    P32(h + 4, 36 + m_bytes);
    P32(h + 24, kSampleRate);
    P32(h + 28, kSampleRate * kCh * 4);
    h[32] = kCh * 4;
    h[34] = 32;
    h[36] = 'd'; h[37] = 'a'; h[38] = 't'; h[39] = 'a';
    P32(h + 40, m_bytes);
    m_file.WriteBytes(h, sizeof(h));
  }
  void Fix()
  {
    const u64 pos = m_file.Tell();
    m_file.Seek(0, File::SeekOrigin::Begin);
    Header();
    m_file.Seek(pos, File::SeekOrigin::Begin);
    m_file.Flush();
  }
  File::IOFile m_file;
  u32 m_bytes = 0, m_writes = 0;
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
  BusWriter bus_wav;
  u64 bus_frames = 0;
  // Output addresses HLE wrote recently; shadow DMAs to these are audio frames.
  std::array<u32, 8> recent_out{};
  u32 recent_pos = 0;
  u64 hle_frames = 0, lle_frames = 0;
  u32 mails_logged = 0;
  u32 dma_logged = 0;
  // Frame being assembled from the shadow's output DMAs (which may arrive in pieces).
  std::array<u16, kFrameBytes / 2> frame{};
  u32 frame_addr = 0;
  u32 frame_filled = 0;
  u32 stuck_mails = 0;
  u64 updates = 0;
};

State* s_state = nullptr;
void FlushShadowFrame();
bool s_failed = false;
u32 s_seen = 0;  // bitmask of steps already logged once

void FirstTime(u32 bit, const char* what)
{
  if (!(s_seen & bit))
  {
    s_seen |= bit;
    NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: first {}", what);
  }
}

using StepFn = void (*)(void*);
#ifdef _MSC_VER
// Runs fn(arg); on a crash returns the exception code instead of taking Dolphin down.
uintptr_t s_fault_addr = 0, s_fault_data = 0;
int Filter(EXCEPTION_POINTERS* ep)
{
  s_fault_addr = reinterpret_cast<uintptr_t>(ep->ExceptionRecord->ExceptionAddress);
  s_fault_data = ep->ExceptionRecord->NumberParameters >= 2 ?
                     ep->ExceptionRecord->ExceptionInformation[1] :
                     0;
  return EXCEPTION_EXECUTE_HANDLER;
}
unsigned long GuardedCall(StepFn fn, void* arg)
{
  __try
  {
    fn(arg);
  }
  __except (Filter(GetExceptionInformation()))
  {
    return GetExceptionCode();
  }
  return 0;
}
#else
unsigned long GuardedCall(StepFn fn, void* arg)
{
  fn(arg);
  return 0;
}
#endif

// Returns false (and disables the shadow) if the step crashed.
bool Guard(const char* what, StepFn fn, void* arg)
{
  if (s_failed)
    return false;
  const unsigned long code = GuardedCall(fn, arg);
  if (code != 0)
  {
    s_failed = true;
    if (s_state)
      s_state->running = false;
    NOTICE_LOG_FMT(DSPHLE,
                   "ShadowLLE: CRASHED in {} (exception {:08x}) at dsp pc={:04x}; shadow disabled, "
                   "HLE continues",
                   what, code, s_state ? s_state->core->DSPState().pc : 0);
#ifdef _MSC_VER
    NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: fault at host {:x} (Dolphin.exe base {:x}), data address {:x}",
                   s_fault_addr, reinterpret_cast<uintptr_t>(GetModuleHandleW(nullptr)),
                   s_fault_data);
#endif
    return false;
  }
  return true;
}

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

void RunStep(void* arg)
{
  s_state->running = true;
  s_state->core->RunCycles(*static_cast<int*>(arg));
  s_state->running = false;
  AckShadowMail();
}

void Run(int cycles)
{
  FirstTime(1, "run");
  Guard("run", RunStep, &cycles);
}
}  // namespace

bool IsEnabled()
{
  return Config::Get(Config::MAIN_DSP_SHADOW_LLE);
}

bool IsRunning()
{
  return s_state && s_state->running && !s_failed;
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

  // The shared DSP opcode table is only built by DSP LLE's own startup, so in
  // HLE mode it is empty; build it before the shadow decodes its first opcode.
  DSP::InitInstructionTable();

  s_failed = false;
  s_seen = 0;
  s_state = new State;
  s_state->core = std::make_unique<DSP::DSPCore>();
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: initializing core");
  if (!s_state->core->Initialize(opts))
  {
    ERROR_LOG_FMT(DSPHLE, "ShadowLLE: core init failed, shadow disabled");
    delete s_state;
    s_state = nullptr;
    return;
  }
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: core initialized, resetting");
  s_state->core->Reset();
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: core reset done");

  const std::string dir = File::GetUserPath(D_DUMPAUDIO_IDX);
  File::CreateFullPath(dir);
  s_state->hle_wav.Open(dir + "shadow_hle.wav");
  s_state->lle_wav.Open(dir + "shadow_lle.wav");
  s_state->bus_wav.Open(dir + "shadow_hle_buses.wav");
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: enabled, writing {}shadow_hle.wav / shadow_lle.wav", dir);
}

void Shutdown()
{
  if (!s_state)
    return;
  FlushShadowFrame();
  NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: shutting down, {} HLE frames, {} LLE frames, {} bus frames",
                 s_state->hle_frames, s_state->lle_frames, s_state->bus_frames);
  s_state->hle_wav.Close();
  s_state->lle_wav.Close();
  s_state->bus_wav.Close();
  s_state->core->Shutdown();
  delete s_state;
  s_state = nullptr;
}

void ControlStep(void* arg)
{
  const u16 value = *static_cast<u16*>(arg);
  auto& core = *s_state->core;
  s_state->running = true;
  core.GetInterpreter().WriteControlRegister(value);
  if (value & DSP::CR_EXTERNAL_INT)
  {
    core.CheckExternalInterrupt();
    core.CheckExceptions();
  }
  s_state->running = false;
}

void OnControlWrite(u16 value)
{
  if (!s_state || s_failed)
    return;
  FirstTime(2, "control write");
  Guard("control write", ControlStep, &value);
}

void MailStep(void* arg)
{
  const u32 mail = *static_cast<u32*>(arg);
  auto& core = *s_state->core;
  core.WriteMailboxHigh(DSP::Mailbox::CPU, u16(mail >> 16));
  core.WriteMailboxLow(DSP::Mailbox::CPU, u16(mail));
}

void OnMailWrite(u32 mail)
{
  if (!s_state || s_failed)
    return;
  FirstTime(4, "mail write");
  if (!Guard("mail write", MailStep, &mail))
    return;
  auto& core = *s_state->core;
  // Let the shadow consume the mail before the next one can overwrite it.
  int spent = 0;
  for (; spent < kMaxCyclesPerMail && !s_failed; spent += 2000)
  {
    if (!(core.PeekMailbox(DSP::Mailbox::CPU) & 0x80000000))
      break;
    Run(2000);
  }
  const bool consumed = !(core.PeekMailbox(DSP::Mailbox::CPU) & 0x80000000);
  if (!consumed)
    ++s_state->stuck_mails;
  if (s_state->mails_logged < 80)
  {
    ++s_state->mails_logged;
    NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: mail {:08x} {} after {} cycles, pc={:04x} cr={:04x}", mail,
                   consumed ? "taken" : "NOT taken", spent, core.DSPState().pc,
                   core.DSPState().control_reg);
  }
}

void OnUpdate(int cycles)
{
  if (!s_state || s_failed || cycles <= 0)
    return;
  if (++s_state->updates % 2000 == 1)
  {
    auto& core = *s_state->core;
    NOTICE_LOG_FMT(DSPHLE,
                   "ShadowLLE: status pc={:04x} cr={:04x} state={} cycles={} cpu_mbox={:08x} "
                   "dsp_mbox={:08x} hle_frames={} lle_frames={} stuck_mails={} iram_crc={:08x}",
                   core.DSPState().pc, core.DSPState().control_reg, static_cast<int>(core.GetState()), cycles,
                   core.PeekMailbox(DSP::Mailbox::CPU), core.PeekMailbox(DSP::Mailbox::DSP),
                   s_state->hle_frames, s_state->lle_frames, s_state->stuck_mails,
                   core.DSPState().GetIRAMCRC());
  }
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

namespace
{
void FlushShadowFrame()
{
  auto& st = *s_state;
  if (st.frame_filled == 0)
    return;
  // DSP words in host order, R,L pairs -> WAV L,R.
  std::vector<s16> lr(kFrameBytes / 2);
  for (u32 i = 0; i < kFrameBytes / 4; ++i)
  {
    lr[2 * i + 0] = s16(st.frame[2 * i + 1]);
    lr[2 * i + 1] = s16(st.frame[2 * i + 0]);
  }
  st.lle_wav.Write(lr.data(), kFrameBytes / 4);
  ++st.lle_frames;
  st.frame.fill(0);
  st.frame_filled = 0;
}
}  // namespace

void OnHLEBuses(const int* const bufs[6])
{
  if (!s_state)
    return;
  std::vector<s32> v(160 * 6);
  for (u32 i = 0; i < 160; ++i)
    for (u32 c = 0; c < 6; ++c)
      v[i * 6 + c] = bufs[c][i];
  s_state->bus_wav.Write(v.data(), 160);
  ++s_state->bus_frames;
}

void OnShadowDMAFromDSP(const u16* src, u32 addr, u32 size)
{
  if (!s_state)
    return;
  FirstTime(8, "DMA from shadow DSP");
  auto& st = *s_state;
  // Is this write inside one of the output buffers HLE wrote recently?
  u32 base = 0;
  for (u32 a : st.recent_out)
  {
    if (a != 0 && addr >= a && addr + size <= a + kFrameBytes)
      base = a;
  }
  if (st.dma_logged < 40)
  {
    ++st.dma_logged;
    NOTICE_LOG_FMT(DSPHLE, "ShadowLLE: shadow DMA to {:08x} size {:x}{}", addr, size,
                   base ? fmt::format(" (output buffer {:08x} +{:x})", base, addr - base) : "");
  }
  if (base == 0)
    return;
  // A write to a different buffer, or back to the start, begins a new frame.
  if (base != st.frame_addr || addr == base)
  {
    FlushShadowFrame();
    st.frame_addr = base;
  }
  const u32 off = (addr - base) / 2;
  for (u32 i = 0; i < size / 2 && off + i < st.frame.size(); ++i)
    st.frame[off + i] = src[i];
  st.frame_filled += size;
  if (st.frame_filled >= kFrameBytes)
    FlushShadowFrame();
}
}  // namespace DSPHLE::Shadow
