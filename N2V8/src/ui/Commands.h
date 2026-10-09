// Commands.h — what each console command prints (Requirements §10).
//
// Every answer is a Responder that produces its lines one at a time from live data, so the console can
// pump them out as TX space allows and never needs a large buffer or ever blocks.
#pragma once

#include <stdarg.h>

#include "../core/LoopStats.h"
#include "../core/NvmSettingsService.h"
#include "../core/System.h"
#include "../drivers/Lcd20x4.h"
#include "../drivers/Rtc3231.h"
#include "../hal/Hal.h"
#include "BuildInfo.h"
#include "ScanReport.h"
#include "Console.h"
#include "LcdScreens.h"

namespace n2 {

// Lets the `post` and `bist` commands ask the application to switch modes (it does so at the start of a
// loop pass, never in the middle of one). Each returns the one line to print.
class CommandLauncher {
 public:
  virtual ~CommandLauncher() = default;
  virtual const char* requestPost() = 0;
  virtual const char* requestBist() = 0;
  virtual const char* requestMode(const char* name, bool confirmed) = 0;   // name: nullptr = report the mode
};

struct ConsoleContext {
  System* sys;
  Console* console;
  LoopStats* loop;
  Hal* hal;
  BuildInfo info;
  LcdLayout layout;
  CommandLauncher* launcher = nullptr;
  Rtc3231* rtc = nullptr;  // the real-time clock, if the build has one
  NvmSettingsService* nvm = nullptr;  // settings in non-volatile memory, if the build has any
  const BoardDef* board = nullptr;    // the pin table, for the `pins` command
  Lcd20x4* lcd = nullptr;             // the LCD driver, for the `lcd` diagnostic command
};

class Commands : public CommandHandler {
 public:
  explicit Commands(const ConsoleContext& ctx);
  Responder* handle(const Command& command) override;

  // Individual answers, public so tests (and the BIST) can reuse them.
  Responder& help() { return help_; }
  Responder& ver() { return ver_; }
  Responder& time() { return time_; }
  Responder& status() { return status_; }
  Responder& faults() { return faults_; }
  Responder& cfg() { return cfg_; }
  Responder& display() { return display_; }
  Responder& loopStats() { return loop_; }
  Responder& scan() { return scan_; }
  Responder& report() { return report_; }

 private:
  class Help : public Responder { public: bool line(uint8_t i, char* b, size_t n) override; };
  class Ver : public Responder {
   public:
    explicit Ver(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Time : public Responder {
   public:
    explicit Time(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Status : public Responder {
   public:
    explicit Status(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Faults : public Responder {
   public:
    explicit Faults(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Cfg : public Responder {
   public:
    explicit Cfg(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Display : public Responder {
   public:
    explicit Display(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Loop : public Responder {
   public:
    explicit Loop(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Pins : public Responder {   // live state of every pin in the board table, to verify wiring (changes nothing)
   public:
    explicit Pins(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  class Scan : public Responder {
   public:
    explicit Scan(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
    uint8_t found_[16] = {};  // bitmap of addresses 0..127
    ScanReport report_;
  };
  class Message : public Responder {
   public:
    void set(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    char text_[96] = {};
  };
  class Report : public Responder {
   public:
    Report(Commands& owner) : o_(owner) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    Commands& o_;
  };

  ConsoleContext c_;
  Help help_;
  Ver ver_;
  Time time_;
  Status status_;
  Faults faults_;
  Cfg cfg_;
  class NvmInfo : public Responder {
   public:
    explicit NvmInfo(const ConsoleContext& c) : c_(c) {}
    bool line(uint8_t i, char* b, size_t n) override;
   private:
    const ConsoleContext& c_;
  };
  NvmInfo nvmInfo_;
  Pins pins_;
  Display display_;
  Loop loop_;
  Scan scan_;
  Message message_;
  Report report_;
};

}  // namespace n2
