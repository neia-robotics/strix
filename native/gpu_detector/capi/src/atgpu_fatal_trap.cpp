#include "atgpu_fatal_trap.h"

#include <glog/logging.h>

#include <cstdlib>
#include <cstring>
#include <mutex>

namespace atgpu {

namespace {

thread_local FatalTrap *tls_current_trap = nullptr;

void RecordFatal(google::LogSeverity severity, const char *base_filename,
                 int line, const char *message, size_t message_len) {
  if (severity != google::GLOG_FATAL) return;
  FatalTrap *trap = tls_current_trap;
  if (trap == nullptr) return;
  trap->message.clear();
  if (base_filename != nullptr) {
    trap->message += base_filename;
    trap->message += ':';
    trap->message += std::to_string(line);
    trap->message += "] ";
  }
  if (message != nullptr) trap->message.append(message, message_len);
}

// glog calls LogSink::send() for every message (including FATAL) before it
// calls the failure function, so this is where the text is captured.  The
// send() signature changed across glog releases; CMake probes which one the
// installed glog has (ATGPU_GLOG_HAS_LOGMESSAGETIME).
class TrapSink : public google::LogSink {
 public:
#if ATGPU_GLOG_HAS_LOGMESSAGETIME
  void send(google::LogSeverity severity, const char * /*full_filename*/,
            const char *base_filename, int line,
            const google::LogMessageTime & /*time*/, const char *message,
            size_t message_len) override {
    RecordFatal(severity, base_filename, line, message, message_len);
  }
#else
  void send(google::LogSeverity severity, const char * /*full_filename*/,
            const char *base_filename, int line, const struct ::tm * /*t*/,
            const char *message, size_t message_len) override {
    RecordFatal(severity, base_filename, line, message, message_len);
  }
#endif
};

// glog's logging_fail_func_t (0.6+) is "void (*)() __attribute__((noreturn))",
// and the GNU attribute is part of the pointer type, so the function must be
// declared with it (C++ [[noreturn]] is not enough for the conversion).
__attribute__((noreturn)) void OnFatal() {
  FatalTrap *trap = tls_current_trap;
  if (trap != nullptr) {
    siglongjmp(trap->env, 1);
  }
  // Not inside an atgpu_* call on this thread: keep glog's default behaviour.
  std::abort();
}

std::once_flag install_once;

}  // namespace

FatalTrap::FatalTrap() : previous_(tls_current_trap) {
  tls_current_trap = this;
}

FatalTrap::~FatalTrap() { tls_current_trap = previous_; }

void FatalTrap::InstallOnce() {
  std::call_once(install_once, [] {
    static TrapSink sink;  // lives for the whole process
    google::AddLogSink(&sink);
    google::InstallFailureFunction(&OnFatal);
  });
}

}  // namespace atgpu
