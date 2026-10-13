// Turns glog LOG(FATAL) / CHECK failures raised on the current thread into a
// non-local return instead of abort().
//
// The detector core uses CHECK/CHECK_CUDA everywhere (src/cuda_frc971.h), and
// a CHECK failure aborts the whole process.  The C layer validates every
// input before calling into the detector, so in normal operation no CHECK
// fires.  For the cases that cannot be validated up front (cudaMalloc out of
// memory, no usable GPU, a driver fault, the CHECK_LT on the number of
// boundary points in GpuDetector::Detect for a pathological frame) this trap
// lets the camera process report an error and fall back instead of dying.
//
// Mechanism: a glog failure function (google::InstallFailureFunction) that
// siglongjmp()s back to the API entry point if a FatalTrap is armed on this
// thread, and a glog LogSink that records the FATAL message text so it can be
// returned through atgpu_last_error().  glog releases its logging mutex before
// calling the failure function (checked in 0.4.0, 0.6.0 and 0.7.1), so logging
// keeps working afterwards.  Caveats: C++ destructors between the CHECK and
// the trap do not run (the partially built detector leaks), and a CUDA sticky
// error leaves the context unusable, so a trapped failure poisons the handle;
// the caller should destroy it and treat the GPU backend as unavailable.
#ifndef ATGPU_FATAL_TRAP_H_
#define ATGPU_FATAL_TRAP_H_

#include <csetjmp>
#include <string>

namespace atgpu {

class FatalTrap {
 public:
  // Arms the trap for the current thread (traps nest; the newest wins).
  FatalTrap();
  ~FatalTrap();
  FatalTrap(const FatalTrap &) = delete;
  FatalTrap &operator=(const FatalTrap &) = delete;

  // Installs the glog failure function and log sink once per process.
  static void InstallOnce();

  // Jump target.  Use through ATGPU_FATAL_TRAP_ARMED().
  sigjmp_buf env;
  // "file:line] message" of the FATAL log that fired, filled by the sink.
  std::string message;

 private:
  FatalTrap *previous_;
};

// Evaluates to true the first time (trap armed), false when control comes
// back from a trapped fatal error.  Must be used directly in the function
// that owns the FatalTrap.
#define ATGPU_FATAL_TRAP_ARMED(trap) (sigsetjmp((trap).env, 1) == 0)

}  // namespace atgpu

#endif  // ATGPU_FATAL_TRAP_H_
