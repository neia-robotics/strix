"""Several Owl-like processes, each holding its own GPU detector, detecting at once.

Owl runs one process per camera, so on a Jetson every camera gets its own CUDA
context and GPU buffers. This starts N processes that each create a detector for a
real test image (tiled up to the requested size), run detect() in a loop, and report
ms per frame. Run it while watching memory (`nvidia-smi` on a VM, `tegrastats` on a
Jetson).

    python tools/gpu_multiproc.py <libapriltag_gpu_capi.so> [--procs 4] [--seconds 10]
"""

import argparse
import importlib.util
import multiprocessing as mp
import pathlib
import statistics
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
CAPI = ROOT / "native" / "gpu_detector" / "capi"
TESTDATA = ROOT / "native" / "gpu_detector" / "testdata"


def load_smoke():
    """The AprilTagGpu ctypes wrapper and image loader from the C API's smoke test."""
    spec = importlib.util.spec_from_file_location("smoke_ctypes", CAPI / "test" / "smoke_ctypes.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def worker(idx, lib, width, height, seconds, start, out):
    import numpy as np

    smoke = load_smoke()
    gray = smoke.load_gray(str(TESTDATA / "grayimage.jpg"))  # 1280x800, one tag (585)
    reps = (-(-height // gray.shape[0]), -(-width // gray.shape[1]))
    frame = np.ascontiguousarray(np.tile(gray, reps)[:height, :width])
    expected = int(reps[0] * reps[1] > 0)

    at = smoke.AprilTagGpu(lib)
    at.create(width, height, at.default_settings())
    first = at.detect(frame)
    start.wait()  # all processes detect at the same time
    times = []
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        t0 = time.perf_counter()
        at.detect(frame)
        times.append((time.perf_counter() - t0) * 1000)
    at.close()
    out.put((idx, len(first), expected, times))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("library")
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=800)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    ctx = mp.get_context("spawn")  # a CUDA context doesn't survive fork()
    start = ctx.Barrier(args.procs)
    out = ctx.Queue()
    procs = [ctx.Process(target=worker, args=(i, args.library, args.width, args.height,
                                              args.seconds, start, out))
             for i in range(args.procs)]
    for p in procs:
        p.start()
    results = [out.get(timeout=args.seconds + 120) for _ in procs]
    for p in procs:
        p.join()

    ok = True
    print(f"{args.procs} processes, {args.width}x{args.height}, {args.seconds:g} s each")
    for idx, found, expected, times in sorted(results):
        p50 = statistics.median(times)
        p95 = sorted(times)[int(0.95 * (len(times) - 1))]
        print(f"  proc {idx}: {len(times)} frames, median {p50:.2f} ms, p95 {p95:.2f} ms, "
              f"tags in first frame {found}")
        ok = ok and found >= expected and all(p.exitcode == 0 for p in procs)
    print("PASSED" if ok else "FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
