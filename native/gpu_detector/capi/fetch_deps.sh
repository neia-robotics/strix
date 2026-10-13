#!/usr/bin/env bash
# Downloads the two source-only dependencies of the C API build into
# capi/deps/ so that cmake can run without internet access afterwards.
#
#   ./capi/fetch_deps.sh            -> capi/deps/{cccl-2.3.2,apriltag-3.3.0}
#   ./capi/fetch_deps.sh /some/dir  -> same, elsewhere (pass -DATGPU_DEPS_DIR=/some/dir)
#
# Everything else the build needs comes from apt / JetPack (see README.md).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
deps="${1:-$here/deps}"

CCCL_VERSION=2.3.2
APRILTAG_VERSION=3.3.0
CCCL_URL="https://github.com/NVIDIA/cccl/archive/refs/tags/v${CCCL_VERSION}.tar.gz"
APRILTAG_URL="https://github.com/cgpadwick/apriltag/archive/refs/tags/${APRILTAG_VERSION}.tar.gz"

mkdir -p "$deps"
cd "$deps"

fetch() {
  local url="$1" tarball="$2" dir="$3" marker="$4"
  if [ -e "$dir/$marker" ]; then
    echo "already present: $deps/$dir"
    return
  fi
  echo "downloading $url"
  curl -fL --retry 3 -o "$tarball" "$url"
  tar xzf "$tarball"
  [ -e "$dir/$marker" ] || { echo "ERROR: $dir/$marker missing after extracting $tarball" >&2; exit 1; }
  rm -f "$tarball"
  echo "ready: $deps/$dir"
}

fetch "$CCCL_URL" "cccl-${CCCL_VERSION}.tar.gz" "cccl-${CCCL_VERSION}" cub/cub/cub.cuh
fetch "$APRILTAG_URL" "apriltag-${APRILTAG_VERSION}.tar.gz" "apriltag-${APRILTAG_VERSION}" apriltag.h

echo
echo "Done.  Configure with:  cmake -S $here -B $here/build-capi"
echo "(the build picks these up automatically from $deps)"
