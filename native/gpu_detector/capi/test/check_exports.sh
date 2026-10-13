#!/usr/bin/env bash
# Fails if the shared library exports anything other than the atgpu_* C API.
# Usage: check_exports.sh /path/to/libapriltag_gpu_capi.so
set -euo pipefail
lib="${1:?usage: check_exports.sh <shared library>}"

case "$(uname -s)" in
  Darwin)
    # Exported, defined, external symbols; macOS prefixes a leading underscore.
    syms=$(nm -gU "$lib" | awk '{print $NF}' | grep -v -E '^_atgpu_' || true)
    ;;
  *)
    # Dynamic symbol table.  The linker always emits a few bookkeeping symbols.
    syms=$(nm -D --defined-only "$lib" | awk '{print $NF}' \
      | grep -v -E '^(atgpu_|_init$|_fini$|_edata$|_end$|__bss_start$)' || true)
    ;;
esac

if [ -n "$syms" ]; then
  echo "ERROR: $lib exports symbols outside the atgpu_* C API:" >&2
  echo "$syms" | head -50 >&2
  echo "($(echo "$syms" | wc -l | tr -d ' ') symbols)" >&2
  exit 1
fi
echo "OK: $lib exports only atgpu_* symbols:"
case "$(uname -s)" in
  Darwin) nm -gU "$lib" | awk '{print "  " $NF}' ;;
  *) nm -D --defined-only "$lib" | awk '{print "  " $NF}' | grep atgpu_ ;;
esac
