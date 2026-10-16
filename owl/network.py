"""Networking settings from the coprocessor page: NetworkTables target, hostname, IP.

NetworkTables (team number or server address, standalone server) is Owl's own
setting and applies immediately. Hostname and DHCP/static IP change the
coprocessor itself, which needs root, so they go through deploy/owl-admin: a
small root-owned helper that Owl runs with `sudo -n` (see deploy/README in
owl/README.md). They're only touched when "manage" is on, and only when you save
from the page; config.json's "network" section records what was last applied.
"""

from __future__ import annotations

import ipaddress
import os
import re
import shutil
import subprocess

HELPER = "/usr/local/sbin/owl-admin"
_REPO_HELPER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "deploy", "owl-admin")

DEFAULT_NETWORK = {
    "manage": False,
    "hostname": "owl",
    "interface": "",
    "mode": "dhcp",  # or "static"
    "ip": "",
    "prefix": 24,
    "gateway": "",
}

_LABEL = re.compile(r"[a-z0-9]([a-z0-9-]{0,61}[a-z0-9])?")
_IFACE = re.compile(r"[A-Za-z0-9_.-]{1,15}")


def valid_hostname(name: str, *, dotted: bool) -> bool:
    labels = name.split(".") if dotted else [name]
    return 0 < len(name) <= 253 and all(_LABEL.fullmatch(label) for label in labels)


def parse_nt_target(text) -> tuple[int | None, str | None]:
    """'7407' -> (7407, None); '10.74.7.2' or 'roborio-7407-frc.local' -> (None, text).
    Empty -> (None, None). Raises ValueError for anything else."""
    text = str(text if text is not None else "").strip()
    if not text:
        return None, None
    if text.isdigit():
        team = int(text)
        if not 1 <= team <= 25599:
            raise ValueError("team number must be between 1 and 25599")
        return team, None
    try:
        return None, str(ipaddress.IPv4Address(text))
    except ValueError:
        pass
    if not valid_hostname(text.lower(), dotted=True):
        raise ValueError("NetworkTables server must be a team number, an IPv4 address or a hostname")
    return None, text.lower()


def nt_target_text(team, server) -> str:
    return server or (str(team) if team is not None else "")


def validate_network(raw: dict) -> dict:
    """Normalized copy of a "network" section; raises ValueError if anything is off."""
    if not isinstance(raw, dict):
        raise ValueError("network settings must be an object")
    unknown = set(raw) - set(DEFAULT_NETWORK)
    if unknown:
        raise ValueError(f"unknown network setting: {sorted(unknown)[0]}")
    n = {**DEFAULT_NETWORK, **raw}
    if not isinstance(n["manage"], bool):
        raise ValueError("manage must be true or false")
    n["hostname"] = str(n["hostname"]).strip().lower()
    if not valid_hostname(n["hostname"], dotted=False):
        raise ValueError("hostname must be 1-63 letters, digits or hyphens (no dots, no leading/trailing hyphen)")
    n["interface"] = str(n["interface"] or "").strip()
    if n["interface"] and not _IFACE.fullmatch(n["interface"]):
        raise ValueError("invalid interface name")
    if n["mode"] not in ("dhcp", "static"):
        raise ValueError("IP mode must be dhcp or static")
    if isinstance(n["prefix"], bool):
        raise ValueError("netmask must be a prefix length")
    n["prefix"] = int(n["prefix"])
    if not 8 <= n["prefix"] <= 30:
        raise ValueError("netmask must be /8 to /30")
    n["ip"] = str(n["ip"] or "").strip()
    n["gateway"] = str(n["gateway"] or "").strip()
    try:
        _check_static(n)
    except ValueError:
        if n["mode"] == "static":
            raise
        n["ip"] = n["gateway"] = ""  # DHCP: static fields are only kept to prefill the page
    if n["manage"] and not n["interface"]:
        raise ValueError("pick the network interface to manage")
    return n


def _check_static(n: dict) -> None:
    if not n["ip"] and n["mode"] == "dhcp" and not n["gateway"]:
        return
    try:
        iface = ipaddress.IPv4Interface(f"{n['ip']}/{n['prefix']}")
    except ValueError:
        raise ValueError("static IP must be an IPv4 address like 10.74.7.11") from None
    if iface.ip in (iface.network.network_address, iface.network.broadcast_address):
        raise ValueError("static IP can't be the subnet's network or broadcast address")
    n["ip"] = str(iface.ip)
    if n["gateway"]:
        try:
            gw = ipaddress.IPv4Address(n["gateway"])
        except ValueError:
            raise ValueError("gateway must be an IPv4 address (or empty)") from None
        if gw not in iface.network or gw == iface.ip:
            raise ValueError("gateway must be another address on the static IP's subnet")
        n["gateway"] = str(gw)


def ethernet_interfaces() -> list[str]:
    """Physical wired interfaces (eth0, end0, enP8p1s0, ...), from /sys. Skips loopback,
    Wi-Fi, virtual ones with no device (docker0, l4tbr0) and USB gadget links."""
    out = []
    try:
        names = sorted(os.listdir("/sys/class/net"))
    except OSError:
        return out
    for name in names:
        base = f"/sys/class/net/{name}"
        try:
            with open(f"{base}/type") as f:
                if f.read().strip() != "1":  # ARPHRD_ETHER
                    continue
        except OSError:
            continue
        if os.path.exists(f"{base}/wireless") or not os.path.exists(f"{base}/device"):
            continue
        if "gadget" in os.path.realpath(f"{base}/device"):  # Jetson's USB device-mode links (usb0, usb1)
            continue
        out.append(name)
    return out


def helper_command() -> list[str] | None:
    """How to run the root helper, or None if it isn't installed."""
    if os.path.exists(HELPER):
        return [HELPER] if os.geteuid() == 0 else ["sudo", "-n", HELPER]
    if os.geteuid() == 0 and os.path.exists(_REPO_HELPER):  # already root: the repo copy is fine
        return ["python3", os.path.abspath(_REPO_HELPER)]
    return None


def system_support() -> str | None:
    """Why hostname/IP can't be managed on this machine, or None if they can."""
    if not os.path.isdir("/sys/class/net"):
        return "only supported on Linux coprocessors"
    if not shutil.which("nmcli"):
        return "NetworkManager (nmcli) isn't installed"
    if helper_command() is None:
        return f"the admin helper isn't installed at {HELPER} (see owl/README.md)"
    return None


def apply_system(net: dict) -> str:
    """Set hostname and IP through the helper (blocking: run it off the main loop).
    Returns a status line; raises RuntimeError with the helper's message on failure."""
    why = system_support()
    if why:
        raise RuntimeError(why)
    args = ["network", "--hostname", net["hostname"], "--interface", net["interface"], "--mode", net["mode"]]
    if net["mode"] == "static":
        args += ["--ip", net["ip"], "--prefix", str(net["prefix"]), "--gateway", net["gateway"]]
    return run_helper(args)


def run_helper(args: list[str]) -> str:
    """Runs deploy/owl-admin as root. Returns its last output line; raises RuntimeError."""
    cmd = helper_command()
    if cmd is None:
        raise RuntimeError(f"the admin helper isn't installed at {HELPER} (see owl/README.md)")
    try:
        r = subprocess.run(cmd + args, capture_output=True, text=True, timeout=60)
    except (OSError, subprocess.SubprocessError) as e:
        raise RuntimeError(f"couldn't run the admin helper: {e}") from None
    text = (r.stdout.strip() or r.stderr.strip()).splitlines()
    if r.returncode != 0:
        msg = (r.stderr.strip() or r.stdout.strip() or f"exit code {r.returncode}").splitlines()[-1]
        if "password is required" in msg or "a terminal is required" in msg:
            msg = "sudo isn't set up for the admin helper (see owl/README.md)"
        raise RuntimeError(msg)
    return text[-1] if text else "applied"
