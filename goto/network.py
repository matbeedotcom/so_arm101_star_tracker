"""Network helpers: address enumeration + Wi-Fi AP control.

Targets Raspberry Pi OS Bookworm (NetworkManager). Falls back to a
pure-Python address probe so this module can be imported on dev
machines (macOS, non-NM Linux) without raising.

All functions are async-safe. They shell out to ``nmcli`` /``ip`` and
return structured dicts ready for JSON serialisation over BLE.
"""

from __future__ import annotations

import asyncio
import json
import logging
import shutil
import socket
from dataclasses import dataclass, asdict
from typing import Optional

log = logging.getLogger("network")


@dataclass
class NetInterface:
    name: str
    ip: str
    type: str        # "wifi" | "eth" | "ap" | "lo" | "other"


@dataclass
class APState:
    active: bool
    ssid: Optional[str] = None
    passphrase: Optional[str] = None
    iface: Optional[str] = None
    client_count: int = 0


# NetworkManager connection name used by start_ap. Distinct from the
# SSID so we can find / tear it down even if the user renames.
_AP_CONN_NAME = "star-tracker-hotspot"


# ── helpers ──

def _have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


async def _run(*args: str, timeout: float = 5.0) -> tuple[int, str, str]:
    proc = await asyncio.create_subprocess_exec(
        *args,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )
    try:
        stdout, stderr = await asyncio.wait_for(proc.communicate(), timeout=timeout)
    except asyncio.TimeoutError:
        proc.kill()
        await proc.wait()
        return 124, "", "timeout"
    return proc.returncode or 0, stdout.decode(errors="replace"), stderr.decode(errors="replace")


# ── addresses ──

async def list_addresses() -> list[NetInterface]:
    """Enumerate non-loopback IPv4 addresses.

    Tries ``ip -j addr`` first (fast, structured, JSON). Falls back to
    a socket probe if ``ip`` is unavailable or returns nothing useful.
    """
    out: list[NetInterface] = []
    debug: list[str] = []  # collected so failures show one line in INFO

    if _have("ip"):
        rc, raw, err = await _run("ip", "-j", "addr", "show")
        if rc == 0:
            try:
                doc = json.loads(raw)
            except json.JSONDecodeError as e:
                doc = []
                debug.append(f"json-parse: {e}")
            for iface in doc:
                name = iface.get("ifname", "")
                kind = _classify(name, iface)
                if kind == "lo":
                    continue
                for a in iface.get("addr_info", []):
                    if a.get("family") != "inet":
                        continue
                    ip = a.get("local")
                    if not ip:
                        continue
                    out.append(NetInterface(name=name, ip=ip, type=kind))
            if not out:
                debug.append(f"ip-j parsed {len(doc)} ifaces but no IPv4")
        else:
            debug.append(f"ip-j rc={rc} err={err.strip()[:80]}")
    else:
        debug.append("ip binary not found")

    if not out:
        fb = await _fallback_addresses()
        out.extend(fb)
        debug.append(f"fallback added {len(fb)}")

    log.info(
        "list_addresses: %d %s%s",
        len(out),
        ",".join(f"{n.name}={n.ip}({n.type})" for n in out) or "(none)",
        f"  [{'; '.join(debug)}]" if debug else "",
    )
    return out


def _classify(name: str, iface_doc: dict) -> str:
    if name == "lo":
        return "lo"
    flags = set(iface_doc.get("flags", []))
    if any(x.startswith("wlan") or x.startswith("wlp") for x in [name]):
        # Hotspot vs station can't be reliably distinguished from `ip`
        # output alone — image_server / nmcli decide that label.
        return "wifi"
    if name.startswith(("eth", "enp", "eno", "ens", "end")):
        return "eth"
    if "POINTOPOINT" in flags:
        return "vpn"
    return "other"


async def _fallback_addresses() -> list[NetInterface]:
    out: list[NetInterface] = []
    # Trick: open a UDP socket to a public IP without sending; the OS
    # picks the outbound interface so getsockname returns the LAN IP.
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        out.append(NetInterface(name="primary", ip=ip, type="other"))
    except OSError:
        pass
    try:
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if ip == "127.0.0.1":
                continue
            if not any(o.ip == ip for o in out):
                out.append(NetInterface(name=host, ip=ip, type="other"))
    except socket.gaierror:
        pass
    return out


async def list_addresses_dict() -> list[dict]:
    return [asdict(x) for x in await list_addresses()]


# ── AP control ──

async def _detect_ap_connection() -> Optional[dict]:
    """Return active AP details if our hotspot is up, else None."""
    if not _have("nmcli"):
        return None
    rc, raw, _ = await _run("nmcli", "-t", "-f", "NAME,TYPE,STATE,DEVICE",
                             "connection", "show", "--active")
    if rc != 0:
        return None
    for line in raw.splitlines():
        parts = line.split(":")
        if len(parts) < 4:
            continue
        name, ctype, state, device = parts[0], parts[1], parts[2], parts[3]
        if name == _AP_CONN_NAME and ctype.endswith("wireless") and state == "activated":
            return {"name": name, "iface": device}
    return None


async def ap_state() -> APState:
    detected = await _detect_ap_connection()
    if not detected:
        return APState(active=False)

    ssid = None
    passphrase = None
    rc, raw, _ = await _run("nmcli", "-s", "-t", "-f",
                             "802-11-wireless.ssid,802-11-wireless-security.psk",
                             "connection", "show", _AP_CONN_NAME)
    if rc == 0:
        for line in raw.splitlines():
            k, _, v = line.partition(":")
            if k.endswith(".ssid"):
                ssid = v or None
            elif k.endswith(".psk"):
                passphrase = v or None

    # Best-effort client count.
    client_count = 0
    iface = detected.get("iface")
    if iface and _have("iw"):
        rc, raw, _ = await _run("iw", "dev", iface, "station", "dump")
        if rc == 0:
            client_count = raw.count("Station ")

    return APState(
        active=True,
        ssid=ssid,
        passphrase=passphrase,
        iface=iface,
        client_count=client_count,
    )


async def start_ap(ssid: str, passphrase: str, iface: str = "wlan0") -> APState:
    """Bring up a Wi-Fi access point. Requires NetworkManager + root."""
    if not _have("nmcli"):
        raise RuntimeError("nmcli not available — install NetworkManager")
    if len(passphrase) < 8:
        raise ValueError("WPA2 passphrase must be ≥8 characters")

    # If a previous instance is hanging around, replace it cleanly.
    await _run("nmcli", "connection", "delete", _AP_CONN_NAME)  # ignore rc

    rc, _, err = await _run(
        "nmcli", "device", "wifi", "hotspot",
        "ifname", iface,
        "con-name", _AP_CONN_NAME,
        "ssid", ssid,
        "password", passphrase,
        timeout=10.0,
    )
    if rc != 0:
        raise RuntimeError(f"nmcli hotspot failed: {err.strip() or 'unknown error'}")

    # Pin the connection to autoconnect=no — we own its lifecycle via BLE.
    await _run("nmcli", "connection", "modify", _AP_CONN_NAME,
               "connection.autoconnect", "no")

    # Give NM a moment to settle, then return current state.
    await asyncio.sleep(0.5)
    return await ap_state()


async def stop_ap() -> APState:
    if not _have("nmcli"):
        raise RuntimeError("nmcli not available")
    await _run("nmcli", "connection", "down", _AP_CONN_NAME)
    await _run("nmcli", "connection", "delete", _AP_CONN_NAME)
    return APState(active=False)
