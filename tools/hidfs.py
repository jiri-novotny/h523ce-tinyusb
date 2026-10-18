#!/usr/bin/env python3
"""Talk to h523ce-tinyusb flash commands over /dev/hidrawN."""
import os, select, struct, sys, time

OPEN, WRITE, READ, CLOSE, BENCH, INFO = 6, 7, 8, 9, 10, 11
dev = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1].startswith("/dev") else "/dev/hidraw0"
fd = os.open(dev, os.O_RDWR)


def send(pkt):
    buf = bytes([0]) + bytes(pkt).ljust(64, b"\0")  # report id 0 + 64 B
    t = time.monotonic()
    os.write(fd, buf)
    return time.monotonic() - t


def recv(timeout):
    r, _, _ = select.select([fd], [], [], timeout)
    if not r:
        return None
    return os.read(fd, 64)


def drain():
    while (p := recv(0.05)) is not None:
        print("  drained", p[:8].hex())


def wait(hdr, timeout=5.0, verbose=True):
    end = time.monotonic() + timeout
    while (left := end - time.monotonic()) > 0:
        p = recv(left)
        if p is None:
            break
        if p[0] == hdr:
            return p
        if verbose:
            print("  other report", p[: 2 + p[1]].hex() if len(p) > 1 else p.hex())
    raise TimeoutError(f"no reply {hdr}")


def status(p):
    return struct.unpack_from("<b", p, 2)[0]


def info():
    t = time.monotonic()
    send([INFO, 0])
    p = wait(INFO)
    st, jedec, bs, bc, used = struct.unpack_from("<bIIII", p, 2)
    print(f"INFO st={st} jedec={jedec:06x} block={bs} count={bc} used={used} ({(time.monotonic()-t)*1000:.1f} ms)")


def fopen(name, write):
    n = name.encode()
    t = time.monotonic()
    send([OPEN, 1 + len(n), 1 if write else 0, *n])
    p = wait(OPEN)
    st, size = struct.unpack_from("<bI", p, 2)
    print(f"OPEN {name} w={write} st={st} size={size} ({(time.monotonic()-t)*1000:.1f} ms)")
    return st, size


def fclose(timeout=10):
    send([CLOSE, 0])
    p = wait(CLOSE, timeout)
    st, n = struct.unpack_from("<bI", p, 2)
    print(f"CLOSE st={st} bytes={n}")
    return st, n


def bench(size, chunk):
    send([BENCH, 6, *struct.pack("<IH", size, chunk)])
    p = wait(BENCH, 60 + size / 5000)
    st, w, r, e = struct.unpack_from("<bIII", p, 2)
    print(f"BENCH st={st} write={w} ms ({size/1024/(w/1000) if w else 0:.1f} KiB/s) "
          f"read={r} ms ({size/1024/(r/1000) if r else 0:.1f} KiB/s) errors={e}")


def usb_write(size):
    data = os.urandom(size)
    t0 = time.monotonic()
    fopen("usb.bin", True)
    slow = 0.0
    for off in range(0, size, 62):
        c = data[off:off + 62]
        dt = send([WRITE, len(c), *c])
        slow = max(slow, dt)
    st, n = fclose()
    dt = time.monotonic() - t0
    print(f"USB write {n} B in {dt*1000:.0f} ms ({n/1024/dt:.1f} KiB/s), max write() {slow*1000:.1f} ms")
    return data


def usb_read(expect=None):
    t0 = time.monotonic()
    st, size = fopen("usb.bin", False)
    send([READ, 0])
    buf = bytearray()
    while True:
        p = wait(READ, 5)
        if p[1] == 0:
            break
        buf += p[2:2 + p[1]]
    fclose()
    dt = time.monotonic() - t0
    ok = "" if expect is None else (" verify OK" if bytes(buf) == expect else " VERIFY FAIL")
    print(f"USB read {len(buf)}/{size} B in {dt*1000:.0f} ms ({len(buf)/1024/dt:.1f} KiB/s){ok}")


if __name__ == "__main__":
    cmds = [a for a in sys.argv[1:] if not a.startswith("/dev")]
    drain()
    for c in cmds:
        if c == "info":
            info()
        elif c.startswith("bench"):
            _, size, chunk = (c.split(":") + ["65536", "256"])[:3]
            bench(int(size), int(chunk))
        elif c.startswith("write"):
            size = int(c.split(":")[1]) if ":" in c else 65536
            last = usb_write(size)
        elif c == "read":
            usb_read(globals().get("last"))
        elif c == "open":
            fopen("usb.bin", True)
        elif c == "close":
            fclose()
