#!/usr/bin/env python3
"""
Arty A7 <-> PC webcam link with FPGA auto-exposure (AE) / auto-white-balance (AWB).

  PC   -> FPGA : A5 5A, then 160*120 RGB565 pixels (high byte first)
  FPGA -> PC   : 5A A5                      (ACK)
  FPGA -> PC   : 5A A5, then the corrected frame

FPGA switches:  SW0 = negative   SW1 = AWB (gray-world)   SW2 = AE (auto exposure)

Usage
  python webcam_fpga_test.py --list            show serial ports
  python webcam_fpga_test.py --selftest        no webcam: test frames, verifies the FPGA output bit-exactly
  python webcam_fpga_test.py                   webcam mode  (s = send one frame, a = auto-send, q = quit)
  python webcam_fpga_test.py --port COM7       choose the port yourself

For every returned frame the script computes what the FPGA *should* have produced
(bit-exact integer model of the Verilog) for all 8 switch combinations and tells you
which one matches. It also computes a floating-point reference of the same algorithm
so you can see the fixed-point FPGA result agrees with the ideal one.
"""
import argparse
import sys
import time

import cv2
import numpy as np

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is missing.  Run:  pip install pyserial")
if not hasattr(serial, "Serial"):
    sys.exit("The wrong 'serial' package is installed.\n"
             "Run:  pip uninstall serial   and then   pip install pyserial")

WIDTH, HEIGHT = 160, 120
FRAME_BYTES = WIDTH * HEIGHT * 2
HELLO = b"\xA5\x5A"      # PC   -> FPGA
REPLY = b"\x5A\xA5"      # FPGA -> PC
DEFAULT_BAUD = 921600

TARGET = 120             # auto-exposure target mean level - must equal the Verilog parameter TARGET
MAX_GAIN = 32768         # 8.0 in Q4.12

NO_ACK_HELP = """
No valid ACK from the FPGA.  Check in this order:
  1. Press 'h' (or run --selftest) and watch LED LD6 (led[2]) on the board.
       LD6 never lights -> the FPGA is not receiving anything:
           * XDC must be  uart_rx -> A9   and   uart_tx -> D10   (use the corrected .xdc)
           * wrong COM port  (python webcam_fpga_test.py --list)
           * another program (Tera Term, PuTTY, Arduino IDE ...) has the port open
           * bitstream was not programmed after rebuilding
       LD6 lights but still no ACK -> baud mismatch (FPGA is built for 921600) or old bitstream.
  2. An old bitstream stays stuck after an aborted transfer -> re-program the FPGA.
"""


# ----------------------------------------------------------------------------- pixel formats
def bgr_to_rgb565(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    r = (rgb[:, :, 0].astype(np.uint16) >> 3) & 0x1F
    g = (rgb[:, :, 1].astype(np.uint16) >> 2) & 0x3F
    b = (rgb[:, :, 2].astype(np.uint16) >> 3) & 0x1F
    packed = (r << 11) | (g << 5) | b
    return packed.astype(">u2").tobytes()          # big endian: high byte first


def rgb565_to_bgr(data):
    r8, g8, b8 = expand565(data)
    shape = (HEIGHT, WIDTH)
    return cv2.merge((b8.reshape(shape).astype(np.uint8),
                      g8.reshape(shape).astype(np.uint8),
                      r8.reshape(shape).astype(np.uint8)))


def expand565(payload):
    """RGB565 bytes -> three flat int64 arrays of 8-bit expanded channel values (as in the FPGA)."""
    px = np.frombuffer(payload, dtype=">u2").astype(np.int64)
    r5, g6, b5 = (px >> 11) & 31, (px >> 5) & 63, px & 31
    return (r5 << 3) | (r5 >> 2), (g6 << 2) | (g6 >> 4), (b5 << 3) | (b5 >> 2)


# ----------------------------------------------------------------------------- FPGA reference models
def fpga_gains(r8, g8, b8, awb, ae, target=TARGET):
    """Integer gains (Q4.12) exactly as the Verilog divider produces them."""
    n = r8.size
    sums = [int(r8.sum()), int(g8.sum()), int(b8.sum())]
    sgm = (sum(sums) * 21845) >> 16                        # sum/3 without a divider
    num = ((target * n) if ae else sgm) * 4096
    gains = []
    for s in sums:
        den = s if awb else sgm
        q = (num // den) if den else (1 << 36) - 1         # den = 0 -> all ones (then clamped)
        gains.append(min(q, MAX_GAIN))
    return gains


def fpga_model(payload, awb, ae, neg):
    """Bit-exact model of the FPGA output for the given switch settings."""
    r8, g8, b8 = expand565(payload)
    gr, gg, gb = fpga_gains(r8, g8, b8, awb, ae)

    def ch(v, g):
        return np.minimum((v * g + 2048) >> 12, 255)

    r, g, b = ch(r8, gr), ch(g8, gg), ch(b8, gb)
    out = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    if neg:
        out = (~out) & 0xFFFF
    return out.astype(">u2").tobytes()


def float_reference(payload, awb, ae):
    """Ideal floating-point gray-world / auto-exposure result (BGR image) for comparison."""
    r8, g8, b8 = (c.astype(np.float64) for c in expand565(payload))
    means = [c.mean() for c in (r8, g8, b8)]
    gray = sum(means) / 3.0
    tgt = TARGET if ae else gray
    gains = [min(tgt / (m if awb else gray) if (m if awb else gray) else 8.0, 8.0) for m in means]
    chans = [np.clip(c * g, 0, 255) for c, g in zip((r8, g8, b8), gains)]
    shape = (HEIGHT, WIDTH)
    return cv2.merge([chans[2].reshape(shape).astype(np.uint8),
                      chans[1].reshape(shape).astype(np.uint8),
                      chans[0].reshape(shape).astype(np.uint8)])


def mode_name(mode):
    neg, awb, ae = mode & 1, (mode >> 1) & 1, (mode >> 2) & 1
    parts = [n for n, f in (("AWB", awb), ("AE", ae)) if f]
    name = "+".join(parts) if parts else "no correction"
    return name + (" + NEGATIVE" if neg else "")


def matching_modes(payload, got):
    """All switch combinations (0..7, bit0=neg bit1=awb bit2=ae) whose bit-exact output equals 'got'.
    More than one can match when a frame is already balanced / correctly exposed."""
    return [mode for mode in range(8)
            if fpga_model(payload, bool((mode >> 1) & 1), bool((mode >> 2) & 1), bool(mode & 1)) == got]


def identify(payload, got):
    modes = matching_modes(payload, got)
    return modes[0] if modes else None


def channel_means(payload):
    return [float(c.mean()) for c in expand565(payload)]


def fmt_means(m):
    return f"R={m[0]:5.1f} G={m[1]:5.1f} B={m[2]:5.1f}"


# ----------------------------------------------------------------------------- serial helpers
def list_serial_ports():
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Is the Arty plugged in (micro-USB 'PROG/UART' port)?")
    for p in ports:
        vid = f"{p.vid:04X}" if p.vid is not None else "----"
        pid = f"{p.pid:04X}" if p.pid is not None else "----"
        print(f"  {p.device:<10} VID:PID={vid}:{pid}  {p.description}")
    return ports


def find_arty_port():
    cands = []
    for p in list_ports.comports():
        text = f"{p.manufacturer or ''} {p.description or ''}".lower()
        if p.vid == 0x0403 or "digilent" in text or "ftdi" in text:
            cands.append(p.device)
    return cands[0] if len(cands) == 1 else None


def read_exact(ser, count, timeout_seconds=5.0):
    result = bytearray()
    deadline = time.monotonic() + timeout_seconds
    while len(result) < count and time.monotonic() < deadline:
        chunk = ser.read(count - len(result))
        if chunk:
            result.extend(chunk)
    return bytes(result)


def handshake(ser, tries=3, timeout=1.0, verbose=True):
    for attempt in range(1, tries + 1):
        ser.reset_input_buffer()
        ser.write(HELLO)
        ser.flush()
        ack = read_exact(ser, 2, timeout)
        if ack == REPLY:
            return True
        if verbose:
            print(f"  handshake attempt {attempt}/{tries}: got "
                  f"{ack.hex(' ') if ack else 'nothing'} (expected 5a a5)")
        time.sleep(0.25)          # > the FPGA's 100 ms receive watchdog
    return False


def exchange(ser, payload, ack_timeout=1.0, hdr_timeout=5.0, frame_timeout=8.0):
    if not handshake(ser, timeout=ack_timeout):
        return None, "no ACK"
    ser.write(payload)
    ser.flush()
    header = read_exact(ser, 2, hdr_timeout)
    if header != REPLY:
        return None, f"no output-frame header (got {header.hex(' ') if header else 'nothing'})"
    data = read_exact(ser, FRAME_BYTES, frame_timeout)
    if len(data) != FRAME_BYTES:
        return None, f"incomplete frame: {len(data)} of {FRAME_BYTES} bytes"
    return data, "ok"


def verdict(payload, got):
    """Returns (ok, text, list_of_matching_modes)."""
    modes = matching_modes(payload, got)
    if len(modes) == 1:
        return True, f"bit-exact match: {mode_name(modes[0])}", modes
    if modes:
        names = " / ".join(mode_name(m) for m in modes)
        return True, f"bit-exact match (this frame cannot tell which: {names})", modes
    a = np.frombuffer(payload, dtype=np.uint8)
    b = np.frombuffer(got, dtype=np.uint8)
    if np.array_equal(a[1:], b[:-1]):
        return False, "MISMATCH: output shifted by one byte (first byte lost)", []
    bad = int(np.count_nonzero(a != b))
    return False, f"MISMATCH: matches none of the 8 switch modes ({bad} bytes differ from the input)", []


# ----------------------------------------------------------------------------- test patterns
def test_pattern():
    x = np.linspace(0, 255, WIDTH, dtype=np.uint8)
    y = np.linspace(0, 255, HEIGHT, dtype=np.uint8)
    img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    img[:, :, 2] = x[None, :]
    img[:, :, 1] = y[:, None]
    img[:, :, 0] = 255 - x[None, :]
    img[10:20, :] = (255, 255, 255)
    img[HEIGHT - 20:HEIGHT - 10, :] = (0, 0, 0)
    return img


def dark_warm_scene():
    """A dim, orange-tinted scene: ideal for seeing AWB and AE work."""
    yy, xx = np.mgrid[0:HEIGHT, 0:WIDTH]
    base = (90 + 60 * np.sin(xx / 17.0) * np.cos(yy / 13.0)).clip(0, 255)
    img = np.zeros((HEIGHT, WIDTH, 3), np.float64)
    img[:, :, 2] = base * 0.60        # R (BGR order)
    img[:, :, 1] = base * 0.38        # G
    img[:, :, 0] = base * 0.17        # B  -> strong orange cast, dark
    img[40:80, 50:110] = (30, 70, 120)  # a reddish-brown patch
    return img.clip(0, 255).astype(np.uint8)


def selftest(ser):
    print("1) handshake ...")
    if not handshake(ser):
        print(NO_ACK_HELP)
        return False
    print("   ACK received - UART link works in both directions.")
    time.sleep(0.3)

    rng = np.random.default_rng(1)
    tests = [("colour gradient", bgr_to_rgb565(test_pattern())),
             ("dark orange-tinted scene", bgr_to_rgb565(dark_warm_scene())),
             ("random bytes (incl. A5/5A values)", rng.integers(0, 256, FRAME_BYTES, dtype=np.uint8).tobytes())]
    ok, candidates = True, set(range(8))
    for i, (name, payload) in enumerate(tests, start=2):
        print(f"{i}) frame: {name} ...")
        t0 = time.monotonic()
        data, msg = exchange(ser, payload)
        dt = time.monotonic() - t0
        if data is None:
            print(f"   FAIL: {msg}")
            ok = False
            continue
        good, text, modes = verdict(payload, data)
        print(f"   {text}   ({dt:.2f} s)")
        if good:
            candidates &= set(modes)
            if "random" not in name:
                print(f"   channel means  in: {fmt_means(channel_means(payload))}")
                print(f"                 out: {fmt_means(channel_means(data))}")
        ok &= good
    if ok and not candidates:
        print("   Frames were returned in different modes - did a switch move during the test?")
        ok = False
    elif ok:
        print("   => FPGA is in mode:", " / ".join(mode_name(m) for m in sorted(candidates)))
    print("\nSELFTEST", "PASSED" if ok else "FAILED")
    return ok


# ----------------------------------------------------------------------------- webcam mode
def open_camera(index):
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW) if sys.platform.startswith("win") else cv2.VideoCapture(index)
    if not cap.isOpened():
        cap = cv2.VideoCapture(index)
    return cap


def show_panels(sent, fpga, ref, ref_label):
    up = lambda im: cv2.resize(im, (WIDTH * 2, HEIGHT * 2), interpolation=cv2.INTER_NEAREST)
    panels = [up(sent), up(fpga), up(ref)]
    for im, label in zip(panels, ("sent", "FPGA result", ref_label)):
        cv2.putText(im, label, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.imshow("sent | FPGA result | PC float reference", cv2.hconcat(panels))


def send_frame(ser, frame):
    small = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
    payload = bgr_to_rgb565(small)
    t0 = time.monotonic()
    data, msg = exchange(ser, payload)
    if data is None:
        print("Transfer failed:", msg)
        if msg == "no ACK":
            print(NO_ACK_HELP)
        return
    good, text, modes = verdict(payload, data)
    mode = modes[0] if modes else None
    print(f"{text}   ({time.monotonic() - t0:.2f} s)")
    print(f"   means in: {fmt_means(channel_means(payload))}   out: {fmt_means(channel_means(data))}")
    sent_img, fpga_img = rgb565_to_bgr(payload), rgb565_to_bgr(data)
    if mode is not None:
        awb, ae = bool((mode >> 1) & 1), bool((mode >> 2) & 1)
        ref = float_reference(payload, awb, ae)
        if mode & 1:
            ref = 255 - ref
        err = float(np.mean(np.abs(ref.astype(int) - fpga_img.astype(int))))
        print(f"   FPGA vs ideal float reference: mean abs error {err:.2f} grey levels")
        show_panels(sent_img, fpga_img, ref, "PC float ref")
    else:
        show_panels(sent_img, fpga_img, sent_img, "(no reference)")


def webcam_mode(ser, cam_index):
    cap = open_camera(cam_index)
    if not cap.isOpened():
        sys.exit("Could not open the webcam. Close other camera apps / check permissions, or try --camera 1")
    print("Click the camera window, then:  s = send one frame   a = auto-send on/off   h = handshake   q = quit")
    print("Board switches: SW0 negative | SW1 auto white balance | SW2 auto exposure")
    auto = False
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Could not read a webcam frame.")
                break
            frame = cv2.flip(frame, 1)
            cv2.imshow("Webcam input (s=send, a=auto, q=quit)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("a"):
                auto = not auto
                print("auto-send", "ON" if auto else "OFF")
            elif key == ord("h"):
                print("handshake:", "ACK OK" if handshake(ser, tries=1) else "no ACK")
                time.sleep(0.25)
            if key == ord("s") or auto:
                send_frame(ser, frame)
    finally:
        cap.release()
        cv2.destroyAllWindows()


# ----------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="Arty A7 webcam <-> FPGA AE/AWB link")
    ap.add_argument("--port", help="serial port, e.g. COM7 or /dev/ttyUSB1 (auto-detected if omitted)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--camera", type=int, default=0, help="webcam index")
    ap.add_argument("--selftest", action="store_true", help="test the link without a webcam")
    ap.add_argument("--list", action="store_true", help="list serial ports and exit")
    args = ap.parse_args()

    if args.list:
        list_serial_ports()
        return

    port = args.port or find_arty_port()
    if port is None:
        print("Could not auto-detect the Arty. Available ports:")
        list_serial_ports()
        sys.exit("Re-run with  --port COMx")
    print(f"Using {port} @ {args.baud} baud")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.2, write_timeout=5)
    except serial.SerialException as exc:
        sys.exit(f"Cannot open {port}: {exc}\nClose any other program using the port, or pick another with --port.")

    with ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        if args.selftest:
            sys.exit(0 if selftest(ser) else 1)
        webcam_mode(ser, args.camera)


if __name__ == "__main__":
    main()  
    
