"""
DM16 HDMI output: test pattern, and checking the screen with a capture card.

HDMI builds have a 320x240 frame buffer, shown at 640x480 60 Hz: 256 colours
(RGB332) on the Tang Console 138K, 16 (the CGA palette, the low 4 bits of
each byte) on the Tang Nano 20K. The configuration readback says which. Programs write it with FBX / FBY / FBW (see dm16_host.py):
FBW copies the low byte of every PE's ACC to the frame buffer at once.

  python3 dm16_hdmi.py                      # draw the test pattern
  python3 dm16_hdmi.py --capture            # ... and check it through a capture card
  python3 dm16_hdmi.py --grab shot.png      # save what the capture card sees

The capture card is a USB (UVC) HDMI grabber, found by name or given with
--video /dev/videoN. Its YUV 4:2:2 video blurs colours a little, so the check
averages each 2x2 block of the 640x480 picture (one frame-buffer pixel) and
rounds it to the nearest RGB332 colour. Some cards also shift a few colours
(an EVGA XR1 Lite shows the darkest red, 0x20 = (36, 0, 0), as (18, 0, 0)),
so the test pattern, which has every colour about 300 times, doubles as a
calibration: --capture saves what the card shows for each of the 256 colours
(~/.cache/dm16/capture_cal.npy), and later checks (dm16_mandelbrot.py --hdmi
--capture) match against that.
"""
import argparse
import glob
import os
import subprocess
import sys

import numpy as np

from dm16_host import (
    DEFAULT_PORT, open_dm16, read_config, NUM_PE, COLS,
    ALU_ADD, ALU_MUL, ALU_MOV, SRC_IMM, SRC_ID, SRC_R, CTX_ALWAYS,
    LOOP, ENDL, FBX, FBY, FBW, FB_W, FB_H,
    alu, ctx, put, get, encode_instr, OP_LDI, upload_and_run,
)

ROWS = NUM_PE // COLS
CAPTURE_NAMES = ("capture", "hdmi", "xr1", "cam link", "usb video")


# ------------------------------------------------------------------------------
# Capture card
# ------------------------------------------------------------------------------
def find_capture_device():
    """First /dev/videoN whose name looks like an HDMI capture device."""
    for path in sorted(glob.glob("/sys/class/video4linux/video*"),
                       key=lambda p: int(p.rsplit("video", 1)[1])):
        try:
            name = open(os.path.join(path, "name")).read().strip().lower()
        except OSError:
            continue
        if any(k in name for k in CAPTURE_NAMES):
            return "/dev/" + os.path.basename(path)
    return None


def grab(device, skip=10):
    """One 640x480 frame as an (480, 640, 3) uint8 RGB array. The first
    frames after opening are often stale or dark, so `skip` are dropped."""
    cmd = ["ffmpeg", "-loglevel", "error", "-f", "v4l2", "-input_format", "yuyv422",
           "-video_size", "640x480", "-i", device,
           "-vf", f"select=gte(n\\,{skip})", "-frames:v", "1",
           "-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    out = subprocess.run(cmd, capture_output=True, timeout=30)
    if out.returncode or len(out.stdout) != 640 * 480 * 3:
        raise RuntimeError(f"capture failed: {out.stderr.decode(errors='replace').strip()}")
    return np.frombuffer(out.stdout, np.uint8).reshape(480, 640, 3)


def save_png(rgb, path):
    from PIL import Image
    Image.fromarray(rgb).save(path)


# ------------------------------------------------------------------------------
# RGB332
# ------------------------------------------------------------------------------
LEVELS3 = np.array([(v << 5 | v << 2 | v >> 1) for v in range(8)])   # as dm16_hdmi.v
LEVELS2 = np.array([v * 0x55 for v in range(4)])


def rgb332_to_rgb(fb):
    """(H, W) RGB332 -> (H, W, 3) uint8, expanded the way the hardware does."""
    fb = np.asarray(fb, np.int64)
    return np.stack([LEVELS3[fb >> 5], LEVELS3[(fb >> 2) & 7], LEVELS2[fb & 3]], -1).astype(np.uint8)


# The 16-colour palette (Nano 20K), as in dm16_hdmi.v
CGA = np.array([(0x00, 0x00, 0x00), (0x00, 0x00, 0xAA), (0x00, 0xAA, 0x00), (0x00, 0xAA, 0xAA),
                (0xAA, 0x00, 0x00), (0xAA, 0x00, 0xAA), (0xAA, 0x55, 0x00), (0xAA, 0xAA, 0xAA),
                (0x55, 0x55, 0x55), (0x55, 0x55, 0xFF), (0x55, 0xFF, 0x55), (0x55, 0xFF, 0xFF),
                (0xFF, 0x55, 0x55), (0xFF, 0x55, 0xFF), (0xFF, 0xFF, 0x55), (0xFF, 0xFF, 0xFF)])


def ideal_palette(ncol):
    """What each pixel value looks like on screen: (ncol, 3) RGB."""
    if ncol == 16:
        return CGA.astype(np.float64)
    return rgb332_to_rgb(np.arange(256)[None, :])[0].astype(np.float64)


def screen_colors(cfg):
    """Colours in the bitstream's frame buffer (0 without HDMI)."""
    if not cfg or not cfg.get("has_hdmi"):
        return 0
    return cfg.get("fb_colors", 256)


def cal_path(ncol):
    name = "capture_cal.npy" if ncol == 256 else f"capture_cal{ncol}.npy"
    return os.path.expanduser("~/.cache/dm16/" + name)


def load_cal(ncol=256):
    try:
        cal = np.load(cal_path(ncol))
        return cal if cal.shape == (ncol, 3) else None
    except (OSError, ValueError):
        return None


def blocks_of(shot):
    """The capture averaged over each 2x2 block: (240, 320, 3) floats."""
    return shot.reshape(FB_H, 2, FB_W, 2, 3).astype(np.float64).mean(axis=(1, 3))


def classify(blocks, table):
    """Pixel value of each block: the entry of `table` (the ideal colours,
    or a calibration of what the card shows) nearest to it."""
    d = ((blocks[..., None, :] - table[None, None, :, :]) ** 2).sum(-1)
    return d.argmin(-1)


def compare(expected_fb, shot, show=5, cal=None, ncol=256):
    """Compare a capture with the expected frame buffer (pixel values,
    240 x 320). Returns (exactly right fraction, mean colour error)."""
    ideal = ideal_palette(ncol)
    blocks = blocks_of(shot)
    got = classify(blocks, ideal if cal is None else cal)
    exp = np.asarray(expected_fb, np.int64)
    ok = got == exp
    err = np.abs(blocks - ideal[exp]).mean()
    bad = np.argwhere(~ok)
    for y, x in bad[:show]:
        print(f"  pixel ({x},{y}): screen {int(got[y, x]):02x} "
              f"(rgb {blocks[y, x].round().astype(int).tolist()}), expected {int(exp[y, x]):02x}")
    return ok.mean(), err


# ------------------------------------------------------------------------------
# Test pattern: tile mode, one COLS x ROWS tile per FBW (8 x 8 on the
# Console, 4 x 2 on the 20K), row by row. PE p in tile k, on tile row ty,
# writes (37 p + 3 k + ty) mod 256: every colour, few neighbours alike, so a
# misplaced pixel or a swapped channel shows. (With 16 colours only the low
# 4 bits show; the "+ ty" keeps rows of tiles different even when a row is a
# multiple of 16 tiles long.)
# ------------------------------------------------------------------------------
def pattern_program():
    return [
        ctx(CTX_ALWAYS),
        encode_instr(OP_LDI), put(1),                  # r1 = 3 k + ty
        FBX(0), FBY(0),
        LOOP(FB_H // ROWS),
            LOOP(FB_W // COLS),
                alu(ALU_MOV, SRC_ID), alu(ALU_MUL, SRC_IMM, 37), alu(ALU_ADD, SRC_R, 1),
                FBW(0),
                get(1), alu(ALU_ADD, SRC_IMM, 3), put(1),
            ENDL,
            get(1), alu(ALU_ADD, SRC_IMM, 1), put(1),
        ENDL,
    ]


def pattern_expected(ncol=256):
    fb = np.zeros((FB_H, FB_W), np.int64)
    tiles_x = FB_W // COLS
    for k in range((FB_W // COLS) * (FB_H // ROWS)):
        ty, tx = divmod(k, tiles_x)
        for p in range(NUM_PE):
            fb[ty * ROWS + p // COLS, tx * COLS + p % COLS] = (37 * p + 3 * k + ty) & (ncol - 1)
    return fb


def check_screen(device, expected_fb, png=None, tries=3, cal=None, ncol=256):
    """Grab the screen and compare it with expected_fb; True if every pixel
    matches (using the saved calibration, if there is one)."""
    if cal is None:
        cal = load_cal(ncol)
    for attempt in range(tries):
        shot = grab(device)
        if png:
            save_png(shot, png)
        frac, err = compare(expected_fb, shot, cal=cal, ncol=ncol)
        print(f"capture: {100 * frac:.3f}% of pixels right "
              f"({'calibrated' if cal is not None else 'uncalibrated'}), "
              f"mean colour error {err:.1f}/255")
        if frac == 1.0:
            return True
    return False


def calibrate(device, png=None, ncol=256):
    """With the test pattern on screen: raw check, then the per-colour
    calibration table, then the check again with it. True if it all matches."""
    exp = pattern_expected(ncol)
    ideal = ideal_palette(ncol)
    shot = grab(device)
    if png:
        save_png(shot, png)
    raw, err = compare(exp, shot, show=0, ncol=ncol)
    blocks = blocks_of(shot)
    cal = np.zeros((ncol, 3))
    for v in range(ncol):
        cal[v] = np.median(blocks[exp == v], axis=0)
    shifted = [v for v in range(ncol) if np.abs(cal[v] - ideal[v]).max() > 12]
    good, _ = compare(exp, shot, cal=cal, ncol=ncol)
    print(f"capture: {100 * raw:.2f}% of pixels exactly right against ideal colours "
          f"(mean colour error {err:.1f}/255)")
    if shifted:
        print("  colours the capture card shows shifted: " + ", ".join(
            f"{v:02x} as {cal[v].round().astype(int).tolist()}" for v in shifted))
    print(f"capture: {100 * good:.3f}% of pixels right against the card's own colours")
    if good == 1.0:
        os.makedirs(os.path.dirname(cal_path(ncol)), exist_ok=True)
        np.save(cal_path(ncol), cal)
        print(f"calibration saved to {cal_path(ncol)}")
        return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--video", help="capture device (default: found by name)")
    ap.add_argument("--capture", action="store_true", help="check the screen with the capture card")
    ap.add_argument("--grab", metavar="PNG", help="only save a capture")
    ap.add_argument("--png", help="with --capture: save the capture here too")
    args = ap.parse_args()

    device = args.video or find_capture_device()
    if args.grab:
        if not device:
            ap.error("no capture device found (use --video)")
        save_png(grab(device), args.grab)
        print(f"saved {args.grab} from {device}")
        return 0

    with open_dm16(args.port) as ser:
        ncol = screen_colors(read_config(ser))
        if not ncol:
            print("this bitstream has no HDMI output", file=sys.stderr)
            return 1
        upload_and_run(ser, pattern_program(), timeout=30)
        print(f"test pattern drawn ({ncol} colours)")
    if args.capture:
        if not device:
            ap.error("no capture device found (use --video)")
        ok = calibrate(device, args.png, ncol)
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
