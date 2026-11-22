"""
DM16 HDMI output: test pattern, and checking the screen with a capture card.

HDMI builds (Tang Console 138K) have a 320x240 RGB332 frame buffer, shown at
640x480 60 Hz. Programs write it with FBX / FBY / FBW (see dm16_host.py):
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


def rgb_to_rgb332(rgb):
    """Nearest RGB332 colour of each pixel."""
    rgb = rgb.astype(np.int64)
    r = np.abs(rgb[..., 0, None] - LEVELS3).argmin(-1)
    g = np.abs(rgb[..., 1, None] - LEVELS3).argmin(-1)
    b = np.abs(rgb[..., 2, None] - LEVELS2).argmin(-1)
    return (r << 5) | (g << 2) | b


CAL_PATH = os.path.expanduser("~/.cache/dm16/capture_cal.npy")


def load_cal():
    try:
        cal = np.load(CAL_PATH)
        return cal if cal.shape == (256, 3) else None
    except (OSError, ValueError):
        return None


def blocks_of(shot):
    """The capture averaged over each 2x2 block: (240, 320, 3) floats."""
    return shot.reshape(FB_H, 2, FB_W, 2, 3).astype(np.float64).mean(axis=(1, 3))


def classify(blocks, cal=None):
    """RGB332 value of each block: the nearest ideal colour, or with a
    calibration table the colour the card showed most like it."""
    if cal is None:
        return rgb_to_rgb332(blocks)
    d = ((blocks[..., None, :] - cal[None, None, :, :]) ** 2).sum(-1)
    return d.argmin(-1)


def compare(expected_fb, shot, show=5, cal=None):
    """Compare a capture with the expected frame buffer (RGB332, 240 x 320).
    Returns (exactly right fraction, mean colour error per channel)."""
    blocks = blocks_of(shot)
    got = classify(blocks, cal)
    exp = np.asarray(expected_fb, np.int64)
    ok = got == exp
    err = np.abs(blocks - rgb332_to_rgb(exp)).mean()
    bad = np.argwhere(~ok)
    for y, x in bad[:show]:
        print(f"  pixel ({x},{y}): screen {int(got[y, x]):02x} "
              f"(rgb {blocks[y, x].round().astype(int).tolist()}), expected {int(exp[y, x]):02x}")
    return ok.mean(), err


# ------------------------------------------------------------------------------
# Test pattern: tile mode, one 8x8 (on 64 PEs) tile per FBW, row by row.
# PE p in tile k writes (37 p + 3 k) mod 256: every colour, no two
# neighbours alike, so a misplaced pixel or a swapped channel shows.
# ------------------------------------------------------------------------------
def pattern_program():
    tiles = (FB_W // COLS) * (FB_H // ROWS)
    return [
        ctx(CTX_ALWAYS),
        encode_instr(OP_LDI), put(1),                  # r1 = 3 k
        FBX(0), FBY(0),
        LOOP(tiles),
            alu(ALU_MOV, SRC_ID), alu(ALU_MUL, SRC_IMM, 37), alu(ALU_ADD, SRC_R, 1),
            FBW(0),
            get(1), alu(ALU_ADD, SRC_IMM, 3), put(1),
        ENDL,
    ]


def pattern_expected():
    fb = np.zeros((FB_H, FB_W), np.int64)
    tiles_x = FB_W // COLS
    for k in range((FB_W // COLS) * (FB_H // ROWS)):
        ty, tx = divmod(k, tiles_x)
        for p in range(NUM_PE):
            fb[ty * ROWS + p // COLS, tx * COLS + p % COLS] = (37 * p + 3 * k) & 255
    return fb


def check_screen(device, expected_fb, png=None, tries=3, cal=None):
    """Grab the screen and compare it with expected_fb; True if every pixel
    matches (using the saved calibration, if there is one)."""
    if cal is None:
        cal = load_cal()
    for attempt in range(tries):
        shot = grab(device)
        if png:
            save_png(shot, png)
        frac, err = compare(expected_fb, shot, cal=cal)
        print(f"capture: {100 * frac:.3f}% of pixels right "
              f"({'calibrated' if cal is not None else 'uncalibrated'}), "
              f"mean colour error {err:.1f}/255")
        if frac == 1.0:
            return True
    return False


def calibrate(device, png=None):
    """With the test pattern on screen: raw check, then the per-colour
    calibration table, then the check again with it. True if it all matches."""
    exp = pattern_expected()
    shot = grab(device)
    if png:
        save_png(shot, png)
    raw, err = compare(exp, shot, show=0)
    blocks = blocks_of(shot)
    cal = np.zeros((256, 3))
    for v in range(256):
        cal[v] = np.median(blocks[exp == v], axis=0)
    shifted = [v for v in range(256)
               if np.abs(cal[v] - rgb332_to_rgb(np.array([[v]]))[0, 0]).max() > 12]
    good, _ = compare(exp, shot, cal=cal)
    print(f"capture: {100 * raw:.2f}% of pixels exactly right against ideal colours "
          f"(mean colour error {err:.1f}/255)")
    if shifted:
        print("  colours the capture card shows shifted: " + ", ".join(
            f"{v:02x} as {cal[v].round().astype(int).tolist()}" for v in shifted))
    print(f"capture: {100 * good:.3f}% of pixels right against the card's own colours")
    if good == 1.0:
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        np.save(CAL_PATH, cal)
        print(f"calibration saved to {CAL_PATH}")
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
        cfg = read_config(ser)
        if not cfg or not cfg.get("has_hdmi"):
            print("this bitstream has no HDMI output", file=sys.stderr)
            return 1
        upload_and_run(ser, pattern_program())
        print("test pattern drawn")
    if args.capture:
        if not device:
            ap.error("no capture device found (use --video)")
        ok = calibrate(device, args.png)
        print("PASS" if ok else "FAIL")
        return 0 if ok else 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
