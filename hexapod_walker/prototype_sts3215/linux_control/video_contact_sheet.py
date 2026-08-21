#!/usr/bin/env python3
"""Extract timestamped frames/contact sheets from robot diagnosis videos."""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import imageio.v2 as imageio
from PIL import Image, ImageDraw, ImageFont


def _parse_times(text: str | None, *, duration: float | None,
                 every: float | None) -> list[float]:
    if text:
        out = []
        for part in text.split(","):
            part = part.strip()
            if not part:
                continue
            out.append(max(0.0, float(part)))
        return out
    if every is None:
        every = 8.0
    if duration is None or not math.isfinite(duration) or duration <= 0.0:
        return [0.0]
    n = int(math.floor(duration / every))
    vals = [round(i * every, 3) for i in range(n + 1)]
    if duration - vals[-1] > every * 0.4:
        vals.append(round(duration, 3))
    return vals


def _parse_crop(text: str | None) -> tuple[float, float, float, float] | None:
    if not text:
        return None
    vals = [float(x.strip()) for x in text.split(",")]
    if len(vals) != 4:
        raise ValueError("--crop must be left,top,right,bottom fractions")
    left, top, right, bottom = vals
    if not (0.0 <= left < right <= 1.0 and 0.0 <= top < bottom <= 1.0):
        raise ValueError("--crop fractions must satisfy 0<=l<r<=1, 0<=t<b<=1")
    return left, top, right, bottom


def _frame_at(reader, fps: float, t: float) -> Image.Image:
    idx = max(0, int(round(float(t) * fps)))
    frame = reader.get_data(idx)
    return Image.fromarray(frame).convert("RGB")


def _label(draw: ImageDraw.ImageDraw, xy: tuple[int, int], text: str) -> None:
    try:
        font = ImageFont.load_default()
    except Exception:
        font = None
    x, y = xy
    bbox = draw.textbbox((x, y), text, font=font)
    pad = 5
    draw.rectangle(
        (bbox[0] - pad, bbox[1] - pad, bbox[2] + pad, bbox[3] + pad),
        fill=(0, 0, 0))
    draw.text((x, y), text, fill=(255, 255, 255), font=font)


def make_contact_sheet(
        video: Path, *, out_dir: Path | None = None,
        times: list[float] | None = None, every: float | None = 8.0,
        crop: tuple[float, float, float, float] | None = None,
        thumb_width: int = 360, columns: int = 4) -> dict:
    reader = imageio.get_reader(str(video), "ffmpeg")
    try:
        meta = reader.get_meta_data()
        fps = float(meta.get("fps") or 30.0)
        duration = meta.get("duration")
        duration = float(duration) if duration is not None else None
        if times is None:
            times = _parse_times(None, duration=duration, every=every)
        if out_dir is None:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            out_dir = (Path.cwd() / "artifacts" / "video_frames"
                       / f"{video.stem}_{stamp}")
            if out_dir.exists():
                for i in range(1, 100):
                    candidate = out_dir.with_name(f"{out_dir.name}_{i:02d}")
                    if not candidate.exists():
                        out_dir = candidate
                        break
        out_dir.mkdir(parents=True, exist_ok=True)
        frame_paths = []
        tiles = []
        for t in times:
            im = _frame_at(reader, fps, t)
            if crop is not None:
                w, h = im.size
                l, top, r, b = crop
                im = im.crop((int(l * w), int(top * h),
                              int(r * w), int(b * h)))
            draw = ImageDraw.Draw(im)
            _label(draw, (16, 16), f"{t:.1f}s")
            frame_path = out_dir / f"{video.stem}_{t:06.2f}s.jpg"
            im.save(frame_path, quality=92)
            frame_paths.append(frame_path)

            tile = im.copy()
            scale = thumb_width / max(1, tile.width)
            tile = tile.resize(
                (thumb_width, max(1, int(tile.height * scale))),
                Image.Resampling.LANCZOS)
            tiles.append(tile)

        columns = max(1, int(columns))
        rows = math.ceil(len(tiles) / columns)
        gap = 12
        tile_h = max((t.height for t in tiles), default=1)
        sheet = Image.new(
            "RGB",
            (columns * thumb_width + (columns + 1) * gap,
             rows * tile_h + (rows + 1) * gap),
            (245, 245, 245))
        for i, tile in enumerate(tiles):
            x = gap + (i % columns) * (thumb_width + gap)
            y = gap + (i // columns) * (tile_h + gap)
            sheet.paste(tile, (x, y))
        sheet_path = out_dir / "contact_sheet.jpg"
        sheet.save(sheet_path, quality=92)
        return {
            "video": str(video),
            "out_dir": str(out_dir),
            "contact_sheet": str(sheet_path),
            "frames": [str(p) for p in frame_paths],
            "fps": fps,
            "duration": duration,
            "times": times,
        }
    finally:
        reader.close()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("video", type=Path)
    ap.add_argument("--out", type=Path, default=None,
                    help="output directory; default artifacts/video_frames/...")
    ap.add_argument("--times", default=None,
                    help="comma-separated timestamps in seconds")
    ap.add_argument("--every", type=float, default=8.0,
                    help="sample interval when --times is omitted")
    ap.add_argument("--crop", default=None,
                    help="optional left,top,right,bottom fractions")
    ap.add_argument("--thumb-width", type=int, default=360)
    ap.add_argument("--columns", type=int, default=4)
    args = ap.parse_args(argv)

    video = args.video.expanduser().resolve()
    times = _parse_times(args.times, duration=None, every=None) \
        if args.times else None
    result = make_contact_sheet(
        video, out_dir=args.out, times=times, every=args.every,
        crop=_parse_crop(args.crop), thumb_width=args.thumb_width,
        columns=args.columns)
    print(f"contact_sheet={result['contact_sheet']}")
    print(f"out_dir={result['out_dir']}")
    print(f"frames={len(result['frames'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
