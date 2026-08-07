#!/usr/bin/env python3
"""Generate a printable US Letter sheet of 16 AprilTag tag36h11 markers (IDs 0–15).

Renders official AprilRobotics bit layouts (same codes / bit_x,y as tag36h11.c).
Output is SVG — open and Print at 100% / Actual size (no "fit to page").
"""

from __future__ import annotations

import argparse
from pathlib import Path

# tag36h11 codes[0..15] from AprilRobotics/apriltag tag36h11.c
CODES_0_15 = [
    0x0000000D7E00984B,
    0x0000000DDA664CA7,
    0x0000000DC4A1C821,
    0x0000000E17B470E9,
    0x0000000EF91D01B1,
    0x0000000F429CDD73,
    0x000000005DA29225,
    0x00000001106CBA43,
    0x0000000223BED79D,
    0x000000021F51213C,
    0x000000033EB19CA6,
    0x00000003F76EB0F8,
    0x0000000469A97414,
    0x000000045DCFE0B0,
    0x00000004A6465F72,
    0x000000051801DB96,
]

# bit_x / bit_y from tag36h11_create(); nbits=36, width_at_border=8, total_width=10
BIT_X = [
    1, 2, 3, 4, 5, 2, 3, 4, 3, 6, 6, 6, 6, 6, 5, 5, 5, 4,
    6, 5, 4, 3, 2, 5, 4, 3, 4, 1, 1, 1, 1, 1, 2, 2, 2, 3,
]
BIT_Y = [
    1, 1, 1, 1, 1, 2, 2, 2, 3, 1, 2, 3, 4, 5, 2, 3, 4, 3,
    6, 6, 6, 6, 6, 5, 5, 5, 4, 6, 5, 4, 3, 2, 5, 4, 3, 4,
]
NBITS = 36
WIDTH_AT_BORDER = 8
TOTAL_WIDTH = 10  # includes 1-cell white quiet zone each side


def tag_grid(code: int) -> list[list[int]]:
    """Return TOTAL_WIDTH×TOTAL_WIDTH grid; 0=black, 1=white. Matches apriltag_to_image()."""
    im = [[0] * TOTAL_WIDTH for _ in range(TOTAL_WIDTH)]
    # Outer quiet-zone ring (white)
    for i in range(TOTAL_WIDTH):
        im[0][i] = 1
        im[TOTAL_WIDTH - 1][i] = 1
        im[i][0] = 1
        im[i][TOTAL_WIDTH - 1] = 1
    border_start = (TOTAL_WIDTH - WIDTH_AT_BORDER) // 2  # 1
    for i in range(NBITS):
        if code & (1 << (NBITS - i - 1)):
            x = BIT_X[i] + border_start
            y = BIT_Y[i] + border_start
            im[y][x] = 1
    return im


def svg_tag(code: int, x: float, y: float, size: float, tag_id: int) -> list[str]:
    """SVG fragments for one tag; (x,y) top-left of quiet-zone square, size = outer side."""
    grid = tag_grid(code)
    cell = size / TOTAL_WIDTH
    parts = [
        f'<g id="tag-{tag_id}">',
        f'<rect x="{x:.4f}" y="{y:.4f}" width="{size:.4f}" height="{size:.4f}" fill="#fff"/>',
    ]
    # Merge black cells into rects (row runs) for smaller SVG
    for r in range(TOTAL_WIDTH):
        c = 0
        while c < TOTAL_WIDTH:
            if grid[r][c] == 1:
                c += 1
                continue
            c0 = c
            while c < TOTAL_WIDTH and grid[r][c] == 0:
                c += 1
            parts.append(
                f'<rect x="{x + c0 * cell:.4f}" y="{y + r * cell:.4f}" '
                f'width="{(c - c0) * cell:.4f}" height="{cell:.4f}" fill="#000"/>'
            )
    parts.append("</g>")
    return parts


def _layout(
    page_w_in: float,
    page_h_in: float,
    margin_in: float,
    cols: int,
    rows: int,
) -> dict:
    id_strip = 0.18
    usable_w = page_w_in - 2 * margin_in
    usable_h = page_h_in - 2 * margin_in
    tag_w = usable_w / cols
    tag_h = (usable_h / rows) - id_strip
    tag_size = min(tag_w, tag_h)
    gap_x = (usable_w - cols * tag_size) / max(cols - 1, 1) if cols > 1 else 0.0
    cell_h = tag_size + id_strip
    gap_y = (usable_h - rows * cell_h) / max(rows - 1, 1) if rows > 1 else 0.0
    black_in = tag_size * (WIDTH_AT_BORDER / TOTAL_WIDTH)
    return {
        "id_strip": id_strip,
        "tag_size": tag_size,
        "gap_x": gap_x,
        "gap_y": gap_y,
        "cell_h": cell_h,
        "black_in": black_in,
    }


def make_sheet_svg(
    out: Path,
    page_w_in: float = 8.5,
    page_h_in: float = 11.0,
    margin_in: float = 0.2,
    cols: int = 4,
    rows: int = 4,
    start_id: int = 0,
) -> dict:
    n = cols * rows
    codes = CODES_0_15[start_id : start_id + n]
    if len(codes) < n:
        raise SystemExit(f"Need {n} codes starting at {start_id}; only have through 15")

    lay = _layout(page_w_in, page_h_in, margin_in, cols, rows)
    tag_size = lay["tag_size"]
    id_strip = lay["id_strip"]
    black_in = lay["black_in"]

    lines: list[str] = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{page_w_in}in" height="{page_h_in}in" '
        f'viewBox="0 0 {page_w_in} {page_h_in}">',
        "<title>AprilTag tag36h11 IDs 0-15 — print at 100%</title>",
        f'<rect width="{page_w_in}" height="{page_h_in}" fill="#fff"/>',
        f'<text x="{margin_in}" y="{margin_in * 0.55}" font-family="Helvetica,Arial,sans-serif" '
        f'font-size="0.08" fill="#444">'
        f"tag36h11 · IDs {start_id}-{start_id + n - 1} · print Actual size / 100% · "
        f"black square ≈ {black_in:.3f} in ({black_in * 25.4:.1f} mm)</text>",
    ]

    for i, code in enumerate(codes):
        r, c = divmod(i, cols)
        x = margin_in + c * (tag_size + lay["gap_x"])
        y = margin_in + r * (lay["cell_h"] + lay["gap_y"])
        tid = start_id + i
        lines.extend(svg_tag(code, x, y, tag_size, tid))
        cx = x + tag_size / 2
        ty = y + tag_size + id_strip * 0.72
        lines.append(
            f'<text x="{cx:.4f}" y="{ty:.4f}" text-anchor="middle" '
            f'font-family="Helvetica,Arial,sans-serif" font-size="0.12" fill="#000">'
            f"{tid}</text>"
        )

    lines.append("</svg>")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return {
        "path": str(out),
        "tag_outer_in": tag_size,
        "black_square_in": black_in,
        "black_square_mm": black_in * 25.4,
        "margin_in": margin_in,
    }


def make_sheet_pdf(
    out: Path,
    page_w_in: float = 8.5,
    page_h_in: float = 11.0,
    margin_in: float = 0.2,
    cols: int = 4,
    rows: int = 4,
    start_id: int = 0,
) -> dict:
    """Dependency-free vector PDF (US Letter points)."""
    n = cols * rows
    codes = CODES_0_15[start_id : start_id + n]
    if len(codes) < n:
        raise SystemExit(f"Need {n} codes starting at {start_id}; only have through 15")

    lay = _layout(page_w_in, page_h_in, margin_in, cols, rows)
    tag_size_in = lay["tag_size"]
    black_in = lay["black_in"]
    page_w, page_h = page_w_in * 72, page_h_in * 72
    margin = margin_in * 72
    tag_size = tag_size_in * 72
    id_strip = lay["id_strip"] * 72
    gap_x = lay["gap_x"] * 72
    gap_y = lay["gap_y"] * 72
    cell_h = lay["cell_h"] * 72

    def esc(s: str) -> str:
        return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")

    ops: list[str] = ["q", "BT /F1 6 Tf 0.27 0.27 0.27 rg"]
    ops.append(f"1 0 0 1 {margin:.2f} {page_h - margin * 0.55:.2f} Tm")
    hdr = (
        f"tag36h11 · IDs {start_id}-{start_id + n - 1} · print Actual size / 100% · "
        f"black square ~ {black_in:.3f} in ({black_in * 25.4:.1f} mm)"
    )
    ops.append(f"({esc(hdr)}) Tj")
    ops.append("ET")

    for i, code in enumerate(codes):
        r, c = divmod(i, cols)
        x = margin + c * (tag_size + gap_x)
        y_top = page_h - (margin + r * (cell_h + gap_y))
        y = y_top - tag_size
        grid = tag_grid(code)
        cell = tag_size / TOTAL_WIDTH
        ops.append("1 1 1 rg")
        ops.append(f"{x:.3f} {y:.3f} {tag_size:.3f} {tag_size:.3f} re f")
        ops.append("0 0 0 rg")
        for row in range(TOTAL_WIDTH):
            col = 0
            while col < TOTAL_WIDTH:
                if grid[row][col] == 1:
                    col += 1
                    continue
                c0 = col
                while col < TOTAL_WIDTH and grid[row][col] == 0:
                    col += 1
                ry = y_top - (row + 1) * cell
                ops.append(
                    f"{x + c0 * cell:.3f} {ry:.3f} {(col - c0) * cell:.3f} {cell:.3f} re f"
                )
        label = str(start_id + i)
        tw = 0.5 * 9 * len(label)
        cx = x + tag_size / 2
        ty = y - id_strip * 0.55
        ops.append("BT /F1 9 Tf 0 0 0 rg")
        ops.append(f"1 0 0 1 {cx - tw / 2:.2f} {ty:.2f} Tm")
        ops.append(f"({label}) Tj")
        ops.append("ET")
    ops.append("Q")
    stream = "\n".join(ops).encode("latin-1")

    objects = [
        b"1 0 obj<< /Type /Catalog /Pages 2 0 R >>endobj\n",
        b"2 0 obj<< /Type /Pages /Kids [3 0 R] /Count 1 >>endobj\n",
        (
            f"3 0 obj<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {page_w} {page_h}] "
            f"/Contents 4 0 R /Resources << /Font << /F1 5 0 R >> >> >>endobj\n"
        ).encode(),
        f"4 0 obj<< /Length {len(stream)} >>stream\n".encode()
        + stream
        + b"\nendstream\nendobj\n",
        b"5 0 obj<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>endobj\n",
    ]
    pdf = bytearray(b"%PDF-1.4\n")
    offsets = [0]
    for obj in objects:
        offsets.append(len(pdf))
        pdf.extend(obj)
    xref_pos = len(pdf)
    pdf.extend(f"xref\n0 {len(objects) + 1}\n".encode())
    pdf.extend(b"0000000000 65535 f \n")
    for off in offsets[1:]:
        pdf.extend(f"{off:010d} 00000 n \n".encode())
    pdf.extend(
        f"trailer<< /Size {len(objects) + 1} /Root 1 0 R >>\n"
        f"startxref\n{xref_pos}\n%%EOF\n".encode()
    )
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(pdf)
    return {
        "path": str(out),
        "tag_outer_in": tag_size_in,
        "black_square_in": black_in,
        "black_square_mm": black_in * 25.4,
        "margin_in": margin_in,
    }


def main() -> None:
    here = Path(__file__).resolve().parent
    default_stem = here.parent / "apriltags" / "tag36h11_0-15_letter"
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "-o",
        "--output",
        type=Path,
        default=default_stem.with_suffix(".svg"),
        help="SVG path (PDF written alongside with .pdf)",
    )
    p.add_argument("--margin", type=float, default=0.2, help="page margin inches")
    args = p.parse_args()
    info = make_sheet_svg(args.output, margin_in=args.margin)
    pdf_path = args.output.with_suffix(".pdf")
    info_pdf = make_sheet_pdf(pdf_path, margin_in=args.margin)
    print(
        f"Wrote {info['path']}\n"
        f"Wrote {info_pdf['path']}\n"
        f"  outer tag (w/ quiet zone): {info['tag_outer_in']:.3f} in\n"
        f"  black square: {info['black_square_in']:.3f} in "
        f"({info['black_square_mm']:.1f} mm)\n"
        f"  Print at 100% / Actual size — do not scale to fit."
    )


if __name__ == "__main__":
    main()
