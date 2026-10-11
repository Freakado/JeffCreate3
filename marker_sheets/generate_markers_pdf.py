#!/usr/bin/env python3
from pathlib import Path
import cv2
import numpy as np
from PIL import Image, ImageDraw
import qrcode

BASE_URL = "http://192.168.1.100:8000"
OUTPUT_DIR = Path.home() / "ros2_ws" / "assets" / "puzzles" / "marker_sheets"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

TAG_IDS = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13]

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
generated_cards = []

# --- Dimension Calculations for Maximum Fill ---
# QR Version 3 + border 4 = 29 + 8 = 37 modules.
# box_size=11 -> 37 * 11 = 407 px.
QR_BOX_SIZE = 11
TARGET_MARKER_SIZE = 407  # AprilTag matched exactly to QR pixel dimensions

for tag_id in TAG_IDS:
    # --- AprilTag Generation ---
    if hasattr(cv2.aruco, "drawMarker"):
        tag_img_cv = cv2.aruco.drawMarker(
            aruco_dict, tag_id, TARGET_MARKER_SIZE, borderBits=1
        )
    else:
        tag_img_cv = cv2.aruco.generateImageMarker(
            aruco_dict, tag_id, TARGET_MARKER_SIZE, borderBits=1
        )
    tag_img_pil = Image.fromarray(tag_img_cv).convert("RGB")

    # --- Robust QR Code Generation (Version 3) ---
    target_url = f"{BASE_URL}/inspect?tag={tag_id}"
    qr = qrcode.QRCode(
        version=3,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=QR_BOX_SIZE,
        border=4,
    )
    qr.add_data(target_url)
    qr.make(fit=False)
    qr_img_pil = qr.make_image(
        fill_color="black", back_color="white"
    ).convert("RGB")
    qr_w, qr_h = qr_img_pil.size

    # --- Assemble Card Layout ---
    pad_x = 40
    gap = 40
    card_w = TARGET_MARKER_SIZE + qr_w + (2 * pad_x) + gap  # ~934 px wide
    card_h = TARGET_MARKER_SIZE + 85  # ~492 px high

    card = Image.new("RGB", (card_w, card_h), "white")

    tag_x = pad_x
    qr_x = pad_x + TARGET_MARKER_SIZE + gap
    tag_y = 20
    qr_y = 20

    card.paste(tag_img_pil, (tag_x, tag_y))
    card.paste(qr_img_pil, (qr_x, qr_y))

    # Add Text Labels
    draw = ImageDraw.Draw(card)
    text_y = TARGET_MARKER_SIZE + 28
    draw.text((tag_x, text_y), f"AprilTag 36h11: ID {tag_id}", fill="black")
    draw.text(
        (qr_x, text_y), f"Inspection QR (Artifact #{tag_id})", fill="black"
    )
    draw.text((tag_x, text_y + 18), f"Target: {target_url}", fill="#6b7280")
    draw.rectangle(
        [(0, 0), (card_w - 1, card_h - 1)], outline="#d1d5db", width=2
    )

    card_filename = OUTPUT_DIR / f"marker_tag_{tag_id}.png"
    card.save(card_filename)
    generated_cards.append(card)
    print(f"Generated marker: {card_filename} (Size: {card_w}x{card_h})")

# --- Stitch Exactly 3 Placards Per Page into A4 PDF ---
A4_PAGE_W = 1240
A4_PAGE_H = 1754

pdf_pages = []
current_page = Image.new("RGB", (A4_PAGE_W, A4_PAGE_H), "white")

slots_per_page = 3
margin_top = 60
# Evenly distribute the 3 cards across remaining vertical space
slot_spacing = (A4_PAGE_H - (2 * margin_top) - (slots_per_page * card_h)) // (
    slots_per_page - 1
)
x_centered = (A4_PAGE_W - card_w) // 2

slot_count = 0

for card in generated_cards:
    y_pos = margin_top + slot_count * (card_h + slot_spacing)
    current_page.paste(card, (x_centered, y_pos))
    slot_count += 1

    if slot_count == slots_per_page:
        pdf_pages.append(current_page)
        current_page = Image.new("RGB", (A4_PAGE_W, A4_PAGE_H), "white")
        slot_count = 0

if slot_count > 0:
    pdf_pages.append(current_page)

pdf_filename = OUTPUT_DIR / "printable_placards_A4.pdf"
pdf_pages[0].save(
    pdf_filename,
    save_all=True,
    append_images=pdf_pages[1:],
    resolution=150.0,
)

print(f"\nSaved all {len(generated_cards)} markers into: {pdf_filename}")