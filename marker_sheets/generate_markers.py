#!/usr/bin/env python3
import cv2
import numpy as np
import qrcode
from PIL import Image, ImageDraw
from pathlib import Path

BASE_URL = "http://192.168.1.100:8000"
OUTPUT_DIR = Path.home() / "ros2_ws" / "assets" / "puzzles" / "marker_sheets"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

TAG_IDS = [1,2,3,4,5,6,7,8,9,10,11,12,13]

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
generated_cards = []

for tag_id in TAG_IDS:
    # AprilTag Generation (Clean 300x300 px) ---
    tag_size_px = 300
    if hasattr(cv2.aruco, "drawMarker"):
        tag_img_cv = cv2.aruco.drawMarker(aruco_dict, tag_id, tag_size_px, borderBits=1)
    else:
        tag_img_cv = cv2.aruco.generateImageMarker(aruco_dict, tag_id, tag_size_px, borderBits=1)
    tag_img_pil = Image.fromarray(tag_img_cv).convert('RGB')

    # Robust QR Code Generation
    target_url = f"{BASE_URL}/inspect?tag={tag_id}"
    qr = qrcode.QRCode(
        version=3,                                          # Lock identical grid geometry across all tags
        error_correction=qrcode.constants.ERROR_CORRECT_H,  # High Reed-Solomon redundancy (30%)
        box_size=8,                                         # Clean integer scaling
        border=4,                                           # Standard 4-module quiet zone
    )
    qr.add_data(target_url)
    qr.make(fit=False)                                      # Enforce Version 3 strictly
    qr_img_pil = qr.make_image(fill_color="black", back_color="white").convert('RGB')
    qr_w, qr_h = qr_img_pil.size

    # --- Assemble Card Layout ---
    card_w = tag_size_px + qr_w + 80
    card_h = max(tag_size_px, qr_h) + 90
    card = Image.new('RGB', (card_w, card_h), 'white')

    # Vertically center both markers in their respective slots
    tag_y = 20 + (max(tag_size_px, qr_h) - tag_size_px) // 2
    qr_y = 20 + (max(tag_size_px, qr_h) - qr_h) // 2

    card.paste(tag_img_pil, (30, tag_y))
    card.paste(qr_img_pil, (tag_size_px + 50, qr_y))

    # Add Text Labels
    draw = ImageDraw.Draw(card)
    text_y = max(tag_size_px, qr_h) + 30
    draw.text((30, text_y), f"AprilTag 36h11: ID {tag_id}", fill='black')
    draw.text((tag_size_px + 50, text_y), f"Inspection QR (Artifact #{tag_id})", fill='black')
    draw.text((30, text_y + 22), f"Target: {target_url}", fill='#6b7280')
    draw.rectangle([(0, 0), (card_w - 1, card_h - 1)], outline="#d1d5db", width=2)

    card_filename = OUTPUT_DIR / f"marker_tag_{tag_id}.png"
    card.save(card_filename)
    generated_cards.append(card)
    print(f"Generated compliant marker: {card_filename} (QR native size: {qr_w}x{qr_h})")

print("\nMarkers regenerated successfully with intact optical ratios.")