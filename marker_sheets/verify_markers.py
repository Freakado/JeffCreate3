#!/usr/bin/env python3
import sys
import cv2
import argparse
from pathlib import Path
from pupil_apriltags import Detector
from pyzbar.pyzbar import decode as decode_qr

TAG_FAMILY = "tag36h11"

def test_marker_folder(folder_path: Path):
    if not folder_path.is_dir():
        print(f"Error: Directory '{folder_path}' does not exist.")
        sys.exit(1)

    png_files = sorted(
        [p for p in folder_path.glob("marker_tag_*.png")],
        key=lambda p: int(''.join(filter(str.isdigit, p.stem)) or 0)
    )

    if not png_files:
        print(f"No .png files found in {folder_path}")
        return

    detector = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0)

    print(f"\nScanning {len(png_files)} images in: {folder_path}")
    print("-" * 88)
    print(f"{'Filename':<22} | {'AprilTag':<10} | {'QR Status':<12} | {'QR Decoded Payload'}")
    print("-" * 88)

    passed_count = 0

    for file in png_files:
        img = cv2.imread(str(file))
        if img is None:
            print(f"{file.name:<22} | FAILED TO READ IMAGE FILE")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # AprilTag Detection
        apriltag_status = "NONE"
        detected_tag_id = None
        detections = detector.detect(gray)
        if detections:
            detected_tag_id = detections[0].tag_id
            apriltag_status = f"ID #{detected_tag_id}"

        # PyZBar QR Detection (Matches robot inspect_qr_payload)
        qr_results = decode_qr(gray)
        qr_status = "FAILED"
        payload_str = "None"

        if qr_results:
            try:
                payload_str = qr_results[0].data.decode('utf-8')
                qr_status = "OK"
            except Exception:
                qr_status = "CORRUPT"

        # Check correspondence between AprilTag and QR payload
        is_valid = False
        if detected_tag_id is not None and qr_status == "OK":
            expected_tail = f"tag={detected_tag_id}"
            if expected_tail in payload_str:
                is_valid = True
                passed_count += 1

        status_flag = "✔" if is_valid else "✖"
        print(f"{file.name:<22} | {apriltag_status:<10} | {qr_status:<12} | {payload_str} {status_flag}")

    print("-" * 88)
    print(f"Result: {passed_count}/{len(png_files)} markers fully validated (AprilTag + QR match).\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Batch verification of AprilTag and QR markers.")
    parser.add_argument(
        "folder",
        nargs="?",
        default=str(Path.home() / "ros2_ws" / "assets" / "puzzles" / "marker_sheets"),
        help="Path to folder containing marker PNGs (default: ~/ros2_ws/assets/puzzles/marker_sheets)"
    )
    args = parser.parse_args()

    test_marker_folder(Path(args.folder))