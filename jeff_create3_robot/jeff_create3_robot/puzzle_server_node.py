#!/usr/bin/env python3

import os
import json
import random
import hashlib
import threading
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import uvicorn

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

# --- Asset Paths ---
ASSETS_DIR = Path.home() / "ros2_ws" / "assets"
DATASET_DIR = ASSETS_DIR / "puzzles" / "captcha"

# --- Trial Sequences ---
TRIAL_SCHEDULES = {
    "DEFAULT": ["card.html", "wires.html", "captcha.html", "sudoku.html"],
    "A": ["sudoku.html", "captcha.html", "wires.html", "card.html"],  # Backwards
    "B": ["wires.html", "card.html", "sudoku.html", "captcha.html"],  # Mix 1
    "C": ["captcha.html", "sudoku.html", "card.html", "wires.html"]   # Mix 2
}

ACTIVE_SEQUENCE = TRIAL_SCHEDULES["DEFAULT"]

# Global switch for test / no-exploration mode
STANDALONE_TEST_MODE = False

# Pre-index sorted file lists on startup
CATEGORY_PREFIXES = {
    "crosswalk": ("crosswalk", "Cross"),
    "fire_hydrant": ("fire_hydrant", "Hydrant"),
    "bicycle": ("bicycle", "Bicycle"),
    "traffic_light": ("traffic_light", "Tlight")
}

IMAGE_POOLS = {}
for cat, (subfolder, prefix) in CATEGORY_PREFIXES.items():
    folder = DATASET_DIR / subfolder
    if folder.is_dir():
        files = sorted(
            [f.name for f in folder.glob(f"{prefix}*.png")],
            key=lambda x: int(''.join(filter(str.isdigit, x)) or 0)
        )
        IMAGE_POOLS[cat] = files
    else:
        IMAGE_POOLS[cat] = []

print(f"[Dataset] Indexed: { {k: len(v) for k, v in IMAGE_POOLS.items()} }")

# --- ROS 2 Node ---
class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('puzzle_web_bridge')

        global STANDALONE_TEST_MODE, ACTIVE_SEQUENCE

        # Parameters
        self.declare_parameter('trial_schedule', 'DEFAULT')
        self.declare_parameter('standalone_test_mode', False)

        STANDALONE_TEST_MODE = self.get_parameter('standalone_test_mode').get_parameter_value().bool_value
        if STANDALONE_TEST_MODE:
            self.get_logger().warn("STANDALONE TEST MODE ACTIVE: Exploration stack not required.")

        schedule_name = self.get_parameter('trial_schedule').get_parameter_value().string_value.upper()
        if schedule_name in TRIAL_SCHEDULES:
            ACTIVE_SEQUENCE = TRIAL_SCHEDULES[schedule_name]
            self.get_logger().info(f"Loaded trial schedule '{schedule_name}': {ACTIVE_SEQUENCE}")
        else:
            self.get_logger().warn(f"Schedule '{schedule_name}' unknown. Using DEFAULT: {ACTIVE_SEQUENCE}")

        operator_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.solve_pub = self.create_publisher(String, 'puzzle_solved', operator_qos)

    def report_solved(self, tag_id: int, puzzle_name: str):
        msg = String()
        msg.data = json.dumps({
            "tag_id": tag_id,
            "puzzle": puzzle_name,
            "status": "SOLVED",
            "standalone_mode": STANDALONE_TEST_MODE
        })
        self.solve_pub.publish(msg)
        self.get_logger().info(f"Broadcasted solved event: {msg.data}")


# --- Share Paths ---
try:
    package_share_dir = get_package_share_directory('jeff_create3_robot')
    STATIC_DIR = Path(package_share_dir) / 'puzzles'
except Exception:
    # Fallback to local path if package is not sourced during isolated web testing
    STATIC_DIR = Path.home() / "ros2_ws" / "src" / "jeff_create3_robot" / "puzzles"

if not STATIC_DIR.is_dir():
    STATIC_DIR = Path("/tmp")  # Graceful fallback


# --- Deterministic RNG Seeder ---
def get_deterministic_rng(tag_id: int, salt: str = "exp_v1") -> random.Random:
    seed_str = f"{salt}_{tag_id}"
    seed_int = int(hashlib.sha256(seed_str.encode('utf-8')).hexdigest()[:8], 16)
    return random.Random(seed_int)


# --- Puzzle Generators ---
def generate_4x4_sudoku(tag_id: int):
    rng = get_deterministic_rng(tag_id, salt="sudoku")
    base_solution = [
        [1, 2, 3, 4],
        [3, 4, 1, 2],
        [2, 1, 4, 3],
        [4, 3, 2, 1]
    ]
    symbols = [1, 2, 3, 4]
    rng.shuffle(symbols)
    mapping = {i + 1: symbols[i] for i in range(4)}
    solution_grid = [[mapping[val] for val in row] for row in base_solution]

    puzzle_grid = [row[:] for row in solution_grid]
    all_coords = [(r, c) for r in range(4) for c in range(4)]
    cells_to_remove = rng.sample(all_coords, 6)

    for r, c in cells_to_remove:
        puzzle_grid[r][c] = 0

    expected_answer = "".join(str(solution_grid[r][c]) for (r, c) in sorted(cells_to_remove))
    return {
        "tag_id": tag_id,
        "type": "sudoku_4x4",
        "grid": puzzle_grid,
        "solution": expected_answer
    }


def generate_symbol_captcha(tag_id: int):
    rng = get_deterministic_rng(tag_id, salt="captcha")
    categories = ["traffic_light", "bicycle", "fire_hydrant", "crosswalk"]
    target_category = categories[tag_id % len(categories)]

    tile_indices = list(range(9))
    target_indices = set(rng.sample(tile_indices, 3))

    tiles = []
    for idx in range(9):
        if idx in target_indices:
            cat = target_category
        else:
            cat = rng.choice([c for c in categories if c != target_category])

        pool = IMAGE_POOLS.get(cat, [])
        if pool:
            file_name = pool[rng.randint(0, len(pool) - 1)]
            img_url = f"/assets/puzzles/captcha/{CATEGORY_PREFIXES[cat][0]}/{file_name}"
        else:
            img_url = f"/static/images/{cat}/1.jpg"

        tiles.append({
            "id": idx,
            "category": cat,
            "image_url": img_url
        })

    expected_answer = ",".join(str(i) for i in sorted(target_indices))
    return {
        "tag_id": tag_id,
        "type": "icon_captcha",
        "target": target_category,
        "instruction": f"Select all squares depicting a {target_category.replace('_', ' ')}",
        "tiles": tiles,
        "solution": expected_answer
    }


def generate_wire_puzzle(tag_id: int):
    rng = get_deterministic_rng(tag_id, salt="wires_v2")
    base_colors = ["red", "blue", "yellow", "green", "orange", "magenta"]

    left_terminals = list(base_colors)
    right_terminals = list(base_colors)
    rng.shuffle(left_terminals)
    rng.shuffle(right_terminals)

    if left_terminals == right_terminals:
        right_terminals[0], right_terminals[1] = right_terminals[1], right_terminals[0]

    mapping = {}
    for l_idx, col in enumerate(left_terminals):
        r_idx = right_terminals.index(col)
        mapping[l_idx] = r_idx

    solution_str = ",".join(f"{l}:{mapping[l]}" for l in sorted(mapping.keys()))

    return {
        "tag_id": tag_id,
        "type": "logic_wires",
        "left_terminals": left_terminals,
        "right_terminals": right_terminals,
        "solution": solution_str
    }


def generate_card_swipe(tag_id: int):
    rng = get_deterministic_rng(tag_id, salt="card_swipe")
    min_ms = 450 + (rng.randint(0, 5) * 20)
    max_ms = 1050 + (rng.randint(0, 6) * 30)
    expected_answer = f"SWIPED_VALID_{tag_id}"

    return {
        "tag_id": tag_id,
        "type": "card_swipe",
        "min_ms": min_ms,
        "max_ms": max_ms,
        "solution": expected_answer
    }


# --- FastAPI Application ---
app = FastAPI(title="JeffCreate3 Puzzle Host")
ros_node = None

class SolutionSubmission(BaseModel):
    tag_id: int
    puzzle_name: str
    solution: str


@app.get("/api/mode")
def get_operating_mode():
    """Check whether standalone test mode is enabled."""
    return {
        "standalone_test_mode": STANDALONE_TEST_MODE,
        "ros_connected": ros_node is not None,
        "active_schedule": ACTIVE_SEQUENCE
    }


@app.post("/api/test_mode")
def set_test_mode(enabled: bool = Query(...)):
    """Toggle standalone test mode without restarting the server."""
    global STANDALONE_TEST_MODE
    STANDALONE_TEST_MODE = enabled
    return {"status": "SUCCESS", "standalone_test_mode": STANDALONE_TEST_MODE}


@app.get("/api/puzzle_data/{tag_id}")
def get_puzzle_state(tag_id: int, type: Optional[str] = Query(None)):
    p_type = type.lower() if type else None

    if p_type in ("sudoku", "sudoku_4x4"):
        puzzle = generate_4x4_sudoku(tag_id)
    elif p_type in ("captcha", "icon_captcha"):
        puzzle = generate_symbol_captcha(tag_id)
    elif p_type in ("wires", "logic_wires"):
        puzzle = generate_wire_puzzle(tag_id)
    elif p_type in ("card", "card_swipe", "swipe"):
        puzzle = generate_card_swipe(tag_id)
    else:
        idx = (tag_id - 1) % len(ACTIVE_SEQUENCE)
        page = ACTIVE_SEQUENCE[idx]

        if page == "sudoku.html":
            puzzle = generate_4x4_sudoku(tag_id)
        elif page == "captcha.html":
            puzzle = generate_symbol_captcha(tag_id)
        elif page == "wires.html":
            puzzle = generate_wire_puzzle(tag_id)
        elif page == "card.html":
            puzzle = generate_card_swipe(tag_id)

    return {k: v for k, v in puzzle.items() if k != "solution"}


@app.post("/api/solve")
def solve_puzzle(data: SolutionSubmission):
    sol = data.solution.strip().lower()

    # Master debug override
    debug_overrides = {10: "42", 379: "blue"}
    if data.tag_id in debug_overrides and sol == debug_overrides[data.tag_id]:
        if ros_node is not None:
            ros_node.report_solved(data.tag_id, data.puzzle_name)
        return {"success": True, "message": "Verification confirmed (Master Override)."}

    p_name = data.puzzle_name.lower()
    if p_name in ("sudoku", "sudoku_4x4"):
        expected = generate_4x4_sudoku(data.tag_id)["solution"]
    elif p_name in ("captcha", "icon_captcha"):
        expected = generate_symbol_captcha(data.tag_id)["solution"]
    elif p_name in ("wires", "logic_wires"):
        expected = generate_wire_puzzle(data.tag_id)["solution"]
    elif p_name in ("card", "card_swipe"):
        expected = generate_card_swipe(data.tag_id)["solution"]
    else:
        return {"success": False, "message": "Unknown puzzle type."}

    if sol == expected.lower():
        if ros_node is not None:
            ros_node.report_solved(data.tag_id, data.puzzle_name)
        msg = "Access Granted! (Test Mode Active)" if STANDALONE_TEST_MODE else "Access Granted! Resuming robot exploration."
        return {"success": True, "message": msg, "test_mode": STANDALONE_TEST_MODE}
    else:
        return {"success": False, "message": "Verification failed. Check your input."}


@app.get("/inspect")
def inspect_artifact(tag: int = Query(..., description="Detected AprilTag ID")):
    idx = (tag - 1) % len(ACTIVE_SEQUENCE)
    target_page = ACTIVE_SEQUENCE[idx]
    redirect_url = f"/{target_page}?tag={tag}"
    return RedirectResponse(url=redirect_url, status_code=307)


@app.post("/api/configure_schedule")
def set_trial_schedule(schedule_name: str = Query(...)):
    global ACTIVE_SEQUENCE
    name = schedule_name.upper()
    if name in TRIAL_SCHEDULES:
        ACTIVE_SEQUENCE = TRIAL_SCHEDULES[name]
        return {"status": "SUCCESS", "current_schedule": ACTIVE_SEQUENCE}
    raise HTTPException(status_code=400, detail=f"Unknown schedule: {schedule_name}")


# --- HTML File Routes ---
@app.get("/sudoku.html")
@app.get("/")
def serve_sudoku():
    return FileResponse(STATIC_DIR / "sudoku.html")

@app.get("/captcha.html")
def serve_captcha():
    return FileResponse(STATIC_DIR / "captcha.html")

@app.get("/wires.html")
def serve_wires():
    return FileResponse(STATIC_DIR / "wires.html")

@app.get("/card.html")
def serve_card():
    return FileResponse(STATIC_DIR / "card.html")

@app.get("/wallet.png")
@app.get("/wallet.jpg")
def serve_wallet_image():
    for ext in ("wallet.png", "wallet.jpg"):
        asset_file = ASSETS_DIR / "puzzles" / "card" / ext
        if asset_file.is_file():
            return FileResponse(asset_file)
        static_file = STATIC_DIR / ext
        if static_file.is_file():
            return FileResponse(static_file)
    raise HTTPException(status_code=404, detail="Wallet image not found")


if ASSETS_DIR.is_dir():
    app.mount("/assets", StaticFiles(directory=str(ASSETS_DIR)), name="assets")

if STATIC_DIR.is_dir():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


def main(args=None):
    global ros_node
    try:
        rclpy.init(args=args)
        ros_node = WebBridgeNode()
        ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
        ros_thread.start()
    except Exception as e:
        print(f"[WebBridge] Running in PURE HTTP TEST MODE (ROS 2 init bypassed: {e})")

    try:
        uvicorn.run(app, host="0.0.0.0", port=8000, log_level="info")
    finally:
        if rclpy.ok() and ros_node is not None:
            ros_node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()