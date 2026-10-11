#!/usr/bin/env python3

import sys
import signal
import json
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from PyQt5.QtWidgets import QApplication, QDialog, QVBoxLayout, QPushButton, QHBoxLayout, QLabel
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QUrl, QTimer
from PyQt5.QtWebEngineWidgets import QWebEngineView


class BridgeSignal(QObject):
    show_puzzle = pyqtSignal(dict)
    puzzle_solved = pyqtSignal(int)


class BrowserModalDialog(QDialog):
    def __init__(self, data, parent=None):
        super().__init__(parent)
        self.data = data
        self.tag_id = data.get("tag_id", "Unknown")
        self.target_url = data.get("payload", "").strip()

        self.setWindowTitle(f"Artifact #{self.tag_id} Inspection")
        self.setWindowFlags(Qt.WindowStaysOnTopHint | Qt.Dialog)
        
        # Sized to standard mobile card puzzle dimensions
        self.resize(520, 680)
        self.setStyleSheet("""
            QDialog { background-color: #121214; }
            QPushButton { 
                padding: 7px 16px; font-weight: bold; border-radius: 5px; 
                font-size: 12px; border: none; background-color: #27272a; color: #a1a1aa; 
            }
            QPushButton:hover { background-color: #3f3f46; color: #ffffff; }
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 8)
        layout.setSpacing(4)

        # Embedded Chromium View
        self.browser = QWebEngineView(self)
        self.browser.page().setBackgroundColor(Qt.transparent)
        
        # Fit content once the DOM completes loading
        self.browser.loadFinished.connect(self.on_page_loaded)
        
        if self.target_url:
            self.browser.setUrl(QUrl(self.target_url))
        layout.addWidget(self.browser)

        # Bottom Bar: minimal footprint
        btn_layout = QHBoxLayout()
        btn_layout.setContentsMargins(12, 0, 12, 0)
        
        status_hint = QLabel(f"Tag #{self.tag_id}")
        status_hint.setStyleSheet("color: #71717a; font-size: 11px; font-family: monospace;")
        btn_layout.addWidget(status_hint)
        btn_layout.addStretch()

        self.close_btn = QPushButton("Dismiss Window", self)
        self.close_btn.clicked.connect(self.reject)
        btn_layout.addWidget(self.close_btn)
        
        layout.addLayout(btn_layout)

    def on_page_loaded(self, success: bool):
        if not success:
            return

        # Auto-scale content to fit the dialog without scrollbars
        js_fit_script = """
        (function() {
            // Remove unnecessary margins and disable scrollbars
            document.body.style.margin = '0';
            document.body.style.padding = '4px';
            document.body.style.overflow = 'hidden';
            document.body.style.display = 'flex';
            document.body.style.justifyContent = 'center';
            document.body.style.alignItems = 'flex-start';

            // Find main container or wrapper if present
            let container = document.querySelector('.container') || 
                            document.querySelector('.card') || 
                            document.querySelector('#app') || 
                            document.body.firstElementChild;

            if (container) {
                container.style.maxWidth = '100%';
                container.style.boxSizing = 'border-box';
            }
        })();
        """
        self.browser.page().runJavaScript(js_fit_script)
        
        # Set zoom to fit width cleanly
        self.browser.setZoomFactor(0.95)


class PopupNode(Node):
    def __init__(self, signal_bridge):
        super().__init__('artifact_popup_handler')
        self.signal_bridge = signal_bridge

        operator_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        self.sub_artifact = self.create_subscription(
            String, 'discovered_artifact', self.artifact_callback, operator_qos
        )
        self.sub_solved = self.create_subscription(
            String, 'puzzle_solved', self.solved_callback, operator_qos
        )

        self.opened_tags = set()
        self.get_logger().info("Browser Popup Handler active: rendering web payload from /discovered_artifact.")

    def artifact_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            tag_id = data.get("tag_id")
            url = data.get("payload", "")

            if not url:
                self.get_logger().warn("Artifact message missing 'payload' URL.")
                return

            if tag_id is not None and tag_id in self.opened_tags:
                return

            if tag_id is not None:
                self.opened_tags.add(tag_id)

            self.get_logger().info(f"Loading native browser for Tag #{tag_id}: {url}")
            self.signal_bridge.show_puzzle.emit(data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse /discovered_artifact: {e}")

    def solved_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            tag_id = data.get("tag_id")
            if tag_id is not None:
                self.signal_bridge.puzzle_solved.emit(tag_id)
        except Exception:
            pass

    def on_dialog_dismissed(self, tag_id, accepted: bool):
        if not accepted and tag_id in self.opened_tags:
            self.opened_tags.remove(tag_id)


def ros_spin_worker(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    rclpy.init()
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)

    bridge = BridgeSignal()
    node = PopupNode(bridge)

    current_dialog = {"instance": None, "tag_id": None}

    def display_dialog(data):
        if current_dialog["instance"] is not None:
            current_dialog["instance"].close()

        dialog = BrowserModalDialog(data)
        current_dialog["instance"] = dialog
        current_dialog["tag_id"] = data.get("tag_id")

        result = dialog.exec_()
        node.on_dialog_dismissed(current_dialog["tag_id"], result == QDialog.Accepted)
        current_dialog["instance"] = None
        current_dialog["tag_id"] = None

    def on_external_solve(tag_id):
        # Auto-close popup once solved via the web client
        if current_dialog["instance"] is not None and current_dialog["tag_id"] == tag_id:
            node.get_logger().info(f"Tag #{tag_id} solved! Closing browser window.")
            QTimer.singleShot(600, current_dialog["instance"].accept)

    bridge.show_puzzle.connect(display_dialog)
    bridge.puzzle_solved.connect(on_external_solve)

    spin_thread = Thread(target=ros_spin_worker, args=(node,), daemon=True)
    spin_thread.start()

    heartbeat_timer = QTimer()
    heartbeat_timer.timeout.connect(lambda: None)
    heartbeat_timer.start(250)

    try:
        sys.exit(app.exec_())
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()