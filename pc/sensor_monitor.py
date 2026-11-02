"""Receive sensor packets from both ESP32 boards over the phone hotspot.

This is a standalone diagnostic tool, separate from the actual game
(game_wifi.py). It listens for the same UDP packets the game reads, prints
a live terminal dashboard, and also serves a simple HTML dashboard at
http://localhost:8000/ (see website.html, which must sit next to this file).

Updated for 6-sensor setup: sensor1-sensor6, support, deadZone fields.
"""

import argparse
import json
import socket
import threading
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


def parse_packet(data):
	"""Return a validated packet, or None for malformed UDP data."""
	try:
		packet = json.loads(data.decode("utf-8"))
		if not isinstance(packet, dict) or not packet.get("mac"):
			return None
		for key in ("sensor1", "sensor2", "sensor3", "sensor4", "sensor5", "sensor6", "closest"):
			try:
				value = float(packet[key])
			except (KeyError, TypeError, ValueError):
				packet[key] = None
				continue
			if value < 0:
				packet[key] = None
			else:
				packet[key] = value
		for key in ("x", "y"):
			try:
				value = float(packet[key])
			except (KeyError, TypeError, ValueError):
				packet[key] = None
			else:
				packet[key] = value if 0.0 <= value <= 1.0 else None
		for key in ("support",):
			try:
				packet[key] = int(packet[key])
			except (KeyError, TypeError, ValueError):
				packet[key] = None
		for key in ("deadZone",):
			try:
				packet[key] = bool(packet[key])
			except (KeyError, TypeError, ValueError):
				packet[key] = None
		return packet
	except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError):
		return None


def format_distance(value):
	return "--" if value is None else f"{value:7.2f}"


def print_dashboard(packets):
	print("ESP32 wireless sensor readings (6-sensor setup)")
	print(datetime.now().strftime("Updated: %Y-%m-%d %H:%M:%S"))
	print("Master reads S1-S3 locally; S4-S6 arrive from slave via ESP-NOW.")
	print("-" * 140)
	print(
		f"{'Master MAC':20} {'IP address':16} "
		f"{'S1 (cm)':9} {'S2 (cm)':9} {'S3 (cm)':9} "
		f"{'S4 (cm)':9} {'S5 (cm)':9} {'S6 (cm)':9} "
		f"{'Closest':9} {'X':6} {'Y':6} {'Sup':4} {'Status':8}"
	)
	print("-" * 140)
	for mac, packet in sorted(packets.items()):
		age = datetime.now().timestamp() - packet["received_at"]
		status = "online" if age < 5 else f"{age:.0f}s ago"
		x_str = f"{packet['x']:.3f}" if packet.get('x') is not None else "--"
		y_str = f"{packet['y']:.3f}" if packet.get('y') is not None else "--"
		sup_str = str(packet.get('support', '--')) if packet.get('support') is not None else "--"
		print(
			f"{mac:20} {packet['address']:16} "
			f"{format_distance(packet.get('sensor1')):9} "
			f"{format_distance(packet.get('sensor2')):9} "
			f"{format_distance(packet.get('sensor3')):9} "
			f"{format_distance(packet.get('sensor4')):9} "
			f"{format_distance(packet.get('sensor5')):9} "
			f"{format_distance(packet.get('sensor6')):9} "
			f"{format_distance(packet.get('closest')):9} "
			f"{x_str:6} {y_str:6} {sup_str:4} {status:8}"
		)
	if not packets:
		print("Waiting for packets from the ESP32 boards...")
	print("-" * 140)
	print("Press Ctrl+C to stop.")


class WifiDistanceReader:
	def __init__(self, host="0.0.0.0", port=4210):
		self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		self.socket.bind((host, port))
		self.socket.settimeout(0.1)

	def read_packet(self):
		try:
			data, address = self.socket.recvfrom(512)
		except socket.timeout:
			return None
		packet = parse_packet(data)
		if packet is not None:
			packet["address"] = address[0]
		return packet

	def close(self):
		self.socket.close()


DASHBOARD_HTML_PATH = Path(__file__).with_name("website.html")


def make_dashboard_handler(packets, packets_lock):
	"""Build a request handler class bound to the shared packets dict."""

	class DashboardHandler(BaseHTTPRequestHandler):
		def log_message(self, format_str, *args):
			pass  # keep the terminal dashboard clean; suppress HTTP access logs

		def do_GET(self):
			if self.path == "/data":
				with packets_lock:
					body = json.dumps(packets).encode("utf-8")
				self.send_response(200)
				self.send_header("Content-Type", "application/json")
				self.send_header("Cache-Control", "no-store")
				self.send_header("Content-Length", str(len(body)))
				self.end_headers()
				self.wfile.write(body)
			elif self.path in ("/", "/website.html"):
				try:
					body = DASHBOARD_HTML_PATH.read_bytes()
				except FileNotFoundError:
					self.send_response(404)
					self.end_headers()
					self.wfile.write(b"website.html not found next to sensor_monitor.py")
					return
				self.send_response(200)
				self.send_header("Content-Type", "text/html")
				self.send_header("Content-Length", str(len(body)))
				self.end_headers()
				self.wfile.write(body)
			else:
				self.send_response(404)
				self.end_headers()

	return DashboardHandler


def start_dashboard_server(packets, packets_lock, port):
	handler = make_dashboard_handler(packets, packets_lock)
	server = ThreadingHTTPServer(("0.0.0.0", port), handler)
	thread = threading.Thread(target=server.serve_forever, daemon=True)
	thread.start()
	return server


def main():
	parser = argparse.ArgumentParser(description="Receive ESP32 sensor data over UDP.")
	parser.add_argument("--port", type=int, default=4210, help="UDP port for sensor packets")
	parser.add_argument("--http-port", type=int, default=8000, help="HTTP port for the HTML dashboard")
	parser.add_argument("--no-http", action="store_true", help="disable the HTML dashboard server")
	args = parser.parse_args()

	reader = WifiDistanceReader(port=args.port)
	packets = {}
	packets_lock = threading.Lock()

	if not args.no_http:
		start_dashboard_server(packets, packets_lock, args.http_port)
		print(f"HTML dashboard: http://localhost:{args.http_port}/")

	print(f"Listening for ESP32 packets on UDP port {args.port}...")
	print_dashboard(packets)
	try:
		while True:
			packet = reader.read_packet()
			if packet is None:
				continue
			packet["received_at"] = datetime.now().timestamp()
			with packets_lock:
				packets[packet["mac"]] = packet
			print_dashboard(packets)
	except KeyboardInterrupt:
		print("\nStopped")
	finally:
		reader.close()


if __name__ == "__main__":
	main()
