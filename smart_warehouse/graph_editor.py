#!/usr/bin/env python3
"""
Graph Editor Server for Autonomous Forklift.

This script starts a local HTTP server to host the Graph Editor.
It provides a web interface to edit the warehouse_graph.geojson file.

Usage:
    python3 graph_editor.py
"""

import http.server
import socketserver
import json
import os
import sys
import webbrowser
from urllib.parse import urlparse, parse_qs

import io
try:
    from PIL import Image
except ImportError:
    print("Warning: PIL (Pillow) not installed. Map image will not be available.")
    Image = None

import argparse

# Configuration
PORT = 8000
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Determine paths
from ament_index_python.packages import get_package_share_directory

try:
    PKG_SHARE = get_package_share_directory('smart_warehouse')
    # INSTALLED PATHS
    INSTALLED_GRAPH = os.path.join(PKG_SHARE, 'graphs', 'warehouse_graph.geojson')
    INSTALLED_TEMPLATES = os.path.join(PKG_SHARE, 'templates')
    INSTALLED_MAP = os.path.join(PKG_SHARE, 'maps', 'warehouse_numerated_map.yaml')
except:
    PKG_SHARE = None

# SOURCE PATHS (Relative to script: src/smart_warehouse/smart_warehouse/graph_editor.py)
PKG_SOURCE = os.path.dirname(SCRIPT_DIR)
SOURCE_GRAPH = os.path.join(PKG_SOURCE, 'graphs', 'warehouse_graph.geojson')
SOURCE_TEMPLATES = os.path.join(PKG_SOURCE, 'templates')
SOURCE_MAP = os.path.join(PKG_SOURCE, 'maps', 'warehouse_numerated_map.yaml')

if os.path.exists(SOURCE_GRAPH):
    # Running from source
    DEFAULT_GRAPH_FILE = SOURCE_GRAPH
    TEMPLATE_DIR = SOURCE_TEMPLATES
    DEFAULT_MAP_FILE = SOURCE_MAP
elif PKG_SHARE:
    # Running installed
    DEFAULT_GRAPH_FILE = INSTALLED_GRAPH
    TEMPLATE_DIR = INSTALLED_TEMPLATES
    DEFAULT_MAP_FILE = INSTALLED_MAP
else:
    # Fallback
    DEFAULT_GRAPH_FILE = 'warehouse_graph.geojson'
    TEMPLATE_DIR = 'templates'
    DEFAULT_MAP_FILE = 'warehouse_numerated_map.yaml'

# Globals to be set by main
GRAPH_FILE = DEFAULT_GRAPH_FILE
MAP_YAML_FILE = DEFAULT_MAP_FILE

class GraphEditorHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        parsed_path = urlparse(self.path)
        
        # Serve the main editor page
        if parsed_path.path == '/' or parsed_path.path == '/editor.html':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            
            template_path = os.path.join(TEMPLATE_DIR, 'editor.html')
            if os.path.exists(template_path):
                with open(template_path, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                self.wfile.write(b"Error: templates/editor.html not found.")
            return

        # API: Get Graph Data
        if parsed_path.path == '/api/graph':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            if os.path.exists(GRAPH_FILE):
                with open(GRAPH_FILE, 'rb') as f:
                    self.wfile.write(f.read())
            else:
                empty_graph = {"type": "FeatureCollection", "features": []}
                self.wfile.write(json.dumps(empty_graph).encode('utf-8'))
            return

        # API: Get Map Info
        if parsed_path.path == '/api/map/info':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            
            if os.path.exists(MAP_YAML_FILE):
                # Simple YAML parser since we don't want to depend on PyYAML if possible
                # But PyYAML is standard in ROS. Let's try to parse manually for safety or just read lines.
                # Actually, let's just return the raw lines or a simple dict.
                info = {}
                try:
                    with open(MAP_YAML_FILE, 'r') as f:
                        for line in f:
                            if ':' in line:
                                key, val = line.split(':', 1)
                                key = key.strip()
                                val = val.strip()
                                if key == 'resolution': info['resolution'] = float(val)
                                if key == 'origin': info['origin'] = json.loads(val) # YAML list looks like JSON list
                                if key == 'image': info['image'] = val
                except Exception as e:
                    print(f"Error parsing YAML: {e}")
                
                self.wfile.write(json.dumps(info).encode('utf-8'))
            else:
                self.wfile.write(b'{}')
            return

        # API: Get Map Image
        if parsed_path.path == '/api/map/image':
            if not Image:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(b"PIL not installed")
                return

            if os.path.exists(MAP_YAML_FILE):
                # Find image path relative to yaml
                info = {}
                with open(MAP_YAML_FILE, 'r') as f:
                    for line in f:
                        if line.strip().startswith('image:'):
                            info['image'] = line.split(':', 1)[1].strip()
                            break
                
                if 'image' in info:
                    image_path = os.path.join(os.path.dirname(MAP_YAML_FILE), info['image'])
                    if os.path.exists(image_path):
                        try:
                            img = Image.open(image_path)
                            # Convert to RGBA (PNG)
                            img_byte_arr = io.BytesIO()
                            img.save(img_byte_arr, format='PNG')
                            img_byte_arr = img_byte_arr.getvalue()
                            
                            self.send_response(200)
                            self.send_header('Content-type', 'image/png')
                            self.end_headers()
                            self.wfile.write(img_byte_arr)
                            return
                        except Exception as e:
                            print(f"Error serving image: {e}")
            
            self.send_response(404)
            self.end_headers()
            return

        # Serve static files (if any)
        super().do_GET()

    def do_POST(self):
        parsed_path = urlparse(self.path)
        
        # API: Save Graph Data
        if parsed_path.path == '/api/graph':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            
            try:
                # Validate JSON
                graph_data = json.loads(post_data)
                
                # Ensure directory exists
                os.makedirs(os.path.dirname(GRAPH_FILE), exist_ok=True)
                
                # Save to file
                with open(GRAPH_FILE, 'w') as f:
                    json.dump(graph_data, f, indent=2)
                
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "message": "Graph saved successfully."}).encode('utf-8'))
                print(f"Graph saved to {GRAPH_FILE}")
                
            except json.JSONDecodeError:
                self.send_response(400)
                self.end_headers()
                self.wfile.write(b'{"status": "error", "message": "Invalid JSON"}')
            except Exception as e:
                self.send_response(500)
                self.end_headers()
                self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode('utf-8'))
            return

def main():
    global GRAPH_FILE, MAP_YAML_FILE
    
    parser = argparse.ArgumentParser(description='Graph Editor Server')
    parser.add_argument('--graph', default=DEFAULT_GRAPH_FILE, help='Path to GeoJSON graph file')
    parser.add_argument('--map', default=DEFAULT_MAP_FILE, help='Path to YAML map file')
    parser.add_argument('--port', type=int, default=8000, help='Port to run server on')
    
    args = parser.parse_args()
    
    GRAPH_FILE = os.path.abspath(args.graph)
    MAP_YAML_FILE = os.path.abspath(args.map)
    port = args.port
    
    # Ensure template directory exists
    if not os.path.exists(TEMPLATE_DIR):
        print(f"Warning: Template directory {TEMPLATE_DIR} does not exist.")
    
    print(f"Starting Graph Editor Server at http://localhost:{port}")
    print(f"  Graph File: {GRAPH_FILE}")
    print(f"  Map File:   {MAP_YAML_FILE}")
    
    # Change directory to map file directory so relative image paths work? 
    # No, we handle absolute paths in the handler.
    
    # We need to pass the port to the handler, but SimpleHTTPRequestHandler doesn't take args easily.
    # We can just use the global PORT or the one passed to TCPServer.
    # But for the print statement above we used 'port'.
    
    with socketserver.TCPServer(("", port), GraphEditorHandler) as httpd:
        # Open browser automatically
        webbrowser.open(f"http://localhost:{port}")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")

if __name__ == "__main__":
    main()
