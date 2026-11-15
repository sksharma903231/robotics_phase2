import os
import json
import time
from flask import Flask, jsonify, render_template, request

app = Flask(__name__)

# Configurable telemetry file path (defaults to /tmp/rover_telemetry.json)
TELEMETRY_PATH = os.environ.get('TELEMETRY_PATH', '/tmp/rover_telemetry.json')
STALE_THRESHOLD_SECONDS = float(os.environ.get('STALE_THRESHOLD_SECONDS', '5.0'))
MOCK_MODE = os.environ.get('MOCK_MODE', '0') == '1'

@app.route('/')
def control_room():
    """Serves the main telemetry control room dashboard."""
    return render_template('index.html')

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    """Returns current rover telemetry from the bridge or simulation."""
    if MOCK_MODE:
        # Standalone demonstration mode without active ROS 2 / Gazebo
        t = time.time()
        import math
        left = round(45.0 + 5.0 * math.sin(t), 2)
        right = round(20.0 + 3.0 * math.cos(t), 2)
        return jsonify({
            "status": "online (simulated)",
            "front_left_rpm": left,
            "front_right_rpm": right,
            "rear_left_rpm": left,
            "rear_right_rpm": right,
            "system_voltage": 12.4,
            "timestamp": round(t, 2)
        })

    if not os.path.exists(TELEMETRY_PATH):
        return jsonify({
            "status": "offline",
            "front_left_rpm": 0.0,
            "front_right_rpm": 0.0,
            "rear_left_rpm": 0.0,
            "rear_right_rpm": 0.0,
            "system_voltage": 0.0,
            "timestamp": None,
            "message": "Waiting for telemetry bridge..."
        })

    try:
        mtime = os.path.getmtime(TELEMETRY_PATH)
        age = time.time() - mtime

        with open(TELEMETRY_PATH, 'r') as f:
            data = json.load(f)

        status = "online"
        if age > STALE_THRESHOLD_SECONDS:
            status = "stale"

        left_rpm = float(data.get('left_rpm', data.get('front_left_rpm', 0.0)))
        right_rpm = float(data.get('right_rpm', data.get('front_right_rpm', 0.0)))
        voltage = float(data.get('system_voltage', 12.4 if status == 'online' else 0.0))

        return jsonify({
            "status": status,
            "front_left_rpm": left_rpm,
            "front_right_rpm": right_rpm,
            "rear_left_rpm": left_rpm,
            "rear_right_rpm": right_rpm,
            "system_voltage": voltage,
            "timestamp": round(mtime, 2)
        })

    except (json.JSONDecodeError, KeyError, PermissionError, OSError) as e:
        return jsonify({
            "status": "error",
            "front_left_rpm": 0.0,
            "front_right_rpm": 0.0,
            "rear_left_rpm": 0.0,
            "rear_right_rpm": 0.0,
            "system_voltage": 0.0,
            "timestamp": None,
            "message": str(e)
        }), 500

@app.route('/api/telemetry', methods=['POST'])
def post_telemetry():
    """Allows posting telemetry directly for testing and mock injection."""
    try:
        payload = request.get_json(force=True)
        tmp_file = f"{TELEMETRY_PATH}.tmp"
        with open(tmp_file, 'w') as f:
            json.dump(payload, f)
        os.replace(tmp_file, TELEMETRY_PATH)
        return jsonify({"status": "success", "written": payload}), 200
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 400

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 5000))
    app.run(host='0.0.0.0', port=port, debug=False)