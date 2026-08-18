from flask import Flask, jsonify, render_template

app = Flask(__name__)

rover_telemetry={
    "status":"online",
    "front_left_rpm":0.0,
    "front_right_rpm":0.0,
    "system_voltage": 12.4
}

@app.route('/')
def control_room():
    return render_template('index.html')

@app.route('/api/telemetry')
def get_telemetry():
    try:
        with open('/tmp/rover_telemetry.json', 'r') as f:
            data = json.load(f)
        return jsonify({
            "status": "online",
            "front_left_rpm": data['left_rpm'],
            "front_right_rpm": data['right_rpm'],
            "system_voltage": 12.4
        })
    except:
        return jsonify({"status": "offline", "front_left_rpm": 0, "front_right_rpm": 0, "system_voltage": 0})

if __name__=='__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)