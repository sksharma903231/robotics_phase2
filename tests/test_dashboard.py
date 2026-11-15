import os
import sys
import json
import pytest

# Add dashboard and scripts to python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(project_root, 'dashboard'))
sys.path.insert(0, os.path.join(project_root, 'ros2_ws', 'src', 'rover_sim_pkg', 'scripts'))

from app import app
from telemetry_bridge import rad_per_sec_to_rpm, extract_wheel_rpms

@pytest.fixture
def client(tmp_path):
    app.config['TESTING'] = True
    test_telemetry_file = str(tmp_path / "rover_telemetry.json")
    
    # Configure app to use temp telemetry file
    import app as app_module
    old_path = app_module.TELEMETRY_PATH
    old_mock = app_module.MOCK_MODE
    app_module.TELEMETRY_PATH = test_telemetry_file
    app_module.MOCK_MODE = False

    with app.test_client() as client:
        yield client, test_telemetry_file

    app_module.TELEMETRY_PATH = old_path
    app_module.MOCK_MODE = old_mock

def test_rad_to_rpm_conversion():
    assert rad_per_sec_to_rpm(0.0) == 0.0
    # 2 * pi rad/s = 1 rev/s = 60 RPM
    assert rad_per_sec_to_rpm(6.2831853) == 60.0
    assert rad_per_sec_to_rpm(1.0) == 9.55

def test_extract_wheel_rpms_named():
    names = ['right_wheel_joint', 'left_wheel_joint']
    vels = [2.0, 5.0]
    result = extract_wheel_rpms(names, vels)
    assert result['left_rpm'] == rad_per_sec_to_rpm(5.0)
    assert result['right_rpm'] == rad_per_sec_to_rpm(2.0)

def test_extract_wheel_rpms_positional():
    names = []
    vels = [3.0, 4.0]
    result = extract_wheel_rpms(names, vels)
    assert result['left_rpm'] == rad_per_sec_to_rpm(3.0)
    assert result['right_rpm'] == rad_per_sec_to_rpm(4.0)

def test_dashboard_home(client):
    c, _ = client
    response = c.get('/')
    assert response.status_code == 200
    assert b"Rover Control Room" in response.data

def test_dashboard_telemetry_offline(client):
    c, path = client
    if os.path.exists(path):
        os.remove(path)
    response = c.get('/api/telemetry')
    assert response.status_code == 200
    data = response.get_json()
    assert data['status'] == 'offline'
    assert data['front_left_rpm'] == 0.0
    assert data['front_right_rpm'] == 0.0

def test_dashboard_telemetry_online(client):
    c, path = client
    payload = {"left_rpm": 48.5, "right_rpm": 22.1, "system_voltage": 12.3}
    with open(path, 'w') as f:
        json.dump(payload, f)

    response = c.get('/api/telemetry')
    assert response.status_code == 200
    data = response.get_json()
    assert data['status'] == 'online'
    assert data['front_left_rpm'] == 48.5
    assert data['front_right_rpm'] == 22.1
    assert data['system_voltage'] == 12.3

def test_dashboard_post_telemetry(client):
    c, path = client
    payload = {"left_rpm": 30.0, "right_rpm": 15.0}
    response = c.post('/api/telemetry', data=json.dumps(payload), content_type='application/json')
    assert response.status_code == 200

    # Verify get reflects newly posted telemetry
    get_res = c.get('/api/telemetry')
    assert get_res.get_json()['front_left_rpm'] == 30.0
    assert get_res.get_json()['front_right_rpm'] == 15.0
