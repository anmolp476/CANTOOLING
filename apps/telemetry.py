import json
import requests
import time
import paho.mqtt.client as mqtt
import threading
from rascan.reader import CANReader
import os
from datetime import datetime, timezone
from dotenv import load_dotenv
load_dotenv()

BROKER_HOST = os.getenv('BROKER_HOST')
BROKER_PORT = os.getenv('BROKER_PORT')
USERNAME = os.getenv('USERNAME')
PASSWORD = os.getenv('PASSWORD')
VEHICLE_ID = os.getenv('VEHICLE_ID')
TOPIC = "incidents/reports"

api_lock = threading.Lock()
latest_api_data = {
    "driver_status": "ALERT",
    "head_direction": "EYES OPEN",
    "observation_complete": True
}

# risk thresholds
RISK_RULES = [
    {'name': 'Over-speed', 'check': lambda s: s.get('Speed', 0) > 120, 'level': 'High'},
    {'name': 'Aggressive throttle', 'check': lambda s: s.get('Throttle', 0) > 80, 'level': 'Medium'},
    {'name': 'Hard braking', 'check': lambda s: s.get('Brake', 0) > 70, 'level': 'Medium'},
    {'name': 'Sharp steering', 'check': lambda s: abs(s.get('Steering', 0)) > 30, 'level': 'Medium'},
    {'name': 'No turn signal while turning', 'check': lambda s: abs(s.get('Steering', 0)) > 10 and not (s.get('Left') or s.get('Right')), 'level': 'Low'},
    {'name': 'High-speed swerving', 'check': lambda s: s.get('Speed', 0) > 80 and abs(s.get('Steering', 0)) > 20, 'level': 'High'},
    {'name': 'Hard acceleration', 'check': lambda s: s.get('Speed', 0) < 20 and s.get('Throttle', 0) > 85, 'level': 'Medium'},
    {'name': 'Unsafe reverse speed', 'check': lambda s: s.get('Speed', 0) < -15, 'level': 'Medium'},
    {'name': 'Unsafe reverse speed', 'check': lambda s: s.get('Speed', 0) < -15, 'level': 'Medium'},
    {'name': 'Late braking reaction (Distracted)', 'check': lambda s: s.get('Brake', 0) > 60 and s.get('head_direction') == 'LOST', 'level': 'High'},
    {'name': 'Unsignaled lane departure', 'check': lambda s: s.get('Speed', 0) > 60 and 5 < abs(s.get('Steering', 0)) < 15 and not (s.get('Left') or s.get('Right')), 'level': 'Medium'},
]

ATTENTION_RULES = [
    {'name': 'Drowsy and inattentive', 'check': lambda s: s['driver_status'] == 'DROWSY' and s['head_direction'] == 'LOST', 'level': 'High'},
    {'name': 'Drowsy with failed observation', 'check': lambda s: s['driver_status'] == 'DROWSY' and s['observation_complete'] is False, 'level': 'High'},
    {'name': 'Alert but completely distracted', 'check': lambda s: s['driver_status'] == 'ALERT' and s['head_direction'] == 'LOST', 'level': 'Medium'},
    {'name': 'Driver is drowsy', 'check': lambda s: s['driver_status'] == 'DROWSY' and s['head_direction'] in ['FORWARD', 'LEFT', 'RIGHT'], 'level': 'Medium'},
    {'name': 'Failed to complete observation',  'check': lambda s: s['driver_status'] == 'ALERT' and s['head_direction'] in ['FORWARD', 'LEFT', 'RIGHT'] and s['observation_complete'] is False, 'level': 'Low'}
]

def mqtt_setup():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set(USERNAME, PASSWORD)
    client.tls_set()

    def on_connect(client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print("[SYSTEM] Connected to MQTT broker")
        else:
            print(f"[ERROR] MQTT Connection failed with code {reason_code}")

    client.on_connect = on_connect
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.loop_start() # Runs the network loop in a background thread
    return client

def publish_risk_event(mqtt_client, result):
    mqtt_client.publish(TOPIC, json.dumps(result), qos=1)
    print(f"[MQTT] Published risk payload: {result['Risk Level']}")

def cam_polling(hz=24):
    global latest_api_data
    interval = 1.0 / hz
    
    # Using a Session keeps the connection open, making 24Hz polling much faster
    session = requests.Session()
    
    while True:
        start_time = time.time()
        
        try:
            # 30ms timeout ensures we don't block past our 41.6ms (24hz) window
            response = session.get("http://localhost:8000/current_status", timeout=0.03)
            if response.status_code == 200:
                data = response.json()
                # Safely update the shared dictionary
                with api_lock:
                    latest_api_data.update(data)
        except requests.exceptions.RequestException:
            # If API drops a frame or server lags, ignore it and try again next tick
            pass

        # Calculate exact sleep time to maintain 24 Hz without drifting
        elapsed = time.time() - start_time
        sleep_time = interval - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

def camera_check(): 
    risks = []
    for rule in ATTENTION_RULES:
        if rule['check'](latest_api_data):
                risks.append({'behavior': rule['name'], 'level': rule['level']})
    return risks
            
def evaluate_risk(temp_frame):
    # merge all signals
    signals_combined = {}
    signals_combined.update(temp_frame.get(384, {}))
    signals_combined.update(temp_frame.get(644, {}))
    signals_combined.update(temp_frame.get(645, {}))
    signals_combined.update(temp_frame.get(658, {}))
    signals_combined.update(temp_frame.get(1549, {}))
    signals_combined.update(temp_frame.get(2, {}))
    signals_combined.update(temp_frame.get(1057, {}))
    signals_combined.update(temp_frame.get(1477, {}))
    #write raw signals file
    raw_signals_path = 'raw_decoded_signals.json'
    with open(raw_signals_path, 'w') as json_file:
        json.dump(signals_combined , json_file, indent=4)

    risks = []
    for rule in RISK_RULES:
        check_signals = signals_combined.copy()
        if rule['check'](check_signals):
            risks.append({'behavior': rule['name'], 'level': rule['level']})

    risks.extend(camera_check())

    # Determine overall risk
    if any(r['level'] == 'High' for r in risks):
        overall_risk = 'High'
    elif any(r['level'] == 'Medium' for r in risks):
        overall_risk = 'Medium'
    elif any(r['level'] == 'Low' for r in risks):
        overall_risk = 'Low'
    else:
        overall_risk = 'Safe'

    return {
        "device_id":VEHICLE_ID,
        "risk_types": ", ".join(r['behaviour'] for r in risks),
        "location": {"lat": 43.6564398, "lon": -79.3767335},
        "severity": overall_risk,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        'additional_data': signals_combined,
       
    }

def main():
    mqtt_client = mqtt_setup()
    camera_thread = threading.Thread(target=cam_polling, daemon=True)
    camera_thread.start()
    reader = CANReader("dbc/nissan_versa_2014.dbc")
    temp_frame = {}  # holds full frame with ids 384, 644, 645, 658, 1549, 2, 1057, 1477
    try: 
        with open(categorized_path, 'w') as json_file:  
            while True:
                frame = reader.read()           
                if frame:
                    frame_id = frame['id']
                    signals = frame['signals']
                    categorized_path = 'categorized_data.json'              
                    if frame_id in [384, 644, 645, 658, 1549, 2, 1057, 1477]:
                        temp_frame[frame_id] = signals

                    # Once we have all 8 messages, evaluate
                    if all(k in temp_frame for k in [384, 644, 645, 658, 1549, 2, 1057, 1477]):
                        result = evaluate_risk(temp_frame) 
                        if result['Risk Level'] != 'Safe':
                            publish_risk_event(mqtt_client, result)
                        #writes latest frame to file 
                        json.dump(result , json_file, indent=4)
                        print(json.dumps(result, indent=4))
                        temp_frame = {}  # reset for next frame
    except KeyboardInterrupt: 
        print("\n[SYSTEM] Shutting Down...")
    finally: 
        mqtt_client.loop_stop()
        mqtt_client.disconnect()

if __name__ == "__main__":
    main()