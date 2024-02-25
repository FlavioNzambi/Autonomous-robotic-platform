import sys
import traceback
from pathlib import Path
import diskcache as dc
from flask import Flask, render_template, Response, request

import cv2

app = Flask(__name__)

tmp = Path("/tmp/stream")

# Control variables
control_values = {'kp': 1, 'ki': 0.5, 'kd': 0.5, 'orangeThreshold': 30, 'roll': 330, 'throttle': 250}

def read_values_from_file(filename='config.txt'):
    try:
        with open(filename, 'r') as file:
            lines = file.readlines()
            values = {}
            for line in lines:
                key, value = line.strip().split('=')
                values[key] = float(value)
            return values
    except Exception as e:
        print(f"Error reading values from file: {e}")
        return {'kp': 1, 'ki': 0.5, 'kd': 0.5, 'orangeThreshold': 30, 'roll': 330, 'throttle': 250}

# Write values to file
def write_values_to_file(values, filename='config.txt'):
    try:
        with open(filename, 'w') as file:
            for key, value in values.items():
                file.write(f"{key}={value}\n")
    except Exception as e:
        print(f"Error writing values to file: {e}")

def generate():
    while True:
        try:
            with dc.Cache(tmp) as cache:
                print(f"[+] Ready to pull data from {tmp}")
              
                while True:
                    (key, value), _ = cache.pull(expire_time=True)
                    image_array = value 
                    if key:
                        _, jpeg = cv2.imencode('.jpg', image_array[...,::-1])

                        yield (b'--frame\r\n'
                                b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() +
                                b'\r\n\r\n' + f"Control Values: KP={control_values['kp']} KI={control_values['ki']} KD={control_values['kd']} "
                                f"Orange Threshold: {control_values['orangeThreshold']} Roll: {control_values['roll']} Throttle: {control_values['throttle']}".encode() + b'\r\n\r\n')

        except Exception as e:
            print(f"Error: {str(e)}")

@app.route('/')
def index():
    control_values = read_values_from_file()
    return render_template('index.html', control_values=control_values)

@app.route('/update_variables', methods=['POST'])
def update_variables():
    try:
        for key in request.form:
            if key in control_values:
                control_values[key] = float(request.form[key])
        write_values_to_file(control_values)
        return {'status': 'success'}
    except Exception as e:
        return {'status': 'error', 'message': str(e)}

@app.route('/video_feed')
def video_feed():
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=8051, debug=True, threaded=True)
