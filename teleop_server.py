

import os
import sys


import json
from flask import Flask
from flask_cors import CORS, cross_origin
from yaks import Yaks, Value, Encoding


CONTROL_RESOURCE = '/turtlebot/move'
STATE_RESOURCE = '/turtlebot/status'
SENSOR_RESOURCE = '/turtlebot/sensors'

app = Flask(__name__)
cors = CORS(app)
app.config['CORS_HEADERS'] = 'Content-Type'

def reponse():
    res = ws.get(STATE_RESOURCE)
    print(res)
    if len(res) > 0:
        v = res[0][1].value

        return v
    return ""

@app.route('/', methods=['GET'])
@cross_origin()
def index():
    return reponse()

@app.route('/sensors', methods=['GET'])
@cross_origin()
def sensors():
    res = ws.get(SENSOR_RESOURCE)
    print(res)
    if len(res) > 0:
        v = res[0][1].value
        d = json.loads(v)
        r = reponse()
        if r != "":
            d2 = json.loads(r)
            d.update(d2)
        return json.dumps(d)
    return ""

@app.route('/fwd', methods=['POST'])
@cross_origin()
def fwd():
    v = Value('fwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()


@app.route('/bwd', methods=['POST'])
@cross_origin()
def bwd():
    v = Value('bwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/stop', methods=['POST'])
@cross_origin()
def stop():
    v = Value('h', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/sx', methods=['POST'])
@cross_origin()
def sx():
    v = Value('sx', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/dx', methods=['POST'])
@cross_origin()
def dx():
    v = Value('dx', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()


def main():

    yip = sys.argv[1]
    global yaks
    yaks = Yaks.login(yip)
    global ws
    ws = yaks.workspace('/')

    app.run(debug=True,host='0.0.0.0')


if __name__ == '__main__':
    main()
