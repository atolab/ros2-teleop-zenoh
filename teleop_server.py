

import os
import sys


import json
from flask import Flask
from yaks import Yaks, Value, Encoding


CONTROL_RESOURCE = '/turtlebot/move'
STATE_RESOURCE = '/turtlebot/status'

app = Flask(__name__)

def reponse():
    res = ws.get(STATE_RESOURCE)
    print(res)
    if len(res) > 0:
        v = res[0][1].value
        return v
    return ""

@app.route('/', methods=['GET'])
def index():
    return reponse()

@app.route('/fwd', methods=['POST'])
def fwd():
    v = Value('fwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()


@app.route('/bwd', methods=['POST'])
def bwd():
    v = Value('bwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/stop', methods=['POST'])
def stop():
    v = Value('h', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/sx', methods=['POST'])
def sx():
    v = Value('sx', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return reponse()

@app.route('/dx', methods=['POST'])
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
