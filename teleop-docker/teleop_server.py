# Copyright (c) 2014,2020 ADLINK Technology Inc.
#
# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Author: Gabriele Baldoni

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

def response():
    res = ws.get(STATE_RESOURCE)
    print(res)
    if len(res) > 0:
        v = res[0].get_value().get_value()
        return v
    return ""

@app.route('/', methods=['GET'])
@cross_origin()
def index():
    return response()

@app.route('/sensors', methods=['GET'])
@cross_origin()
def sensors():
    return response()

@app.route('/fwd', methods=['POST'])
@cross_origin()
def fwd():
    v = Value('fwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return response()


@app.route('/bwd', methods=['POST'])
@cross_origin()
def bwd():
    v = Value('bwd', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return response()

@app.route('/stop', methods=['POST'])
@cross_origin()
def stop():
    v = Value('h', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return response()

@app.route('/sx', methods=['POST'])
@cross_origin()
def sx():
    v = Value('sx', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return response()

@app.route('/dx', methods=['POST'])
@cross_origin()
def dx():
    v = Value('dx', Encoding.STRING)
    ws.put(CONTROL_RESOURCE, v)
    return response()


def main():

    yip = sys.argv[1]
    global yaks
    yaks = Yaks.login(yip)
    global ws
    ws = yaks.workspace('/')

    app.run(debug=True,host='0.0.0.0')


if __name__ == '__main__':
    main()
