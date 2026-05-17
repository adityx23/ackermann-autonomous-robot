def test_core_imports():
    import numpy
    import scipy
    import cv2
    import depthai
    import serial
    import smbus2
    import can
    import zmq
    import fastapi
    import psutil
    import yaml

    assert numpy is not None
    assert scipy is not None
    assert cv2 is not None
    assert depthai is not None
    assert serial is not None
    assert smbus2 is not None
    assert can is not None
    assert zmq is not None
    assert fastapi is not None
    assert psutil is not None
    assert yaml is not None
