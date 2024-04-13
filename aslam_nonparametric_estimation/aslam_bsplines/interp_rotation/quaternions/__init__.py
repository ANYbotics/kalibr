import sm_python
import numpy


def qdot(q1,q2):
    return numpy.dot(sm_python.quatPlus(q1),q2)

def qinv(q):
    return sm_python.quatInv(q)

def qlog(q):
    return sm_python.quat2AxisAngle(q)

def qexp(a):
    return sm_python.axisAngle2quat(a)