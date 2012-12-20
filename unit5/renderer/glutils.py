import math
import pyglet
from pyglet.gl import *
import ctypes


def VecF(*args):
    """Simple function to create ctypes arrays of floats"""
    return (GLfloat * len(args))(*args)

def getOpenGLVersion():
    """Get the OpenGL minor and major version number"""
    versionString = glGetString(GL_VERSION)
    return ctypes.cast(versionString, ctypes.c_char_p).value

def getGLError():
    e = glGetError()
    if e != 0:
        errstr = gluErrorString(e)
        print 'GL ERROR:', errstr
        return errstr
    else:
        return None

def createProjectionMatrix(fovY, aspect, zNear, zFar):
    """Creates perspective projection matrix. fovY should be in radians"""
    f = 1.0 / math.tan(fovY/2.0) #cotangent(fovY/2)
    dz = zNear - zFar
    m = []
    m.extend([f/aspect, 0.0,                 0.0,                       0.0])
    m.extend([0.0,        f,                 0.0,                       0.0])
    m.extend([0.0,      0.0, (zFar + zNear) / dz, (2.0 * zFar * zNear) / dz])
    m.extend([0.0,      0.0,                -1.0,                       0.0])
    return m
