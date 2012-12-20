import pyglet
from pyglet.gl import *

import ctypes
import threading

import gobject
import gst

import cv


class Receiver(object):

    def __init__(self, (video_tex_id, tex_w, tex_h), 
                 pipeline_string, 
                 needdata,
                 circle_callback):
        self.tex_updated = True
        self.texdata = (ctypes.c_ubyte * 640 * 480 * 4)()
        self.video_tex_id = video_tex_id
        self.tex_size = (tex_w, tex_h)
        self.circle_callback = circle_callback
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX , 0.7, 0.7, 0, 1, 8)
        
        # Create GStreamer pipeline
        self.pipeline = gst.parse_launch(pipeline_string)

        # Create bus to get events from GStreamer pipeline
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message::eos', self.on_eos)
        self.bus.connect('message::error', self.on_error)

        self.fakesink = self.pipeline.get_by_name('fakesink0')
        self.fakesink.props.signal_handoffs = True
        self.fakesink.connect("handoff", self.on_gst_buffer)

        appsrc = self.pipeline.get_by_name('appsrc0')
        if appsrc is not None and needdata is not None:
            appsrc.connect('need-data', needdata)

        self.pipeline.set_state(gst.STATE_PLAYING)
        self.lock = threading.Lock()


    def on_gst_buffer(self, fakesink, buff, pad, data=None):
        with self.lock:
            self.tex_updated = False
            ctypes.memmove(self.texdata, buff.data, buff.size)
        return gst.FLOW_OK


    def draw_circles(self, storage, output):
        cx = 0
        cy = 0
        r = 0
        if storage.rows > 0:
            # Calculate average center and radius.  Only process the
            # first 5 or less because usually there should be 1 or 2
            # circles detected. If there is more, then cv.HoughCircles
            # parameters should be adjusted
            for r in range(min(storage.rows, 5)):
                p = storage[r,0]
                cx += cv.Round(p[0])
                cy += cv.Round(p[1])
                r += cv.Round(p[2])
            cx /= storage.rows
            cy /= storage.rows
            r /= storage.rows
            cv.Circle(output, (cx, cy), 1, cv.CV_RGB(0, 255, 0), -1, 8, 0)
            cv.Circle(output, (cx, cy), r, cv.CV_RGB(0, 0, 255), 3, 8, 0)
            cv.PutText(output, 'loc: %(x)03d %(y)03d r: %(r)03d' % {"x":cx, "y":cy, "r":r}, (cx+5, cy), self.font, cv.CV_RGB(0, 255, 0))

            # Invoke the callback to let application process detected
            # circle
        self.circle_callback(cx, cy, r)


    def updateTexture(self):
        if not self.tex_updated and not self.texdata == None:
            with self.lock:
                orig = cv.CreateImageHeader((640,480), cv.IPL_DEPTH_8U, 4)
                cv.SetData(orig, self.texdata)
                processed = cv.CreateImage((640,480), cv.IPL_DEPTH_8U, 1)
                cv.CvtColor(orig, processed, cv.CV_BGRA2GRAY)

                storage = cv.CreateMat(orig.width, 1, cv.CV_32FC3)

                # Simetimes canny can help since HoughCircles seems to
                # prefer ring like circles to filled ones.  
                cv.Canny(processed, processed, 5, 70, 3)

                # smooth to reduce noise a bit more
                cv.Smooth(processed, processed, cv.CV_GAUSSIAN, 7, 7)

                # find circles
                cv.HoughCircles(processed, 
                                storage, 
                                cv.CV_HOUGH_GRADIENT, 
                                2, 32, 
                                200, 340,
                                10, 0)
                cv.CvtColor(processed, orig, cv.CV_GRAY2BGRA)
                self.draw_circles(storage, orig)

                glBindTexture(GL_TEXTURE_2D, self.video_tex_id)
                glTexSubImage2D(GL_TEXTURE_2D, 
                                0, 0, 0, 
                                640, 480, 
                                GL_RGBA, GL_UNSIGNED_BYTE, 
                                self.texdata)
                self.tex_updated = True
      
        
    def on_eos(self, bus, msg):
        print('on_eos(): seeking to start of video')
#        self.pipeline.seek_simple(
#            gst.FORMAT_TIME,        
#            gst.SEEK_FLAG_FLUSH | gst.SEEK_FLAG_KEY_UNIT,
#            0L
#        )


    def on_error(self, bus, msg):
        print('on_error():', msg.parse_error())


    def cleanup(self):
        print 'Video receiver cleaning up...'
        self.pipeline.set_state(gst.STATE_NULL)
        print 'Video receiver uninitialized.'
