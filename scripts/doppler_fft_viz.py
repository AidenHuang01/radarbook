#!/usr/bin/env python
PKG = 'radarbook'
import roslib; roslib.load_manifest(PKG)
import cv2
import numpy as np
import sys
import threading
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg


# def callback_data(data):
#     print (rospy.get_name(), "Data received from radarbook2 node::Data %s"%str(data.data))

# def callback_RP(data):
#     print (rospy.get_name(), "Data received from radarbook2 node::RP %s"%str(data.data))

# def callback_RDMap(data):
#     print (rospy.get_name(), "Data received from radarbook2 node::RP %s"%str(data.data))
#     self.fb.write_frame(fft_mag)


# def listener():
#     cv2.namedWindow('doppler_fft_viz', cv2.WINDOW_FREERATIO)
#     rospy.init_node('doppler_fft_viz_listener')
#     rospy.Subscriber("radarbook/RDMap", numpy_msg(Floats), callback_data)
#     rospy.spin()





class FrameBuffer():
    #DoubleBuffer for for vis
    def __init__(self, framesize=(331,64)):
        self.buff = [np.zeros(framesize),
                     np.zeros(framesize),
                    ]

        self.dirty = True
        self.frontbuff = 0

    def write_frame(self, new_frame):
        backbuff = (self.frontbuff+1)%2
        self.buff[backbuff] = new_frame
        self.frontbuff = backbuff
        self.dirty = True

    def get_frame(self):
        d = self.dirty
        self.dirty = False
        return d, self.buff[self.frontbuff]

class radarbook_fftviz:
    def __init__(self, fb):
        self.subscriber = rospy.Subscriber("radarbook/RDMap", numpy_msg(Floats), self.callback_RDMap)
        print("subscribed to radarbook RDMap")
        self.fb = fb

    def callback_RDMap(self, data):
        flat_arr = data.data.flatten()
        RDMap = np.reshape(flat_arr, (331, 512))
        self.fb.write_frame(RDMap)
        # print(data.data.shape)
        
def normalize_and_color(data, max_val=0, cmap=None):
    data_max = data.max()
    data_min = data.min()
    data_norm = (data - data_min) / (data_max - data_min)
    img = data_norm
    if data_norm.min() < 0:
        img = (1 + data_norm) * 255 
        img = img.astype(np.uint8)
    # if cmap:
    #     img = cv2.applyColorMap(img, cmap)
    return img

def imshow_thread(framebuffer, max_val=0.0, cmap=cv2.COLORMAP_WINTER):
    #k = cv2.waitKey(1)

    cv2.namedWindow('doppler_fft_viz', cv2.WINDOW_FREERATIO)

    ax = 2

    while not rospy.is_shutdown():
        update, img = framebuffer.get_frame()
        
        if update:
            img = normalize_and_color(img, max_val, cmap)
            # print(img)
            
            cv2.imshow('doppler_fft_viz', img)
        cv2.waitKey(1)

    cv2.destroyAllWindows()




def main(args):
    rospy.init_node('doppler_fft_viz_listerner', anonymous=True)

    fb = FrameBuffer()
    fft_viz = radarbook_fftviz(fb)

    ui_thread = threading.Thread(target=imshow_thread, args=(fb,))
    ui_thread.setDaemon(True)
    ui_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("End FFTVIZ")

if __name__ == '__main__':
    main(sys.argv)