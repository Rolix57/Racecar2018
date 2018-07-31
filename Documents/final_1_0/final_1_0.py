#!/usr/bin/env python

from final_wall_follower import wallFollower as wf
from final_face_recognition import faceRecognition as fr
from final_color_detection import colorDetection as cd
from final_ar_tag_reader import arTagReader as artr
from final_control_module import controlModule as cm
from final_potential_field import potentialField as pf

switcher = {
    0: wf,
    1: fr,
    2: cd,
    3: artr,
    4: cm,
    5: pf
}

def routineSelection(value):
    return switcher.get(value, 0)

if __name__ == '__main__':
    routineSelection(artr.getValue())

    
