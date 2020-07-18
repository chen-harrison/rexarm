#!/usr/bin/python3
"""file for color detection"""

import numpy as np
import sys

colorNames = ["green", "red", "blue", "orange", "pink", "black", "purple", "yellow"]

colors = []

def initColors():
    global colors
    for cn in colorNames:
        colors.append(Color(cn))

class Color:
    """ class for describing a color"""
    def __init__(self, name):
        self.name = name
        self.load()

    def load(self):
        try:
            points = np.load("data/" + self.name + "_RGB.npy")
        except:
            print("Couldn't load color data for", self.name)
            sys.exit(1)
        if points.size == 0:
            self.info = np.identity(3)
            self.mean = np.ones((1,3)) * 128
        else:
            self.info = np.linalg.inv(np.cov(points, rowvar=False)) # inverse of covariance
            self.mean = np.average(points, axis=0) # 1x3

def chooseColor(point):
    """given an HSV 1x3 array, find which color it should be.
    Returns color name as a string"""
    global colors
    choice = "???"
    minD = 3
    for c in colors:
        d = np.sqrt((point - c.mean) @ c.info @ (point - c.mean).T)
        # if c.name == "red":
        #     print(point,d,sep='\n')
        if (d < minD):
            minD = d
            choice = c.name
    return choice, minD

if __name__ == "__main__":
    initColors()
    for color in colors:
        print("color", color.name)
        print("RGB", np.load("data/" + color.name + "_RGB.npy"))
        print("HSV", np.load("data/" + color.name + "_HSV.npy"))
        print("mean", color.mean)
        print("cov", np.round(np.linalg.inv(color.info),2))
    r = input("reset color? ")
    for color in colors:
        if color.name == r:
            print("resetting", color.name)
            a = np.array([])
            np.save("data/" + r + "_HSV.npy", a)
            np.save("data/" + r + "_RGB.npy", a)
