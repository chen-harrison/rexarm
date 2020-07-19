"""!
Class to represent the kinect.
"""

import cv2
import numpy as np
from PyQt4.QtGui import QImage
import freenect
import os
import sys
from kinematics import board_z, block_size, board_width
script_path = os.path.dirname(os.path.realpath(__file__))
import color

font = cv2.FONT_HERSHEY_SIMPLEX

class Kinect():
    """!
    @brief      This class describes a kinect.
    """

    def __init__(self):
        """!
        @brief      Constructs a new instance.
        """
        self.VideoFrame = np.array([])
        self.VideoFrameHSV = np.array([])
        self.DepthFrameRaw = np.array([]).astype(np.uint16)
        """ Extra arrays for colormapping the depth image"""
        self.DepthFrameHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRGB = np.array([])
        self.DepthFrameFiltered = np.array([])

        """initialize kinect & turn off auto gain and whitebalance"""
        freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_MEDIUM)
        # print(freenect.sync_set_autoexposure(False))
        freenect.sync_set_autoexposure(False)
        # print(freenect.sync_set_whitebalance(False))
        freenect.sync_set_whitebalance(False)
        """check depth returns a frame, and flag kinectConnected"""
        if(freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT) == None):
            self.kinectConnected = False
        else:
            self.kinectConnected = True

        self.kinectCalibrated = False
        # mouse clicks & calibration variables
        # self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]]) #no transform
        # self.depth2rgb_affine = np.float32([[  9.28726252E-1,   -1.14277108E-2,  -2.06562788],
        #                                     [ 6.51754476E-3,   9.21278359E-1,   4.02982221E+1]]) # older version

        self.depth2rgb_affine = np.float32([[  9.21074557E-1,   -9.91213238E-3,  -2.15895387E-1],
                                            [ 3.72252283E-3,   9.19210560E-1,   4.14502181E+1]]) # determined from test_kinect


        """ inverse extrinsic matrix """
        self.loadCameraCalibration("/home/student/armlab-w20/util/calibration.cfg")
        self.getWorkspaceBoundary()

        self.last_click = np.array([0,0])
        self.last_rclick = np.array([0,0])
        self.new_click = False
        self.new_rclick = False
        self.rgb_click_points = np.zeros((5,2), np.float32)
        self.depth_click_points = np.zeros((5,2), int)

        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([])
        self.blocks = []
        color.initColors()


    def toggleExposure(self, state):
        """!
        @brief      Toggle auto exposure

        @param      state  False turns off auto exposure True turns it on
        """
        if state == False:
            freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_MEDIUM)
            # print(freenect.sync_set_autoexposure(False))
            freenect.sync_set_autoexposure(False)
            # print(freenect.sync_set_whitebalance(False))
            freenect.sync_set_whitebalance(False)
        else:
            freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_MEDIUM)
            # print(freenect.sync_set_autoexposure(True))
            freenect.sync_set_autoexposure(True)
            # print(freenect.sync_set_whitebalance(True))
            freenect.sync_set_whitebalance(True)

    def captureVideoFrame(self):
        """!
        @brief Capture frame from Kinect, format is 24bit RGB
        """
        if(self.kinectConnected):
           self.VideoFrame = freenect.sync_get_video_with_res(resolution=freenect.RESOLUTION_MEDIUM)[0]
        else:
            self.loadVideoFrame()
        self.VideoFrameHSV = cv2.cvtColor(self.VideoFrame, cv2.COLOR_RGB2HSV)


    def processFrame(self, frame):
        """!
        @brief      Process a frame by adding contours
        """
        cv2.drawContours(frame,self.block_contours,-1,(100,100,255),1)
        for block in self.blocks:
            textX = np.min(block.contour[:,0]) #find the min X and Y to put the text at
            textY = np.min(block.contour[:,1])
            text = str(block.color) + ' ' + str(block.stackHeight) +' '+ str(round(block.angle, 1))
            cv2.putText(frame,text, (textX, textY), font, 0.4, (0,0,0))
        cv2.drawContours(frame, [self.workBound], -1, (255,0,255), 2)


    def captureDepthFrame(self):
        """!
        @brief Capture depth frame from Kinect, format is 16bit Grey, 10bit resolution.
        """
        if(self.kinectConnected):
            if(self.kinectCalibrated):
                self.DepthFrameRaw = self.registerDepthFrame(freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT)[0])
            else:
                self.DepthFrameRaw = freenect.sync_get_depth_with_res(format = freenect.DEPTH_11BIT)[0]
        else:
            self.loadDepthFrame()

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        #DepthFrameResize = cv2.resize(self.DepthFrameRaw, (640, 480))
        self.DepthFrameHSV[...,0] = self.DepthFrameRaw
        self.DepthFrameHSV[...,1] = 0x9F
        self.DepthFrameHSV[...,2] = 0xFF
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread(script_path + "/data/rgb_image.png",cv2.IMREAD_UNCHANGED),cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread(script_path + "/data/raw_depth.png",0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """
        self.processFrame(self.VideoFrame)   # add contours for display

        try:
            frame = cv2.resize(self.VideoFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertQtDepthFrame(self):
       """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
       self.processFrame(self.DepthFrameRGB)   # add contours for display
       try:
           img = QImage(self.DepthFrameRGB,
                            self.DepthFrameRGB.shape[1],
                            self.DepthFrameRGB.shape[0],
                            QImage.Format_RGB888
                            )
           return img
       except:
           return None

    def convertQtFilteredFrame(self):
       """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
       frame = np.zeros((480,640, 3)).astype(np.uint8)
       frame[...,0] = 1
       frame[...,1] = 0
       frame[...,2] = self.DepthFrameFiltered
       frame = cv2.cvtColor(frame,cv2.COLOR_HSV2RGB)
       img = QImage(frame,
                        self.DepthFrameFiltered.shape[1],
                        self.DepthFrameFiltered.shape[0],
                        QImage.Format_RGB888
                        )
       return img

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

        @param      coord1  The coordinate 1 (nx2 numpy array)
        @param      coord2  The coordinate 2 (nx2 numpy array)

        affine*coord1 = coord2

        @return     Affine transform between coordinates.
        """
        if coord1.shape[0] != coord2.shape[0]:
            print("size of coord1 and coord2 do not match!")
            return -1

        A = np.zeros((2*coord1.shape[0], 6))
        coord1_homog = np.concatenate((coord1, np.ones([coord1.shape[0],1])), axis=1)

        for i in range(coord1.shape[0]):
            A[2*i,0:3] = coord1_homog[i,:]
            A[2*i + 1,3:6] = coord1_homog[i,:]

        b = coord2.flatten()

        x = np.linalg.inv(A.T @ A) @ A.T @ b

        affine = x.reshape((2,3))
        return affine


    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

        @param      frame  The frame

        @return     the transformed frame
        """
        frame = cv2.warpAffine(frame, self.depth2rgb_affine, (640,480))
        return frame

    def loadCameraCalibration(self, file):
        """!
        @brief      Load camera intrinsic matrix from file.

        @param      file  The file
        """
        try:
            with open(file) as inFile:
                allFile = inFile.read()
                start = allFile.index("matrix:") + len("matrix:") + 1
                end = allFile.index("distortion") - 1
        except IOError:
            print("Could not open", file)
        except:
            print("Could not read calibration.cfg data :(")
        else:
            # idk if this is the best way to read it but it works
            arrStr = allFile[start:end].replace('[','').replace(']','').replace(',','').replace('\n',' ')
            intrinsic = np.reshape(np.fromstring(arrStr, sep=' '), (3,3))
            self.inv_intrinsic = np.linalg.inv(intrinsic)
        try:
            self.inv_extrinsic = np.load("util/extrinsic.npy")
        except:
            print("Couldn't load extrinsic matrix")
            self.inv_extrinsic = np.identity(4)
        else:
            print("Calibration Loaded")
            self.kinectCalibrated = True

    def getWorkspaceBoundary(self):
        """!
        @brief after calibration, find the locations (in pixels) in the images
        that represent the edge of the wooden board. Saved in self.workBound
        as a 4x2 matrix: (u,v) for each of bottom left, top left, top right, bottom left
        """
        self.workBound = np.zeros((4,2), dtype = np.int)
        b_w = board_width / 2
        model_points = np.array([[-b_w, -b_w, board_z, 1], [-b_w, b_w, board_z, 1],
                                 [b_w, b_w, board_z, 1], [b_w, -b_w, board_z, 1]])
        for i in range(4):
            cf = (np.linalg.inv(self.inv_extrinsic) @ model_points[i,:])[0:3] # camera frame
            cf = cf / cf[2] # normalize by z
            self.workBound[i,:] = np.round((np.linalg.inv(self.inv_intrinsic) @ cf))[0:2].astype(np.int)

    def depthToMeter(self, u, v):
        """!
        @brief      given the pixels of a point, find the depth Z

        @param      u and v, offset in the image

        @return     Z in meters
        """
        d = self.DepthFrameRaw[v,u]
        return 0.1236*np.tan(d/2842.5 + 1.1863)

    def meterToDepth(self, Z):
        """ given Z in meters, returns the value of d that the depth cam should have"""
        Z = Z - 0.03 # this is the thing at the end of pix2Glob
        return 2842.5*(np.arctan(Z/0.1236) - 1.1863)

    def pix2Glob(self, pix, depthPix = None):
        """!
        @brief      given the pixels of a point, find the X Y Z of that image in global coordinates

        @param      pix  the pixels as [u, v, 1]
        @param      depthVal    the depth of that point. If not set, gets it from the depth image.

        @return     a vector of [X, Y, Z, 1] in global coordinates
        """
        camera_coords = np.ones(4) # 4 because it's homogenous
        if depthPix == None:
            depthVal = self.depthToMeter(pix[0], pix[1])
        else:
            depthVal = self.depthToMeter(depthPix[0], depthPix[1])
        camera_coords[0:3] = depthVal * self.inv_intrinsic @ pix
        return self.inv_extrinsic @ camera_coords + np.array([0,0,0.03,0])

    def clearSurroundings(self, frame):
        """!
        @brief     take a frame and make it black outside of the boundary defined by self.workBound
        @param     frame: the frame to use. This function edits the frame in place
        @return    the edited frame
        """
        stencil = np.zeros(frame.shape).astype(frame.dtype)
        cv2.fillPoly(stencil, [self.workBound], [255,255,255])
        frame[:] = cv2.bitwise_and(frame, stencil)
        return frame

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        self.detectBlocksInDepthImage()
        sw = 4 # half width of the square in the middle of the block that's used for color
        # find the color of each block
        for block in self.blocks:
            # blockArea is the 2*sw by 2*sw section of the block used for getting color
            blockArea = self.VideoFrame[block.pixY - sw : block.pixY + sw, block.pixX - sw : block.pixX + sw, :]
            point = np.average(blockArea, axis=(0,1)).reshape((1,3)) #take the average of the area
            block.color, block.mah = color.chooseColor(point)
        # TODO if we have time: if the block is big, it's probably 2 blocks, use RGB to split them apart
        # TODO if we have time: if the block is big and not monochrome, it's probably not a block, it's the arm

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth image.
                    writes to self.blocks, self.block_contours
        """
        board_depth_d = 725
        block_depth_d = [7.5, 23.5, 41, 60.5, 82]
        block_pwidth = 22   # TODO: get this from actual block depth
        smoothing_size = 3  # number of pixels for smoothing (more = smoother)
        maxBlockHeight = 5  # maximum number of stacked blocks to detect
        kernel = np.ones((smoothing_size,smoothing_size),np.uint8) # kernel for smoothing

        self.blocks = []
        self.block_contours = []
        lastFilterFrame = np.zeros(self.DepthFrameRaw.shape, dtype=np.uint8) # initialize last frame to having nothing
        for i in range(maxBlockHeight, 0, -1): # detect blocks stacked up to 5 high, starting from the top
            # remember, subtracting d makes it stacked higher
            d = board_depth_d - block_depth_d[i-1] # half a block's height less than the top
            self.DepthFrameFiltered = ((self.DepthFrameRaw < d) * 255).astype(np.uint8)
            self.clearSurroundings(self.DepthFrameFiltered) # remove the area outside the board
            # now, subtract the last image, so that this only has the block stacks that end on this level
            self.DepthFrameFiltered -= lastFilterFrame

            self.DepthFrameFiltered = cv2.morphologyEx(self.DepthFrameFiltered, cv2.MORPH_OPEN, kernel)  # remove white dots
            self.DepthFrameFiltered = cv2.morphologyEx(self.DepthFrameFiltered, cv2.MORPH_CLOSE, kernel) # remove black dots
            # now self.DepthFrameFiltered should have squares for all the blocks at this height

            tmp = self.DepthFrameFiltered.copy()
            tmp, contours, tmph = cv2.findContours(tmp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                rect = cv2.minAreaRect(contour)
                cntArea = cv2.contourArea(contour)
                rectCnt = cv2.boxPoints(rect).astype(np.int32)
                # if we think this is a valid block. TODO: make this better
                if (cntArea > (block_pwidth ** 2) * .75 and     # if area is at least 1/2 of a block
                    cntArea > .65 * cv2.contourArea(rectCnt)):  # if significant fraction of the rectangle is occupied
                    self.block_contours.append(rectCnt)
                    self.blocks.append(Block(rectCnt, i, rect[2]*np.pi/180, rect[0][0], rect[0][1]))
                    self.blocks[-1].getCoords(self) # get the x,y,z transform
            lastFilterFrame |= self.DepthFrameFiltered

class Block:
    """ class for detected blocks"""
    def __init__(self, contour, stackHeight, angle, pixX, pixY, color=None):
        self.contour = contour
        self.stackHeight = stackHeight
        self.angle = angle
        self.color = color
        self.pixX = int(pixX)
        self.pixY = int(pixY)
        self.x = None
        self.y = None
        self.z = None
    def getCoords(self, kinect):
        """get self.x, y, and z from other info"""
        pix = np.array([self.pixX, self.pixY, 1])
        homog = kinect.pix2Glob(pix)
        self.x = homog[0]
        self.y = homog[1]
        self.z = homog[2] - block_size/2
