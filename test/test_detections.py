import cv2
import numpy as np

cv2.namedWindow("raw",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("filter",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("smooth",cv2.WINDOW_AUTOSIZE)
cv2.namedWindow("contour",cv2.WINDOW_AUTOSIZE)
scaleSize = 6

depthmaxx = 519
depthmaxy = 442
depthminx = 143
depthminy = 66

def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("depth Clicked point: (",x,',',y,") depth:",k.DepthFrameRaw[y,x])

class Block:
    def __init__(self, contour, stackHeight, angle, pixX, pixY, color=None):
        self.contour = contour
        self.stackHeight = stackHeight
        self.angle = angle
        self.color = color
        self.x = None
        self.y = None
        self.z = None
    def getCoords(self):
        """get self.x, y, and z from other info"""
        pass

class Kinect:
    def __init__(self):
        cv2.setMouseCallback("raw",mouse_callback)
        self.DepthFrameRaw = cv2.imread('raw_depth.png', cv2.IMREAD_UNCHANGED).astype(np.uint16) // pow(2,scaleSize)
        cv2.imshow('raw', self.DepthFrameRaw * pow(2,scaleSize))
        self.blocks = []
        self.workBound = np.array([[144,441], [142,66], [515,68], [518,441]])


    def clearSurroundings(self, frame):
        """!
        @brief     take a frame and make it black outside of the boundary defined by self.workBound
        @param     frame: the frame to use. This function edits the frame in place
        @return    the edited frame
        """
        stencil = np.zeros(frame.shape).astype(frame.dtype)
        cv2.fillPoly(stencil, [self.workBound], [255, 255, 255])
        frame[:] = cv2.bitwise_and(frame, stencil)
        return frame
        
        
    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """
        self.detectBlocksInDepthImage()
        # TODO: Find the color of each block
        # TODO if we have time: if the block is big, it's probably 2 blocks, use RGB to split them apart

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

        """
        board_depth_d = 853 # 725 for actual kinect image
        block_depth_d = 70  # 15 for actual kinect image
        block_pwidth = 26   # TODO: get this from actual block depth
        smoothing_size = 3  # number of pixels for smoothing (more = smoother)
        maxBlockHeight = 1  # maximum number of stacked blocks to detect
        kernel = np.ones((smoothing_size,smoothing_size),np.uint8) #kernel for smoothing

        self.blocks = []
        self.block_contours = []
        lastFilterFrame = np.zeros(self.DepthFrameRaw.shape, dtype=np.uint8) #initialize last frame to having nothing
        for i in range(maxBlockHeight,0,-1): #detect blocks stacked up to 5 high, starting from the top
            # remember, subtracting d makes it stacked higher
            d = board_depth_d - block_depth_d * (i - .5) #half a block's height less than the top
            self.DepthFrameFiltered = ((self.DepthFrameRaw < d) * 255).astype(np.uint8)
            self.clearSurroundings(self.DepthFrameFiltered)
            # now, subtract the last image, so that this only has the block stacks that end on this level
            self.DepthFrameFiltered -= lastFilterFrame
            cv2.imshow("filter", self.DepthFrameFiltered * pow(2,scaleSize+2))
            self.DepthFrameFiltered = cv2.morphologyEx(self.DepthFrameFiltered, cv2.MORPH_OPEN, kernel) #remove white dots
            self.DepthFrameFiltered = cv2.morphologyEx(self.DepthFrameFiltered, cv2.MORPH_CLOSE, kernel) #remove black dots
            cv2.imshow("smooth", self.DepthFrameFiltered * pow(2,scaleSize+2))
            # now self.DepthFrameFiltered should have squares for all the blocks at this height

            tmp = self.DepthFrameFiltered.copy()
            tmp, contours, tmph = cv2.findContours(tmp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                # print (contour.shape)
                rect = cv2.minAreaRect(contour)
                # if we think this is a valid block. TODO: make this better
                if (rect[1][0] > block_pwidth*.75 and rect[1][1] > block_pwidth*.75 and # if length and width are at least 3/4 of a block
                    np.all(contour > 0)): # if it doesn't go off the screen
                    self.block_contours.append(cv2.boxPoints(rect).astype(np.int32))
                    self.blocks.append(Block(contour, i, rect[2]*np.pi/180, rect[0][0], rect[0][1]))
        contourImage = self.DepthFrameRaw.copy()
        cv2.drawContours(contourImage, self.block_contours, -1, (255,0,255), 1)
        cv2.drawContours(contourImage, [self.workBound], -1, (255,0,255), 2)
        cv2.imshow("contour", contourImage * pow(2,scaleSize+2))

k = Kinect()
k.detectBlocksInDepthImage()
while True: # wait for escape before closing windows
	ch = 0xFF & cv2.waitKey(10)
	if ch == 0x1B:
    	    break
cv2.destroyAllWindows()
