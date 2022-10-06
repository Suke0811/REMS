import time

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
from scipy.spatial.transform import Rotation as R
from pathlib import Path


class ENV:
    LENGTH = 0.88   # [m]
    WIDTH =  1.15   # [m]

class ARUCO:
    PREFIX = 'image'
    FORMAT_JPG = 'jpg'
    FORMAT_PNG = 'png'
    CALIBRATION = 'calibration.yml'
    CALIB_WIDTH = 9
    CALIB_HEIGHT = 7

    REFINMENT_SIZE = 2

    CALIBRATION_PATH = Path(__file__).parent
    square_size = 0.022  #0.022m square size
    FRAME = 'Woodbot Tracker'

    TAG_TYPE_7 = aruco.Dictionary_get(aruco.DICT_5X5_1000)  # Use 7x7 dictionary to find markers
    TAG_SIZE = 0.05
    FRAME_INIT = [0,0,0, 0,0,0] #frame init, [x,y,z, i,j,k,w]


    CAMERA_RES = [int(480),int(640)]

    EULER = 'zyx'

    TAG_HOME_ID = 255
    TAG_ARENA_ID = [253,252,254,255]   # IDs of arena left down, right down, left top, right
    ARENA_CORNER = np.array([[0.0, 0.0], [ENV.WIDTH, 0.0], [0.0, ENV.LENGTH], [ENV.WIDTH, ENV.LENGTH]], np.float32) * 1000
    ARENA_ORIGIN_OFFSET =[-ENV.WIDTH/2,-ENV.LENGTH/2,0,0,0,0] #World origin to World tag
    CORNER_INDEX = [3,2,1,0]
    FPS = 10
    VIDEO_NAME = 'video.avi'


class ArucoHelper:
    def __init__(self, camera_id=0, calib_file_dir=ARUCO.CALIBRATION_PATH, tag_size=ARUCO.TAG_SIZE,fps=ARUCO.FPS, video_name=ARUCO.VIDEO_NAME):
        self.calib_file_dir = calib_file_dir.resolve()
        self.id_tack =[]
        self.pipe = None
        self.camera = None
        self.video_out = None
        self.fps = fps
        self.video_name = video_name
        self.tag_type = ARUCO.TAG_TYPE_7
        self.tag_size = tag_size
        self.camera_id = camera_id
        self.frames = {}   # tracking ids and respective frames
        self.arena_corner = np.array([[0.0, 0.0], [ENV.WIDTH, 0.0], [0.0, ENV.LENGTH], [ENV.WIDTH, ENV.LENGTH]], np.float32) * 1000
        #calibration setting
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.t = 0.0

    def get_camera(self):

        self.camera = cv2.VideoCapture(self.camera_id)
        assert self.camera.isOpened()
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, ARUCO.CAMERA_RES[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, ARUCO.CAMERA_RES[1])
        RES = ARUCO.CAMERA_RES

        # Video Feeds
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.video_out = cv2.VideoWriter(self.video_name, fourcc, self.fps, (RES[1], RES[0]))


    def get_image(self):
        if self.camera is None:
            frames = self.pipe.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                return

            # Convert images to numpy arrays
            image = np.asanyarray(color_frame.get_data())
        else:
            ret, image = self.camera.read()  # capture camera frame
            if not ret:
                return
        return image


    def init_camera(self):
        try:
            self.get_camera()
            self._load_coefficients()    # load calibration
        except ImportError:
            self.camera_matrix = None       # if no calibration file found
            self.distortion_matrix = None
        self.t = time.perf_counter()


    def track(self):
        """Track tag and get tag frame w.r.t. camera, and update display image """
        if self.camera_matrix is None:
            self._load_coefficients()   # making sure calibration info is available

        image = self.get_image()
        if image is None:
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Change grayscale
        parameters = aruco.DetectorParameters_create()  # Marker detection parameters
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        parameters.cornerRefinementWinSize = ARUCO.REFINMENT_SIZE
                # lists of detected markers
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, self.tag_type,
                                                                parameters=parameters,
                                                                cameraMatrix=self.camera_matrix,
                                                                distCoeff=self.distortion_matrix)
        if ids is not None:
            for i, id in enumerate(ids):
                if id in ARUCO.TAG_ARENA_ID:
                    index = ARUCO.TAG_ARENA_ID.index(id)
                    self.arena_corner[index] = corners[i][0][ARUCO.CORNER_INDEX[index]]
            image = self._corner_display(corners, ids, image)
            self._corner_2_frame(corners, ids)
        image = np.array(image, dtype=np.uint8)
        #if time.perf_counter() - self.t >= round(1/15):
        cv2.imshow(ARUCO.FRAME, image)
        #    self.t = time.perf_counter()

        self.video_out.write(image)
        cv2.waitKey(1)


    def _corner_display(self, corners, ids, image):
        if np.all(ids is not None):  # If there are markers found by detector
            for i, id in enumerate(ids.flatten().tolist()):  # Iterate in markers
                #pose estimation given 4 corners
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], self.tag_size, self.camera_matrix,
                                                                           self.distortion_matrix)
                (rvec - tvec).any()  # numpy value array error
                aruco.drawDetectedMarkers(image, corners)  # Draw A square around the markers
                aruco.drawAxis(image, self.camera_matrix, self.distortion_matrix, rvec, tvec, self.tag_size/2)  # Draw Axis
        return image
        # Display the resulting image


    def _corner_2_frame(self, corners, ids):

        h_trans, status = cv2.findHomography(self.arena_corner, ARUCO.ARENA_CORNER, cv2.RANSAC) #perspective matrix
        # this things uses mm regardless of the other part of units

        if np.all(ids is not None):  # If there are markers found by detector
            for i, id in enumerate(ids.flatten().tolist()):  # Iterate in markers
                #pose estimation given 4 corners
                c = cv2.perspectiveTransform(corners[i], h_trans)
                rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(c, self.tag_size, self.camera_matrix,
                                                                           self.distortion_matrix)
                (rvec - tvec).any()  # numpy value array error
                # update frame using rotation from pose estimation and xy from H_transed 4 corners
                self.frames[id] = np.concatenate((self.getMarkerCenter(c), self._rvec_2_euler(rvec.flatten())), axis=None)

    def getMarkerCenter(self, corners):
         px = (corners[0][0][0] + corners[0][1][0] + corners[0][2][0] + corners[0][3][0]) * 0.25
         py = (corners[0][0][1] + corners[0][1][1] + corners[0][2][1] + corners[0][3][1]) * 0.25
         return np.array([px,py,0])/1000


    def get_frames(self, track_ids=None):
        """Track multiple tags at a time, Returns list of frames in the world frame"""
        ret_frames = {}
        if track_ids is not None:
            self.track()
            for id in track_ids:
                if id in self.frames.keys() and ARUCO.TAG_ARENA_ID[0] in self.frames.keys():
                    ret_frames[str(id)] = (self.frames[id]) + ARUCO.ARENA_ORIGIN_OFFSET
                else:
                    self.frames[id] = (ARUCO.FRAME_INIT)
        return ret_frames

    @staticmethod
    def _rvec_2_euler(rvec):
        euler_rvec = R.from_rotvec(rvec).as_euler(ARUCO.EULER)
        return euler_rvec

    def run(self, track_ids):
        """run function is to test run the aruco tag detection standalone
        No need to use this in the simulation implementation"""
        while True:
            fs = self.get_frames(track_ids)
            print(fs)
            # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
            key = cv2.waitKey(int(1000/self.fps)) & 0xFF
            if key == ord('q'):  # Quit
                break
        self.close()


    def calibrate(self, square_size, image_format=ARUCO.FORMAT_JPG, prefix=ARUCO.PREFIX, width=ARUCO.CALIB_WIDTH, height=ARUCO.CALIB_HEIGHT):
        """ Apply camera calibration operation for images in the given directory path.
         Credit: aliyasineser"""
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((height*width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

        objp = objp * square_size

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.


        images = glob.glob(str(self.calib_file_dir.absolute()) + '\\image\\' + prefix + ' (*).' + image_format)

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)
                #cv2.imshow(ARUCO.FRAME, img)


        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        self._save_coefficients(mtx, dist)


    def _save_coefficients(self, mtx, dist):
        """ Save the camera matrix and the distortion coefficients to given path/file.
        Credit: aliyasineser"""
        cv_file = cv2.FileStorage(str(self.calib_file_dir.resolve() / ARUCO.CALIBRATION), cv2.FILE_STORAGE_WRITE)
        cv_file.write("Camera", mtx)
        cv_file.write("Distortion", dist)
        # note you *release* you don't close() a FileStorage object
        cv_file.release()

    def _load_coefficients(self):
        """ Loads camera matrix and distortion coefficients.
        Credit: aliyasineser"""
        # FILE_STORAGE_READ
        cv_file = cv2.FileStorage(str(self.calib_file_dir.absolute() / ARUCO.CALIBRATION), cv2.FILE_STORAGE_READ)

        # note we also have to specify the type to retrieve other wise we only get a
        # FileNode object back instead of a matrix
        camera_matrix = cv_file.getNode("Camera").mat()
        dist_matrix = cv_file.getNode("Distortion").mat()

        cv_file.release()
        self.camera_matrix = camera_matrix
        self.distortion_matrix = dist_matrix

    def __del__(self):
        self.close()

    def close(self):
        if self.camera is not None:
            self.camera.release()
        if self.pipe is not None:
            self.pipe.stop()
        if self.video_out is not None:
            self.video_out.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    # camera id could be 1 if there is another one
    a = ArucoHelper(camera_id=0)
    # init_camera tries to read calibration data
    a.init_camera()
    #a.calibrate(0.02)
    a.run([3,2,1])
