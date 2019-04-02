# encoding: UTF-8
import vision_definitions
from naoqi import ALProxy
import cv2
import numpy as np
from PIL import Image
import time
import math
import os
import motion
import thread
import sys
import almath
PORT = 9559
robotIP = "192.168.1.109"
alpha = -math.pi/2
maxstepx = 0.04
maxstepy = 0.14
maxsteptheta = 0.3
maxstepfrequency = 0.2
stepheight = 0.02
torsowx = 0.0
torsowy = 0.0
def zhuagan():
    while True:
        headTouchedmidlleFlag = memoryProxy.getData("MiddleTactilTouched")
        if headTouchedmidlleFlag == 1.0:
            print "right hand touched"
            tts.say("请给我一个球杆，好吗")
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4)
            time.sleep(2)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.1)
            time.sleep(2)
            break
def shougan():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")  # Y轴动（许诺）
    times.append([1, 2, 3, 4])  # 是一个关节对应四个时间，坐标？在第几秒钟转到对应坐标？怎么做到的？（许诺）
    keys.append([0, 0, 0, 0])

    names.append("HeadYaw")  # Z轴动（许诺）          重复的为什么不用一个值代替
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LAnklePitch")  # 脚踝Z轴
    times.append([1, 2, 3, 4])
    keys.append([-0.349794, -0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")  # 脚踝X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LElbowRoll")  # 肘Z轴
    times.append([1, 2, 3, 4])
    keys.append([-0.321141, -0.321141, -0.965791, -0.965791])

    names.append("LElbowYaw")  # X轴
    times.append([1, 2, 3, 4])
    keys.append([-1.37757, -1.37757, -1.466076, -1.466076])

    names.append("LHand")  # 左掌
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.6000, 0.6000, 0.6000, 0.6000, 0.1800])

    names.append("LHipPitch")  # 腿Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.450955, -0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")  # 腿X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LHipYawPitch")  # 啥关节
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LKneePitch")  # 膝盖Y轴
    times.append([1, 2, 3, 4])
    keys.append([0.699462, 0.699462, 0.699462, 0.699462])

    names.append("LShoulderPitch")  # 肩Y轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([1.53885, 1.43885, 1.03856, 1.03856, 1.03856])

    names.append("LShoulderRoll")  # 肩Z轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.268407, 0.268407, -0.04014, -0.04014, -0.04014])

    names.append("LWristYaw")  # 手腕X轴
    times.append([1, 2, 3, 4])
    keys.append([-0.016916, -0.016916, -1.632374, -1.632374])

    names.append("RAnklePitch")  # 脚踝Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.354312, -0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")  # 脚踝X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RElbowRoll")  # 肘Z轴
    times.append([1, 2, 3, 4])
    keys.append([0.958791, 0.958791, 0.958791, 0.958791])

    names.append("RElbowYaw")  # 肘X轴
    times.append([1, 2, 3, 4])
    keys.append([1.466076, 1.466076, 1.466076, 1.466076])

    names.append("RHand")
    times.append([1, 2, 3, 4])
    keys.append([0.0900, 0.0900, 0.0900, 0.0900])

    names.append("RHipPitch")  # 腿Y轴
    times.append([1, 2, 3, 4])
    keys.append([-0.451038, -0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")  # 腿X轴
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RKneePitch")  # 膝盖Y轴
    times.append([1, 2, 3, 4])
    keys.append([0.699545, 0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")  # 肩Y轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([1.03856, 1.03856, 1.03856, 1.03856, 1.03856])

    names.append("RShoulderRoll")  # 肩Z轴
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.04014, 0.04014, 0.04014, 0.04014, 0.04014])

    names.append("RWristYaw")  # 腕X轴
    times.append([1, 2, 3, 4])
    keys.append([1.632374, 1.632374, 1.632374, 1.632374])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)
def rebball_coordinate_mark(allballdata, HeadPitch=0.0, n=2):
    print '###身体开始定位红球与mark###'
    motionPrx.angleInterpolationWithSpeed("HeadPitch", HeadPitch, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    headangle_x = allballdata[0][0]
    headangle_y = allballdata[1][0]
    center_x = allballdata[2][1][0]
    center_y = allballdata[2][1][1]
    # 摄像头在机器人坐标系中的横坐标
    redball_camera_x = allballdata[2][3][0]
    # 摄像头在机器人坐标系中的纵坐标
    redball_camera_y = allballdata[2][3][1]
    print '初始center_x = ', center_x

    # 机器人行走参数
    h = 0.45469
    i = 0
    while (abs(center_x) > 0.02) and i < n:
        motionPrx.moveTo(0, 0, headangle_x + center_x,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        time.sleep(1)
        ballData = memoryProxy.getData("redBallDetected")
        center_x = ballData[1][0]
        center_y = ballData[1][1]
        # 头部水平偏转角
        headangle_x = motionPrx.getAngles("HeadYaw", True)
        headangle_x = headangle_x[0]
        # 头部竖直偏转角
        headangle_y = motionPrx.getAngles("HeadPitch", True)
        headangle_y = headangle_y[0]
        # 摄像头在机器人坐标系中的横坐标
        redball_camera_x = ballData[3][0]
        # 摄像头在机器人坐标系中的纵坐标
        redball_camera_y = ballData[3][1]
        i += 1
    # 与红球的距离
    redball_dis = (h - 0.021) / (math.tan(headangle_y + center_y + (39.7 * math.pi / 180.0)))

    redball_data = [redball_dis, center_x, center_y, redball_camera_x, redball_camera_y]

    print 'center_x = ', center_x
    print 'redball_dis = ', redball_dis

    # 开始定位马克
    time.sleep(1)
    tts.say("开始定位马克")
    # mark半径
    r = 0.10
    headYawAngle = -2.09
    camProxy.setActiveCamera(0)
    # use top camera
    currentCamera = "CameraTop"
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    landmarkProxy.subscribe("landmarkTest")
    while (headYawAngle < 2.09):
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.3)
        memoryProxy.insertData("LandmarkDetected", [])
        time.sleep(2)

        markData = memoryProxy.getData("LandmarkDetected")

        if markData != []:
            tts.say("找到马克了!")
            # landmark识别符0代表识别到，1代表未识别到。
            landmarkFlag = 0

            # alph
            mark_alph = markData[1][0][0][1]
            # beta
            mark_beta = markData[1][0][0][2]
            # SizeX
            mark_sizex = markData[1][0][0][3]
            # 水平偏转角
            mark_headangle0 = motionPrx.getAngles("HeadYaw", True)
            mark_headangle_x = mark_headangle0[0]
            # 竖直偏转角
            mark_headangle1 = motionPrx.getAngles("HeadPitch", True)
            mark_headangle_y = mark_headangle1[0]
            # 摄像头在机器人坐标系中的横坐标
            mark_camera_x = markData[3][0]
            # 摄像头在机器人坐标系中的纵坐标
            mark_camera_y = markData[3][1]
            mark_dis = r / (2 * math.tan(mark_sizex / 2))

            # 补偿
            mark_dis = mark_dis * math.cos(mark_beta)
            mark_dis = mark_dis - 0.06

            while (mark_alph > 0.03):
                motionPrx.angleInterpolationWithSpeed("HeadYaw", mark_headangle_x + mark_alph, 0.1)
                time.sleep(1)
                memoryProxy.insertData("LandmarkDetected", [])

                markData = memoryProxy.getData("LandmarkDetected")

                if markData != []:
                    # print 'markData = ', markData
                    # alph ****list index out of range
                    mark_alph = markData[1][0][0][1]
                    # beta
                    mark_beta = markData[1][0][0][2]
                    # SizeX
                    mark_sizex = markData[1][0][0][3]
                    # 水平偏转角
                    mark_headangle0 = motionPrx.getAngles("HeadYaw", True)
                    mark_headangle_x = mark_headangle0[0]
                    # 竖直偏转角
                    mark_headangle1 = motionPrx.getAngles("HeadPitch", True)
                    mark_headangle_y = mark_headangle1[0]
                    # 摄像头在机器人坐标系中的横坐标
                    mark_camera_x = markData[3][0]
                    # 摄像头在机器人坐标系中的纵坐标
                    mark_camera_y = markData[3][1]
                    # 与mark的距离
                    mark_dis = r / (2 * math.tan(mark_sizex / 2))  # 相机到mark的距离

                    # 补偿
                    mark_dis = mark_dis * math.cos(mark_beta)
                    mark_dis = mark_dis - 0.06

                    tts.say('马克定位完成')

            print 'mark_alph = ', mark_alph
            print 'mark_dis = ', mark_dis

            # 头部水平偏转角
            markheadangle = mark_headangle_x + mark_alph

            # 所有mark的数据
            mark_data = [mark_dis, mark_alph, mark_beta, mark_sizex, markheadangle, mark_camera_x, mark_camera_y]

            alldata = [redball_data, mark_data, landmarkFlag]

            return alldata


        else:
            # tts.say("马克去哪了?")
            headYawAngle = headYawAngle + 0.5

def searchredball(HeadPitch=0.0):
    print '###开始定位红球###'
    # 初始化相机
    camProxy.setActiveCamera(1)
    redballProxy.subscribe("redBallDetected")
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", HeadPitch, 0.3)
    memoryProxy.insertData("redBallDetected", [])
    time.sleep(1)
    ballDatatest = memoryProxy.getData("redBallDetected")

    if ballDatatest == []:

        headYawAngle = -(90 * math.pi / 180)
        while (headYawAngle < (90 * math.pi / 180)):
            memoryProxy.insertData("redBallDetected", [])
            motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.3)
            time.sleep(2)
            for i in range(3):
                ballDatatest = memoryProxy.getData("redBallDetected")
            if ballDatatest != []:
                tts.say('找到红球了')
                # centerX
                wzCamera = ballDatatest[1][0]
                # centerY
                wyCamera = ballDatatest[1][1]
                # SizeX
                angularSize = ballDatatest[1][2]
                # 头部水平偏转角
                headanglex = motionPrx.getAngles("HeadYaw", True)
                # 头部竖直偏转角
                headangley = motionPrx.getAngles("HeadPitch", True)
                redballFlag = 0
                allballData = [headanglex, headangley, ballDatatest, redballFlag]
                return allballData

            else:
                headYawAngle += (30 * math.pi / 180)

        tts.say("没有找到红球，开始下一次寻找!")
        redballFlag = 1
        motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
        motionPrx.setMoveArmsEnabled(False, False)
        # 找不到球就前进0.4米
        motionPrx.moveTo(0.4, 0.0, 0.0,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        allballDatatest = [0, 0, 0, redballFlag]
        return allballDatatest
    else:
        # centerX
        wzCamera = ballDatatest[1][0]
        # centerY
        wyCamera = ballDatatest[1][1]
        # SizeX
        angularSize = ballDatatest[1][2]
        # 头部水平偏转角
        headanglex = motionPrx.getAngles("HeadYaw", True)
        # 头部竖直偏转角
        headangley = motionPrx.getAngles("HeadPitch", True)
        tts.say("找到红球了!")
        redballFlag = 0
        allballData = [headanglex, headangley, ballDatatest, redballFlag]
        return allballData
# -------------------------- 构建机器人坐标系中的三角形并计算如何才能三点一线 -------------------------------#
def coordinate_system(alldata, x=0.0, y=0.0):
    print '###建立坐标系###'
    '''
    alldata = [redball_data, mark_data, landmarkFlag]

    redball_data = [redball_dis, center_x, center_y, redball_camera_x, redball_camera_y]

     mark_data = [mark_dis, mark_alph, mark_beta, mark_sizex, markheadangle, mark_camera_x, mark_camera_y]
    '''
    redball_data = alldata[0]
    mark_data = alldata[1]

    redball_dis = redball_data[0]
    redball_camera_x = redball_data[3]
    redball_camera_y = redball_data[4]

    mark_dis = mark_data[0]
    mark_camera_x = mark_data[5]
    mark_camera_y = mark_data[6]
    mark_headangle = mark_data[4]

    if mark_headangle < 0:
        if abs(mark_headangle) < math.pi / 2:
            M_y = -(mark_dis * math.sin(abs(mark_headangle))) + mark_camera_y
            M_x = mark_dis * math.cos(mark_headangle) + mark_camera_x

            B_y = 0 + redball_camera_y
            B_x = redball_dis + redball_camera_x
        else:
            M_y = -(mark_dis * math.cos(abs(mark_headangle) - math.pi / 2)) + mark_camera_y
            M_x = -(mark_dis * math.sin(abs(mark_headangle) - math.pi / 2)) + mark_camera_x

            B_y = 0 + redball_camera_y
            B_x = redball_dis + redball_camera_x
    else:
        if abs(mark_headangle) < math.pi / 2:
            M_y = (mark_dis * math.sin(abs(mark_headangle))) + mark_camera_y
            M_x = mark_dis * math.cos(mark_headangle) + mark_camera_x

            B_y = 0 + redball_camera_y
            B_x = redball_dis + redball_camera_x
        else:
            M_y = (mark_dis * math.cos(abs(mark_headangle) - math.pi / 2)) + mark_camera_y
            M_x = -(mark_dis * math.sin(abs(mark_headangle) - math.pi / 2)) + mark_camera_x

            B_y = 0 + redball_camera_y
            B_x = redball_dis + redball_camera_x

    BM = math.sqrt(pow((M_y - B_y), 2) + pow((M_x - B_x), 2))+0.09
    BO = math.sqrt(pow((B_y - 0), 2) + pow((B_x - 0), 2))
    MO = math.sqrt(pow((M_y - 0), 2) + pow((M_x - 0), 2))+0.09
    print 'ball_to_robot = ', BO
    print 'ball_to_mark = ', BM
    print 'mark_to_robot = ', MO
    print 'mark_headangle =', mark_headangle
    # 用正弦定理会出现问题，因为在[0,pi]内，sin(i) = sin(pi-i)，故用余弦定理
    # game = math.asin((MO / BM) * math.sin(abs(mark_headangle)))
    game = math.acos((pow(BM, 2) + pow(BO, 2) - pow(MO, 2)) / 2 * BM * BO)

    if (game < math.pi / 2):
        dis_vertical = BO * math.sin(game) - x
        dis_level = BO * math.cos(game) - y
        theta = (math.pi / 2 - game)
    else:
        dis_vertical = BO * math.cos(game - math.pi / 2) - x
        dis_level = (BO * math.sin(game - math.pi / 2) - y)
        theta = (game - math.pi / 2)

    coordinate = [theta, dis_vertical, dis_level, B_x, B_y, M_x, M_y, mark_headangle, game]
    print 'y = ', dis_level
    print 'x = ', dis_vertical

    return coordinate


# ---------------------------------- 走路：三点一线 ------------------------------------------------#
def walk_dis(coordinate, xflag=1, turn=1, yflag=1):
    print '###开始行走###'
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0, 0.1)
    maxstepx = 0.02
    maxstepy = 0.14
    maxsteptheta = 0.1
    maxstepfrequency = 0.2
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    mark_headangle = coordinate[7]
    game = coordinate[8]

    # 补偿
    theta = coordinate[0]
    x = coordinate[1]
    y = coordinate[2]

    if mark_headangle < 0:
        # 4种情况相应补偿
        if game < math.pi / 2:
            theta = -(theta+0.05)
            x = x
            y = y + 0.05
        else:
            theta = theta+0.05
            x = x
            y = -(y+0.05)
    else:
        if game < math.pi / 2:
            theta = theta+0.05
            x = x
            y = -(y+0.05)
        else:
            theta = -(theta+0.05)
            x = x
            y = y+0.05
    print '计算旋转角度 = ', theta * 180 / math.pi

    motionPrx.setMoveArmsEnabled(False, False)
    if turn == 1:
        motionPrx.moveTo(0, 0, 2.0 * theta,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        time.sleep(1)

    if yflag == 1:
        motionPrx.moveTo(0, y, 0,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        time.sleep(1)

    if xflag == 1:
        motionPrx.moveTo(x, 0, 0,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
        time.sleep(1)
def hitBall():
    if (alpha < 0):

        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.15)
        time.sleep(1)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR32, 0.15)
        time.sleep(1)
        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR34, 0.1)

        motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR33, 0.612)
def landmarkdetect():

    resolution = vision_definitions.kVGA #640*480
    colorSpace = vision_definitions.kRGBColorSpace #BGR
    fps = 10
    video = camProxy.subscribe("python_client",resolution,colorSpace,fps)
    naoImage = camProxy.getImageRemote(video)
    time.sleep(0.05)
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)
    filename = 'robot0.png'
    im.save(filename, "PNG")
    camProxy.unsubscribe(video)
    return im

def opencv(img):

    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    lower_hsv = np.array([0, 160, 87])
    upper_hsv = np.array([124, 255, 255])
    mask = cv2.inRange(hsv, lowerb=lower_hsv, upperb=upper_hsv)
    dst = cv2.bitwise_and(img, img, mask=mask)
    imgG = cv2.GaussianBlur(dst, (3, 3), 0)
    gray = cv2.cvtColor(imgG, cv2.COLOR_HSV2BGR)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    ret, img2 = cv2.threshold(gray, 127, 255, cv2.THRESH_OTSU)
    kernelx = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    opened = cv2.morphologyEx(img2, cv2.MORPH_OPEN, kernelx)
    imgs, contours, hier = cv2.findContours(opened, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for i, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        x,y,w,h= cv2.boundingRect(c)
        rate =  rect[1][1]/rect[1][0]
        print(rate)
        print(x,y,w,h)
        if  h>50 and rate>7.5 or 1/rate>7.5:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            sx = -(x + w/2)+320
            print(sx)
            sy = (y + h)-240
            print(sy)
            thetaw = 60.97*math.pi/180
            thetah = 47.64*math.pi/180
            rw = math.sqrt(math.pow(640, 2) / ((1 - math.cos(thetaw)) * 2))
            anglex = (math.pi - thetaw) / 2
            oa = rw * math.sin(anglex)
            if sx>0:
                ob = math.sqrt(math.pow(rw, 2) + math.pow(320-sx, 2) - 2 * rw * (320 - sx) * math.cos(anglex))
                theta = (math.pow(oa, 2) + math.pow(ob, 2) - math.pow(sx, 2)) / (2 * oa * ob)
                if theta > 1:
                    theta = 1
            else:
                sx = math.fabs(sx)+320
                print('****---------*****')
                ob = math.sqrt(math.pow(rw, 2) + math.pow(sx, 2) - 2 * rw * sx * math.cos(59.5 * math.pi / 180))
                theta = (math.pow(oa, 2) + math.pow(ob, 2) - math.pow(sx-320, 2)) / (2 * oa * ob)
            thetax = math.acos(theta)
            print(ob,oa,theta,thetax,h)
            wy = y * math.tan(thetax)
            sizex = (w*60.97/640)*math.pi/180
            sizey = (h*47.64 /480)*math.pi/180
            stickheight = 47.5
            distance = stickheight/ (math.tan(sizey))
            th = (sx*60.97/640)*math.pi/180
            stick = [thetax,sizex,sizey,distance,th]
    print(stick)
    cv2.imshow('are', img)
    return stick
def headtouch():
    while True:
        headTouchedButtonFlag = memoryProxy.getData("FrontTactilTouched")
        if headTouchedButtonFlag == 1.0:
            print "front head touched"
            # tts.say("开始击球")
            break
def searchNAOmark():
    headYawAngle = -2.09
    camProxy.setActiveCamera(0)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.3)
    while (headYawAngle < 2.09):
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headYawAngle, 0.1)
        time.sleep(1)
        im = landmarkdetect()
        src = cv2.imread('robot0.png', 1)

        markData = opencv(src)
        if (markData and isinstance(markData, list) and len(markData) >= 2):
            tts.say("i saw landmark!")
            landmarkFlag = 0  # landmark识别符0代表识别到，1代表未识别到。
            markCamera = markData[4]
            headangle = motionPrx.getAngles("HeadYaw", True)
            markheadangle = markCamera + headangle[0]
            angle = markCamera+markheadangle+math.pi/2
            print(markheadangle)
            allmarkdata = [markCamera, markheadangle,angle, landmarkFlag]
            motionPrx.moveTo(0.0, 0.0, angle,
                             [["MaxStepX", maxstepx],
                              ["MaxStepY", maxstepy],
                              ["MaxStepTheta", maxsteptheta],
                              ["MaxStepFrequency", maxstepfrequency],
                              ["StepHeight", stepheight],
                              ["TorsoWx", torsowx],
                              ["TorsoWy", torsowy]])
            print allmarkdata
            return allmarkdata  # 因为已经返回值了，所以不会执行while循环之后的程序
        else:
            tts.say("马克去哪了?")
        headYawAngle = headYawAngle + 0.5
    tts.say("抱歉我找不到马克，我将直接击球 ")  # 后面可以加击球程序
    landmarkFlag = 1  # landmark识别符0代表识别到，1代表未识别到。
    allmarkdata = [0, 0, 0, landmarkFlag]
    landmarkProxy.unsubscribe("landmarkTest")
    return allmarkdata


PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
PositionJointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
golfPositionJointAnglesR32 = [1.35787, 0.05760, 1.50098, 1.50971, 0.698, 0.2]
golfPositionJointAnglesR33 = [1.35787, -0.5396, 1.50098, 1.50971, -0.34, 0.2]
golfPositionJointAnglesR31 = [0.9546, 0.05760, 1.50098, 1.50971, 0.0, 0.2]
golfPositionJointAnglesR34 = [1.35787, 0.05760, 1.50098, 1.50971, 0.34, 0.2]

golfPositionJointAnglesR6 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.2]


golfPositionJointAnglesR5 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.6]




memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
tts = ALProxy("ALTextToSpeech", robotIP, PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
Lifestop = ALProxy("ALAutonomousLife", robotIP, PORT)
Lifestop.setState("disabled")

camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
landmarkProxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
postureProxy.goToPosture("StandInit", 3.0)
# motionPrx.moveTo(0, 0.0, 0,
#                              [["MaxStepX", maxstepx],
#                               ["MaxStepY", maxstepy],
#                               ["MaxStepTheta", maxsteptheta],
#                               ["MaxStepFrequency", maxstepfrequency],
#                               ["StepHeight", stepheight],
#                               ["TorsoWx", torsowx],
#                               ["TorsoWy", torsowy]])
zhuagan()

headtouch()
i = 0
while i < 3:
    headtouch()
    hitBall()
    i+=1
# markdata = searchNAOmark()
# shougan()
# headtouch()
motionPrx.openHand('LHand')
motionPrx.openHand('RHand')
cv2.waitKey(0)
cv2.destroyAllWindows()
