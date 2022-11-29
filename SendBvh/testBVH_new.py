from bvh import Bvh, BvhNode
import numpy as np
import math
import vtk
import cv2
from ezc3d import c3d
#import pandas as pd
from math import cos, sin, pi
import socket
import time


def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def eulerAnglesToRotationMatrix(theta, format='degree', fileType='bvh'):
    if fileType is 'bvh':
        tmp = theta.copy()  # 原始bvh数据是按照ZXY顺序存储欧拉角的
        theta = [tmp[1], tmp[2], tmp[0]]  # 改为XYZ顺序存储

    if format is 'degree':
        theta = [i * math.pi / 180.0 for i in theta]

    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    if fileType is 'bvh':
        # R = np.dot(R_y, np.dot(R_x, R_z))  # bvh规定的旋转顺序是zxy
        R = np.dot(R_z, np.dot(R_x, R_y))  # bvh规定的旋转顺序是zxy
    else:
        R = np.dot(R_z, np.dot(R_y, R_x))

    return R


def cal_H(r, t):
    r = np.array(r)
    R = np.eye(3)
    t = np.array(t)
    cv2.Rodrigues(r, R)  # 计算旋转矩阵
    H = np.eye(4)
    H[0:3, 0:3] = R
    H[0:3, 3] = t
    return H


def Right2Left(axis, Rr, Tr):  # 将右手系的RT转换为左手系的RT，axis为选择反向的轴
    Rl = Rr.copy()
    Tl = Tr.copy()

    if axis == "X" or axis == "x":
        C = np.array([[1, -1, -1], [-1, 1, 1], [-1, 1, 1]])
        Rl = np.multiply(Rl, C)  # 按元素相乘
        Tl[0] = -Tl[0]

    if axis == "Y" or axis == "y":
        C = np.array([[1, -1, 1], [-1, 1, -1], [1, -1, 1]])
        Rl = np.multiply(Rl, C)  # 按元素相乘
        Tl[1] = -Tl[1]

    if axis == "Z" or axis == "z":
        C = np.array([[1, 1, -1], [1, 1, -1], [-1, -1, 1]])
        Rl = np.multiply(Rl, C)  # 按元素相乘
        Tl[2] = -Tl[2]

    return Rl, Tl  # 左手系的RT

def Quat2Rot(q):#四元数转旋转矩阵,输入顺序为[w,x,y,z]
    R=np.zeros((3,3),dtype='float')
    R[0,0]=1-2*q[0,2]*q[0,2]-2*q[0,3]*q[0,3]
    R[0,1]=2*q[0,1]*q[0,2]+2*q[0,0]*q[0,3]
    R[0,2]=2*q[0,1]*q[0,3]-2*q[0,0]*q[0,2]
    
    R[1,0]=2*q[0,1]*q[0,2]-2*q[0,0]*q[0,3]
    R[1,1]=1-2*q[0,1]*q[0,1]-2*q[0,3]*q[0,3]
    R[1,2]=2*q[0,2]*q[0,3]+2*q[0,0]*q[0,1]
    
    R[2,0]=2*q[0,1]*q[0,3]+2*q[0,0]*q[0,2]
    R[2,1]=2*q[0,2]*q[0,3]-2*q[0,0]*q[0,1]
    R[2,2]=1-2*q[0,1]*q[0,1]-2*q[0,2]*q[0,2]
    
    return R
    
def Rot2Quat(R):  # 旋转矩阵转四元数
    m11 = R[0, 0]
    m12 = R[0, 1]
    m13 = R[0, 2]
    m21 = R[1, 0]
    m22 = R[1, 1]
    m23 = R[1, 2]
    m31 = R[2, 0]
    m32 = R[2, 1]
    m33 = R[2, 2]

    # 探测w, x, y, z中的最大值
    fourWSquaredMinus1 = m11 + m22 + m33
    fourXSquaredMinus1 = m11 - m22 - m33
    fourYSquaredMinus1 = m22 - m11 - m33
    fourZSquaredMinus1 = m33 - m11 - m22

    fourBiggestSquaredMinus1 = fourWSquaredMinus1
    biggestIndex = 0
    if fourXSquaredMinus1 > fourBiggestSquaredMinus1:
        fourBiggestSquaredMinus1 = fourXSquaredMinus1
        biggestIndex = 1

    if fourYSquaredMinus1 > fourBiggestSquaredMinus1:
        fourBiggestSquaredMinus1 = fourYSquaredMinus1
        biggestIndex = 2

    if fourZSquaredMinus1 > fourBiggestSquaredMinus1:
        fourBiggestSquaredMinus1 = fourZSquaredMinus1
        biggestIndex = 3

    # 计算平方根和除法
    biggestVal = math.sqrt(fourBiggestSquaredMinus1 + 1.0) * 0.5
    mult = 0.25 / biggestVal
    w, x, y, z = 0, 0, 0, 0
    if biggestIndex == 0:
        w = biggestVal
        x = (m23 - m32) * mult
        y = (m31 - m13) * mult
        z = (m12 - m21) * mult
    elif biggestIndex == 1:
        x = biggestVal
        w = (m23 - m32) * mult
        y = (m12 + m21) * mult
        z = (m31 + m13) * mult
    elif biggestIndex == 2:
        y = biggestVal
        w = (m31 - m13) * mult
        x = (m12 + m21) * mult
        z = (m23 + m32) * mult
    elif biggestIndex == 3:
        z = biggestVal
        w = (m12 - m21) * mult
        x = (m31 + m13) * mult
        y = (m23 + m32) * mult

    q = np.array([w, x, y, z])
    return q


class BVHDrawer():
    def __init__(self, window_name='bvh', window_size=[800, 600]):
        self.renderer = vtk.vtkRenderer()
        self.renderWindow = vtk.vtkRenderWindow()
        self.renderWindow.SetSize(window_size[0], window_size[1])
        self.window_name = window_name
        # self.renderWindow.SetWindowName(window_name)  # 在这里不起作用
        self.renderWindow.AddRenderer(self.renderer)
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)
        H = np.eye(4)
        self.axes(H, scale=500)  # 世界坐标系
        self.picker = vtk.vtkCellPicker()
        self.dataloader = None
        self.frame_index = 0

    def pickerfunc(self, object, event):
        # style = clientdata;
        # pickPos = picker.GetPickPosition()
        # print event

        if event == 'LeftButtonPressEvent':
            pass
        if event == 'LeftButtonReleaseEvent':
            pass
            # self.updateOneFrame()

    def axes(self, scale=100):
        self.line([0, 0, 0], [scale, 0, 0], np.array([255, 0, 0]) / 255.0)
        self.line([0, 0, 0], [0, scale, 0], np.array([0, 255, 0]) / 255.0)
        self.line([0, 0, 0], [0, 0, scale], np.array([0, 0, 255]) / 255.0)

    def axes(self, H, scale=100):
        # Xw=R*X0+T
        axesActor = vtk.vtkAxesActor()
        axesActor.AxisLabelsOff()
        # axesActor.SetPosition(10000, 0, 0)
        # axesActor.SetOrientation([90, 90, 90])
        Transform = vtk.vtkTransform()
        # Transform.RotateWXYZ()
        # Transform.Translate(Trans)
        Matrix4x4 = vtk.vtkMatrix4x4()
        for i in range(4):
            for j in range(4):
                Matrix4x4.SetElement(i, j, H[i, j])

        Transform.SetMatrix(Matrix4x4)
        axesActor.SetUserTransform(Transform)
        axesActor.SetTotalLength(scale, scale, scale)
        # axesActor.SetCylinderRadius(0.02);
        self.renderer.AddActor(axesActor)

    def line(self, start, end, color=np.array([54, 175, 201]) / 255.0):
        # Create a vtkPoints object and store the points in it
        points = vtk.vtkPoints()
        points.InsertNextPoint(np.array(start))
        points.InsertNextPoint(np.array(end))
        # Create a polydata to store everything in
        linesPolyData = vtk.vtkPolyData()
        # Add the points to the dataset
        linesPolyData.SetPoints(points)
        # Create a cell array to store the lines in and add the lines to it
        line = vtk.vtkLine()
        line.GetPointIds().SetId(0, 0)
        line.GetPointIds().SetId(1, 1)
        lines = vtk.vtkCellArray()
        lines.InsertNextCell(line)
        # Add the lines to the dataset
        linesPolyData.SetLines(lines)

        # Setup actor and mapper
        # colors = vtk.vtkNamedColors()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(4)
        # actor.GetProperty().SetColor(colors.GetColor3d("Peacock"))
        actor.GetProperty().SetColor(color)
        self.renderer.AddActor(actor)

    def polyLines(self, pts, color=np.array([54, 175, 201]) / 255.0):
        dim = pts.ndim  # 数据的维度
        if dim == 1:
            num = int(max(pts.shape) / 3)  # 确定有几个点要绘制
            ptdata = pts.reshape(num, 3)  # 转换成num行3列的矩阵
        else:
            num = pts.shape[0]
            ptdata = pts
        # Create a vtkPoints object and store the points in it
        points = vtk.vtkPoints()
        for i in range(num):
            points.InsertNextPoint(ptdata[i, :])

        # Create a polydata to store everything in
        linesPolyData = vtk.vtkPolyData()
        # Add the points to the dataset
        linesPolyData.SetPoints(points)

        lines = vtk.vtkCellArray()
        for i in range(num - 1):
            # Create a cell array to store the lines in and add the lines to it
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, i)
            line.GetPointIds().SetId(1, i + 1)  # 线的起点和终点
            lines.InsertNextCell(line)

        # Add the lines to the dataset
        linesPolyData.SetLines(lines)
        # Setup actor and mapper
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(linesPolyData)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetLineWidth(4)
        # colors = vtk.vtkNamedColors()
        # actor.GetProperty().SetColor(colors.GetColor3d("Peacock"))
        actor.GetProperty().SetColor(color)
        self.renderer.AddActor(actor)

    def sphere(self, pos, radius, color=np.array([255, 0, 0]) / 255.0):
        pt = np.array(pos)
        # Create a sphere
        sphereSource = vtk.vtkSphereSource()
        sphereSource.SetCenter(pt[0], pt[1], pt[2])
        sphereSource.SetRadius(radius)
        # Make the surface smooth.
        sphereSource.SetPhiResolution(50)
        sphereSource.SetThetaResolution(50)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphereSource.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        # actor.GetProperty().SetColor(colors.GetColor3d("Red"))
        actor.GetProperty().SetColor(color)
        self.renderer.AddActor(actor)

    def multi_sphere(self, pos, radius, color=np.array([255, 0, 0]) / 255.0):
        dim = pos.ndim  # 数据的维度
        if dim == 1:
            num = int(max(pos.shape) / 3)  # 确定有几个点要绘制
            ptdata = pos.reshape(num, 3)  # 转换成num行3列的矩阵
        else:
            num = pos.shape[0]
            ptdata = pos

        # colors = vtk.vtkNamedColors()
        for i in range(num):
            pt = ptdata[i, :]
            # Create a sphere
            sphereSource = vtk.vtkSphereSource()
            sphereSource.SetCenter(pt[0], pt[1], pt[2])
            sphereSource.SetRadius(radius)
            # Make the surface smooth.
            sphereSource.SetPhiResolution(50)
            sphereSource.SetThetaResolution(50)

            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(sphereSource.GetOutputPort())

            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            # actor.GetProperty().SetColor(colors.GetColor3d("Red"))
            actor.GetProperty().SetColor(color)
            self.renderer.AddActor(actor)

    def bodyLine(self, pos, color=np.array([54, 175, 201]) / 255.0):
        # 绘制人体姿态连线
        dim = pos.ndim  # 数据的维度
        if dim == 1:
            num = int(max(pos.shape) / 3)  # 确定有几个点要绘制
            ptdata = pos.reshape(num, 3)  # 转换成num行3列的矩阵
        else:
            num = pos.shape[0]
            ptdata = pos

        if num != 31:
            print("绘制点的数量不符合要求！")
            return

        # 0~5号点是左腿
        self.polyLines(ptdata[0:6, :], color)
        # 0-6~10号点是右腿
        self.line(ptdata[0, :], ptdata[6, :], color)
        self.polyLines(ptdata[6:11, :], color)
        # 0-11~16是躯干到头部
        self.line(ptdata[0, :], ptdata[11, :], color)
        self.polyLines(ptdata[11:17, :], color)
        # 13-17~23是左臂
        self.line(ptdata[13, :], ptdata[17, :], color)
        self.polyLines(ptdata[17:24, :], color)
        # 13-24~30是右臂
        self.line(ptdata[13, :], ptdata[24, :], color)
        self.polyLines(ptdata[24:31, :], color)

    def text3D(self, pos, scale=0.5):
        dim = pos.ndim  # 数据的维度
        if dim == 1:
            num = int(max(pos.shape) / 3)  # 确定有几个点要绘制
            ptdata = pos.reshape(num, 3)  # 转换成num行3列的矩阵
        else:
            num = pos.shape[0]
            ptdata = pos

        colors = vtk.vtkNamedColors()
        for i in range(num):
            pt = ptdata[i, :]
            textSource = vtk.vtkTextSource()
            textSource.SetText(str(i))
            # textSource.SetForegroundColor(colors.GetColor3d('DarkSlateGray'))
            # textSource.SetBackgroundColor(colors.GetColor3d('NavajoWhite'))
            textSource.SetForegroundColor(np.array([0, 0, 255]) / 255.0)
            textSource.SetBackgroundColor(np.array([255, 0, 0]) / 255.0)
            textSource.BackingOff()  # 不要显示背景
            # textSource.Update()

            # Create a mapper and actor
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(textSource.GetOutputPort())

            # actor = vtk.vtkActor()
            actor = vtk.vtkFollower()
            actor.SetMapper(mapper)
            actor.SetPosition(pt[0], pt[1], pt[2])
            actor.SetScale(scale, scale, scale)
            actor.SetCamera(self.renderer.GetActiveCamera())  # 设定跟随那个相机转动

            self.renderer.AddActor(actor)

    def text3D_v2(self, pos, scale=0.5):
        dim = pos.ndim  # 数据的维度
        if dim == 1:
            num = int(max(pos.shape) / 3)  # 确定有几个点要绘制
            ptdata = pos.reshape(num, 3)  # 转换成num行3列的矩阵
        else:
            num = pos.shape[0]
            ptdata = pos

        colors = vtk.vtkNamedColors()
        for i in range(num):
            pt = ptdata[i, :]
            text = vtk.vtkVectorText()
            text.SetText(str(i))

            # Create a mapper and actor
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(text.GetOutputPort())

            # actor = vtk.vtkActor()
            actor = vtk.vtkFollower()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(np.array([0, 0, 255]) / 255.0)  # 颜色
            actor.SetPosition(pt[0], pt[1], pt[2])  # 位置
            actor.SetScale(scale, scale, scale)  # 缩放比例
            actor.SetCamera(self.renderer.GetActiveCamera())  # 设定跟随那个相机转动

            self.renderer.AddActor(actor)

    def text(self, pos, str, scale=0.5):
        colors = vtk.vtkNamedColors()
        pt = np.array(pos)
        text = vtk.vtkVectorText()
        text.SetText(str)

        # Create a mapper and actor
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(text.GetOutputPort())

        # actor = vtk.vtkActor()
        actor = vtk.vtkFollower()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(np.array([0, 0, 255]) / 255.0)  # 颜色
        actor.SetPosition(pt[0], pt[1], pt[2])  # 位置
        actor.SetScale(scale, scale, scale)  # 缩放比例
        actor.SetCamera(self.renderer.GetActiveCamera())  # 设定跟随那个相机转动
        self.renderer.AddActor(actor)

    def drawOneFrame(self, joints, joint_radius=20, joint_color=np.array([255, 0, 0]) / 255.0, txt_scale=0.5):
        for key, val in joints.items():
            self.sphere(val.Xw, joint_radius, joint_color)  # 绘制节点圆球
            self.text(val.Xw, str=key, scale=txt_scale)  # 绘制节点文字
            self.axes(val.Hw, scale=100)  # 节点局部坐标轴
            if key != 'Hips':  # 身体连线
                self.line(val.Xw, joints[val.parentName].Xw, color=np.array([54, 175, 201]) / 255.0)

        # # 绘制节点圆球
        # self.sphere(joints['Hips'].Xw, joint_radius, joint_color)
        # self.sphere(joints['Spine'].Xw, joint_radius, joint_color)
        # self.sphere(joints['Spine1'].Xw, joint_radius, joint_color)
        # self.sphere(joints['Neck'].Xw, joint_radius, joint_color)
        # self.sphere(joints['Head'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftShoulder'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftArm'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftForeArm'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftHand'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightShoulder'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightArm'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightForeArm'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightHand'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftUpLeg'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftLeg'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftFoot'].Xw, joint_radius, joint_color)
        # self.sphere(joints['LeftToeBase'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightUpLeg'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightLeg'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightFoot'].Xw, joint_radius, joint_color)
        # self.sphere(joints['RightToeBase'].Xw, joint_radius, joint_color)
        # # 绘制节点文字
        # self.text(joints['Hips'].Xw, str='Hips', scale=txt_scale)
        # self.text(joints['Spine'].Xw, str='Spine', scale=txt_scale)
        # self.text(joints['Spine1'].Xw, str='Spine1', scale=txt_scale)
        # self.text(joints['Neck'].Xw, str='Neck', scale=txt_scale)
        # self.text(joints['Head'].Xw, str='Head', scale=txt_scale)
        # self.text(joints['LeftShoulder'].Xw, str='LeftShoulder', scale=txt_scale)
        # self.text(joints['LeftArm'].Xw, str='LeftArm', scale=txt_scale)
        # self.text(joints['LeftForeArm'].Xw, str='LeftForeArm', scale=txt_scale)
        # self.text(joints['LeftHand'].Xw, str='LeftHand', scale=txt_scale)
        # self.text(joints['RightShoulder'].Xw, str='RightShoulder', scale=txt_scale)
        # self.text(joints['RightArm'].Xw, str='RightArm', scale=txt_scale)
        # self.text(joints['RightForeArm'].Xw, str='RightForeArm', scale=txt_scale)
        # self.text(joints['RightHand'].Xw, str='RightHand', scale=txt_scale)
        # self.text(joints['LeftUpLeg'].Xw, str='LeftUpLeg', scale=txt_scale)
        # self.text(joints['LeftLeg'].Xw, str='LeftLeg', scale=txt_scale)
        # self.text(joints['LeftFoot'].Xw, str='LeftFoot', scale=txt_scale)
        # self.text(joints['LeftToeBase'].Xw, str='LeftToeBase', scale=txt_scale)
        # self.text(joints['RightUpLeg'].Xw, str='RightUpLeg', scale=txt_scale)
        # self.text(joints['RightLeg'].Xw, str='RightLeg', scale=txt_scale)
        # self.text(joints['RightFoot'].Xw, str='RightFoot', scale=txt_scale)
        # self.text(joints['RightToeBase'].Xw, str='RightToeBase', scale=txt_scale)
        # # 身体连线
        # color = np.array([54, 175, 201]) / 255.0
        # self.line(joints['Hips'].Xw, joints['Spine'].Xw, color)
        # self.line(joints['Spine'].Xw, joints['Spine1'].Xw, color)
        # self.line(joints['Spine1'].Xw, joints['Neck'].Xw, color)
        # self.line(joints['Neck'].Xw, joints['Head'].Xw, color)
        # self.line(joints['Spine1'].Xw, joints['LeftShoulder'].Xw, color)
        # self.line(joints['LeftShoulder'].Xw, joints['LeftArm'].Xw, color)
        # self.line(joints['LeftArm'].Xw, joints['LeftForeArm'].Xw, color)
        # self.line(joints['LeftForeArm'].Xw, joints['LeftHand'].Xw, color)
        # self.line(joints['Spine1'].Xw, joints['RightShoulder'].Xw, color)
        # self.line(joints['RightShoulder'].Xw, joints['RightArm'].Xw, color)
        # self.line(joints['RightArm'].Xw, joints['RightForeArm'].Xw, color)
        # self.line(joints['RightForeArm'].Xw, joints['RightHand'].Xw, color)
        # self.line(joints['Hips'].Xw, joints['LeftUpLeg'].Xw, color)
        # self.line(joints['LeftUpLeg'].Xw, joints['LeftLeg'].Xw, color)
        # self.line(joints['LeftLeg'].Xw, joints['LeftFoot'].Xw, color)
        # self.line(joints['LeftFoot'].Xw, joints['LeftToeBase'].Xw, color)
        # self.line(joints['Hips'].Xw, joints['RightUpLeg'].Xw, color)
        # self.line(joints['RightUpLeg'].Xw, joints['RightLeg'].Xw, color)
        # self.line(joints['RightLeg'].Xw, joints['RightFoot'].Xw, color)
        # self.line(joints['RightFoot'].Xw, joints['RightToeBase'].Xw, color)
        # # 节点局部坐标轴
        # scale = 100
        # self.axes(joints['Hips'].Hw, scale)
        # self.axes(joints['Spine'].Hw, scale)
        # self.axes(joints['Spine1'].Hw, scale)
        # self.axes(joints['Neck'].Hw, scale)
        # self.axes(joints['Head'].Hw, scale)
        # self.axes(joints['LeftShoulder'].Hw, scale)
        # self.axes(joints['LeftArm'].Hw, scale)
        # self.axes(joints['LeftForeArm'].Hw, scale)
        # self.axes(joints['LeftHand'].Hw, scale)
        # self.axes(joints['RightShoulder'].Hw, scale)
        # self.axes(joints['RightArm'].Hw, scale)
        # self.axes(joints['RightForeArm'].Hw, scale)
        # self.axes(joints['RightHand'].Hw, scale)
        # self.axes(joints['LeftUpLeg'].Hw, scale)
        # self.axes(joints['LeftLeg'].Hw, scale)
        # self.axes(joints['LeftFoot'].Hw, scale)
        # self.axes(joints['LeftToeBase'].Hw, scale)
        # self.axes(joints['RightUpLeg'].Hw, scale)
        # self.axes(joints['RightLeg'].Hw, scale)
        # self.axes(joints['RightFoot'].Hw, scale)
        # self.axes(joints['RightToeBase'].Hw, scale)

    def show(self):
        self.renderer.ResetCamera()
        self.renderer.GetActiveCamera().Azimuth(30)
        self.renderer.GetActiveCamera().Elevation(30)
        self.renderer.ResetCameraClippingRange()
        self.renderer.SetBackground(vtk.vtkNamedColors().GetColor3d("Silver"))
        self.renderWindow.Render()
        self.renderWindow.SetWindowName(self.window_name)

        style = vtk.vtkInteractorStyleTrackballCamera()
        self.renderWindowInteractor.SetInteractorStyle(style)  # 该设置让鼠标控制界面更易用
        # self.picker.AddObserver("EndPickEvent", self.pickerfunc)  # 将事件与回调函数建立连接
        # style.AddObserver('LeftButtonPressEvent', self.pickerfunc)
        # style.AddObserver('LeftButtonReleaseEvent', self.pickerfunc)

        # style.AddObserver('MouseMoveEvent', self.pickerfunc)
        # style.RemoveObservers ('LeftButtonPressEvent')        ##删除消息事件连接
        # style.RemoveObservers ('LeftButtonReleaseEvent')
        # style.RemoveObservers ('MouseMoveEvent')
        self.renderWindowInteractor.SetPicker(self.picker)

        self.renderWindowInteractor.Initialize()
        self.renderWindowInteractor.Start()

    def setLoader(self, loader):
        self.dataloader = loader

    def updateOneFrame(self):
        self.dataloader.updateOneFrameData(self.frame_index)  # 读取并更新第N帧数据
        self.dataloader.calJointPos()  # 计算每个节点的世界坐标
        self.drawOneFrame(self.dataloader.joints, joint_radius=30, txt_scale=25)
        self.renderWindow.Render()
        self.renderWindow.SetWindowName("frame: {}".format(self.frame_index))
        self.frame_index += 1


class BVHJoint():
    def __init__(self, OFFSET, parent_name):
        offset = np.array(OFFSET) * 10  # 从cm转换成mm
        self.offset = offset.reshape(3, 1)  # 转换成3行1列的np数组
        self.R = None  # 通过CHANNELS解析得到的旋转
        self.euler = None
        self.T = None  # 通过CHANNELS解析得到的平移
        self.H = None  # 当前节点局部坐标系的H
        self.Hw = None  # 从当前节点到世界坐标系的4x4变换矩阵H，包含了R、T、offset信息
        self.Xw = None  # 当前节点的世界坐标
        self.parentName = parent_name  # 父节点的名字
        self.r = np.zeros(3)  # 局部坐标系下的旋转向量（训练用）

    def addChannels(self, CHANNELS):
        if len(CHANNELS) < 6:  # 只有欧拉角
            self.R = eulerAnglesToRotationMatrix(CHANNELS, format='degree')  # 通过CHANNELS解析得到的旋转
            self.euler = CHANNELS
            self.T = np.zeros([3, 1])
        else:  # 有位置和欧拉角
            T = np.array(CHANNELS[0:3]) * 10  # 从cm转换成mm
            self.T = T.reshape(3, 1)  # 通过CHANNELS解析得到的平移，从cm转换成mm
            self.R = eulerAnglesToRotationMatrix(CHANNELS[3:6], format='degree')  # 通过CHANNELS解析得到的旋转
            self.euler = CHANNELS[3:6]

        H = np.eye(4)
        H[0:3, 0:3] = self.R
        H[0:3, [3]] = self.T + self.offset
        self.H = H  # 当前节点局部坐标系的H
        cv2.Rodrigues(self.R, self.r)  # 计算旋转向量

    def cal_Xw(self, parentJoint):
        self.Hw = np.dot(parentJoint.Hw, self.H)
        self.Xw = self.Hw[0:3, 3]
        # X0 = np.zeros([4, 1])
        # X0[3, 0] = 1  # 每个节点在其自身局部坐标系下的坐标都为0，这里用齐次坐标表示
        # self.Xw = np.dot(self.Hw, X0)

    def generateUDPdata(self):  # 生成UDP发送所需的左手系下的四元数和T，描述的是Xw=RX+T中的R和T
        # 右手系RT转左手系RT
        Rl, Tl = Right2Left("X", self.Hw[0:3, 0:3], self.Hw[0:3, 3])
        # 转换R为四元数
        q = Rot2Quat(Rl)
        return q, Tl,Rl


class BVH_Loader():
    def __init__(self, file_path):
        with open(file_path) as f:
            self.mocap = Bvh(f.read())

        self.joints = {}  # 建立节点字典，注意父节点一定要在子节点之前添加！
        self.joints['Hips'] = BVHJoint(self.mocap.joint_offset('Hips'), None)#0
        self.joints['Spine'] = BVHJoint(self.mocap.joint_offset('Spine'), 'Hips')#1
        self.joints['Spine1'] = BVHJoint(self.mocap.joint_offset('Spine1'), 'Spine')#2
        self.joints['Neck'] = BVHJoint(self.mocap.joint_offset('Neck'), 'Spine1')#3
        self.joints['Head'] = BVHJoint(self.mocap.joint_offset('Head'), 'Neck')#4
        self.joints['LeftShoulder'] = BVHJoint(self.mocap.joint_offset('LeftShoulder'), 'Spine1')#5
        self.joints['LeftArm'] = BVHJoint(self.mocap.joint_offset('LeftArm'), 'LeftShoulder')#6
        self.joints['LeftForeArm'] = BVHJoint(self.mocap.joint_offset('LeftForeArm'), 'LeftArm')#7
        self.joints['LeftHand'] = BVHJoint(self.mocap.joint_offset('LeftHand'), 'LeftForeArm')#8
        self.joints['RightShoulder'] = BVHJoint(self.mocap.joint_offset('RightShoulder'), 'Spine1')#9
        self.joints['RightArm'] = BVHJoint(self.mocap.joint_offset('RightArm'), 'RightShoulder')#10
        self.joints['RightForeArm'] = BVHJoint(self.mocap.joint_offset('RightForeArm'), 'RightArm')#11
        self.joints['RightHand'] = BVHJoint(self.mocap.joint_offset('RightHand'), 'RightForeArm')#12
        self.joints['LeftUpLeg'] = BVHJoint(self.mocap.joint_offset('LeftUpLeg'), 'Hips')#13
        self.joints['LeftLeg'] = BVHJoint(self.mocap.joint_offset('LeftLeg'), 'LeftUpLeg')#14
        self.joints['LeftFoot'] = BVHJoint(self.mocap.joint_offset('LeftFoot'), 'LeftLeg')#15
        self.joints['LeftToeBase'] = BVHJoint(self.mocap.joint_offset('LeftToeBase'), 'LeftFoot')#16
        self.joints['RightUpLeg'] = BVHJoint(self.mocap.joint_offset('RightUpLeg'), 'Hips')#17
        self.joints['RightLeg'] = BVHJoint(self.mocap.joint_offset('RightLeg'), 'RightUpLeg')#18
        self.joints['RightFoot'] = BVHJoint(self.mocap.joint_offset('RightFoot'), 'RightLeg')#19
        self.joints['RightToeBase'] = BVHJoint(self.mocap.joint_offset('RightToeBase'), 'RightFoot')  # 共21个节点

        #self.joint_pq=np.zeros((1,21*7),dtype='float')
        # self.joints['HeadEnd'] = BVHJoint(self.mocap.joint_offset('Site'), 'Head')
        # self.joints['LeftHandEnd'] = BVHJoint(self.mocap.joint_offset('Site'), 'LeftHand')

    # 更新一帧数据
    def updateOneFrameData(self, frame_idx):
        # 添加每个节点的channel信息，计算R,T,H矩阵
        for key, val in self.joints.items():
            if key is 'Hips':  # 只有Hips有位移信息
                val.addChannels(self.mocap.frame_joint_channels(frame_idx, key,
                                                                ["Xposition", "Yposition", "Zposition",
                                                                 "Zrotation", "Xrotation", "Yrotation"]))
            else:
                val.addChannels(self.mocap.frame_joint_channels(frame_idx, key,
                                                                ["Zrotation", "Xrotation", "Yrotation"]))

        # self.joints['Hips'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "Hips",
        #                                     ["Xposition", "Yposition", "Zposition", "Zrotation", "Xrotation",
        #                                      "Yrotation"]))
        #
        # self.joints['Spine'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "Spine", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['Spine1'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "Spine1", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['Neck'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "Neck", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['Head'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "Head", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftShoulder'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftShoulder", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftArm'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftArm", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftForeArm'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftForeArm", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftHand'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftHand", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightShoulder'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightShoulder", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightArm'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightArm", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightForeArm'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightForeArm", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightHand'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightHand", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftUpLeg'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftUpLeg", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftLeg'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftLeg", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftFoot'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftFoot", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['LeftToeBase'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "LeftToeBase", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightUpLeg'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightUpLeg", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightLeg'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightLeg", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightFoot'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightFoot", ["Zrotation", "Xrotation", "Yrotation"]))
        #
        # self.joints['RightToeBase'].addChannels(
        #     self.mocap.frame_joint_channels(frame_idx, "RightToeBase", ["Zrotation", "Xrotation", "Yrotation"]))

    # 计算每个节点的世界坐标
    def calJointPos(self):      
        for key, val in self.joints.items():
            # print('当前节点是：' + key)
            if key is 'Hips':
                val.Hw = val.H  # 只有根节点满足该等式
                val.Xw = val.Hw[0:3, 3]
            else:
                val.cal_Xw(self.joints[val.parentName])  # 利用父节点计算当前节点的坐标    
    

        # # 从根节点Hips开始计算
        # X0 = np.zeros([4, 1])
        # X0[3, 0] = 1  # 每个节点在其自身局部坐标系下的坐标都为0，这里用齐次坐标表示
        # self.joints['Hips'].Hw = self.joints['Hips'].H  # 只有根节点满足该等式
        # # self.joints['Hips'].Xw = np.dot(self.joints['Hips'].Hw, X0)
        # self.joints['Hips'].Xw = self.joints['Hips'].Hw[0:3, 3]
        #
        # # Spine
        # self.joints['Spine'].cal_Xw(parentJoint=self.joints['Hips'])  # 利用根节点计算当前节点的坐标
        # # Spine1
        # self.joints['Spine1'].cal_Xw(parentJoint=self.joints['Spine'])
        # # Neck
        # self.joints['Neck'].cal_Xw(parentJoint=self.joints['Spine1'])
        # # Head
        # self.joints['Head'].cal_Xw(parentJoint=self.joints['Neck'])
        #
        # # LeftShoulder
        # self.joints['LeftShoulder'].cal_Xw(parentJoint=self.joints['Spine1'])
        # # LeftArm
        # self.joints['LeftArm'].cal_Xw(parentJoint=self.joints['LeftShoulder'])
        # # LeftForeArm
        # self.joints['LeftForeArm'].cal_Xw(parentJoint=self.joints['LeftArm'])
        # # LeftHand
        # self.joints['LeftHand'].cal_Xw(parentJoint=self.joints['LeftForeArm'])
        #
        # # RightShoulder
        # self.joints['RightShoulder'].cal_Xw(parentJoint=self.joints['Spine1'])
        # # RightArm
        # self.joints['RightArm'].cal_Xw(parentJoint=self.joints['RightShoulder'])
        # # RightForeArm
        # self.joints['RightForeArm'].cal_Xw(parentJoint=self.joints['RightArm'])
        # # RightHand
        # self.joints['RightHand'].cal_Xw(parentJoint=self.joints['RightForeArm'])
        #
        # # LeftUpLeg
        # self.joints['LeftUpLeg'].cal_Xw(parentJoint=self.joints['Hips'])
        # # LeftLeg
        # self.joints['LeftLeg'].cal_Xw(parentJoint=self.joints['LeftUpLeg'])
        # # LeftFoot
        # self.joints['LeftFoot'].cal_Xw(parentJoint=self.joints['LeftLeg'])
        # # LeftToeBase
        # self.joints['LeftToeBase'].cal_Xw(parentJoint=self.joints['LeftFoot'])
        #
        # # RightUpLeg
        # self.joints['RightUpLeg'].cal_Xw(parentJoint=self.joints['Hips'])
        # # RightLeg
        # self.joints['RightLeg'].cal_Xw(parentJoint=self.joints['RightUpLeg'])
        # # RightFoot
        # self.joints['RightFoot'].cal_Xw(parentJoint=self.joints['RightLeg'])
        # # RightToeBase
        # self.joints['RightToeBase'].cal_Xw(parentJoint=self.joints['RightFoot'])


class C3D_Loader():
    def __init__(self, file_path):
        c = c3d(file_path)
        self.point_data = c['data']['points']

    def getOneFrameData(self, frame_idx):
        frame = self.point_data[:, :, frame_idx]  # 4x37的矩阵

        # 对点云进行欧式变换，以便与bvh坐标系对齐
        r = [-math.pi / 2, 0., 0.]
        t = [0., 0., 0.]
        H1 = cal_H(r, t)

        # r = [0., math.pi, 0.]
        # t = [0., 0., 0.]
        # H2 = cal_H(r, t)
        #
        # H = np.dot(H2, H1)

        frame = np.dot(H1, frame)  # 旋转变换
        frame = frame[0:3, :]
        frame = np.transpose(np.array(frame))  # 转换成37x3的矩阵

        return frame * 1000  # 单位转换为mm

    # def getOneFrameData(self, frame_idx):
    #     frame = self.point_data[:, :, frame_idx]  # 4x37的矩阵
    #     frame = frame[0:3, :]
    #     frame = np.transpose(np.array(frame))  # 转换成37x3的矩阵
    #
    #     tmp = frame.copy()
    #     # C3D数据输出时，做了Y=-Z，Z=Y的变换，因此这里需要进行逆操作
    #     frame[:, 1] = tmp[:, 2]  # Y=Z
    #     frame[:, 2] = -tmp[:, 1]  # Z=-Y
    #     return frame * 1000  # 单位转换为mm

def run():
    frame_index = 0  # 第几帧数据
    bvhDrawer = BVHDrawer("bvh data frame: {}".format(frame_index), window_size=[800, 600])
    bvh_loader = BVH_Loader('test.bvh')  # 加载文件
    bvh_loader.updateOneFrameData(frame_index)  # 读取并更新第N帧数据
    
# =============================================================================
#     # 将模型根节点的旋转平移归零
#     bvh_loader.joints['Hips'].R = np.eye(3, 3)
#     bvh_loader.joints['Hips'].T[0, 0] = 0
#     bvh_loader.joints['Hips'].T[2, 0] = 0
#     bvh_loader.joints['Hips'].H[0:3, 0:3] = bvh_loader.joints['Hips'].R
#     bvh_loader.joints['Hips'].H[0:3, [3]] = bvh_loader.joints['Hips'].T
#     # 将模型根节点的旋转平移归零
# =============================================================================
    bvh_loader.calJointPos()  # 计算每个节点的世界坐标

    bvhDrawer.drawOneFrame(bvh_loader.joints, joint_radius=30, txt_scale=25)
    # bvh_loader.generateTrainingData()  # 生成训练数据（每个节点的旋转向量）

    c3d_loader = C3D_Loader('test.c3d')
    pt_pos = c3d_loader.getOneFrameData(frame_index)
    bvhDrawer.multi_sphere(pt_pos, radius=20, color=np.array([0, 0, 255]) / 255.0)
    bvhDrawer.show()



def testBVH():
    bvhDrawer = BVHDrawer(window_size=[800, 600])
    bvh_loader = BVH_Loader('SrcData/Take Dance 1 Multi.bvh')  # 加载文件
    bvhDrawer.setLoader(bvh_loader)
    bvhDrawer.show()


def UDPsend(IP,PORT):
    bvh_loader = BVH_Loader('test.bvh')  # 加载文件
    frames = bvh_loader.mocap.nframes
    # client 发送端
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #IP = "127.0.0.1"  # IP地址
    #PORT = 4321 # 端口号
    server_address = (IP, PORT)  # 接收方 服务器的ip地址和端口号
    jointName = ['Head', 'Spine', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot']  # 需要发送数据的节点名称
    #jointName = ['Head']  # 需要发送数据的节点名称
    
    #多发几帧T-pose
    for frame_index in range(100):
        bvh_loader.updateOneFrameData(0)  # 读取并更新第N帧数据
         # 将模型根节点的旋转平移归零
        bvh_loader.joints['Hips'].R = np.eye(3, 3)
        bvh_loader.joints['Hips'].T[0, 0] = 0
        bvh_loader.joints['Hips'].T[2, 0] = 0
        bvh_loader.joints['Hips'].H[0:3, 0:3] = bvh_loader.joints['Hips'].R
        bvh_loader.joints['Hips'].H[0:3, [3]] = bvh_loader.joints['Hips'].T
         # 将模型根节点的旋转平移归零
        bvh_loader.calJointPos()  # 计算每个节点的世界坐标
        msg = ''
        # msg += '第{}帧数据: '.format(frame_index)
        for name in jointName:
            q, T,R = bvh_loader.joints[name].generateUDPdata()  # 生成该节点的发送数据
            msg += ''.join(str(i) + ' ' for i in q)
            msg += ''.join(str(i/1000) + ' ' for i in T)#mm转换为m
            if name=='Head':
                print(T/1000)
        client_socket.sendto(msg.encode('gbk'), server_address)  # 将msg内容发送给指定接收方
        time.sleep(0.08333)        

    for frame_index in range(frames):
        bvh_loader.updateOneFrameData(frame_index)  # 读取并更新第N帧数据
        bvh_loader.calJointPos()  # 计算每个节点的世界坐标
        msg = ''
        # msg += '第{}帧数据: '.format(frame_index)
        for name in jointName:
            q, T,R = bvh_loader.joints[name].generateUDPdata()  # 生成该节点的发送数据
            msg += ''.join(str(i) + ' ' for i in q)
            msg += ''.join(str(i/1000) + ' ' for i in T)#mm转换为m
# =============================================================================
#             if name=='Head':
#                 print(q)
# =============================================================================
        client_socket.sendto(msg.encode('gbk'), server_address)  # 将msg内容发送给指定接收方
        time.sleep(0.08333)  

def SaveUnity(path_in,path_out):
    bvh_loader = BVH_Loader(path_in)  # 加载文件
    frames = bvh_loader.mocap.nframes    
    jointName = ['Head', 'Spine', 'LeftHand', 'RightHand', 'LeftFoot', 'RightFoot']  # 需要发送数据的节点名称
    data=np.zeros((1,len(jointName)*7),dtype=float)

    for frame_index in range(100):
        bvh_loader.updateOneFrameData(0)  # 读取并更新第N帧数据
         # 将模型根节点的旋转平移归零
        bvh_loader.joints['Hips'].R = np.eye(3, 3)
        bvh_loader.joints['Hips'].T[0, 0] = 0
        bvh_loader.joints['Hips'].T[2, 0] = 0
        bvh_loader.joints['Hips'].H[0:3, 0:3] = bvh_loader.joints['Hips'].R
        bvh_loader.joints['Hips'].H[0:3, [3]] = bvh_loader.joints['Hips'].T
         # 将模型根节点的旋转平移归零
        bvh_loader.calJointPos()  # 计算每个节点的世界坐标
        
        d=np.zeros((1,len(jointName)*7),dtype=float)
        for i in range(len(jointName)):
            name=jointName[i]
            q, T,R = bvh_loader.joints[name].generateUDPdata()
            T=T/1000
            d[0,i*7:i*7+4]=q.reshape(1,4)
            d[0,i*7+4:i*7+7]=T.reshape(1,3)
        data=np.r_[data,d]


    for frame_index in range(frames):
        bvh_loader.updateOneFrameData(frame_index)  # 读取并更新第N帧数据
        bvh_loader.calJointPos()  # 计算每个节点的世界坐标
        
        d=np.zeros((1,len(jointName)*7),dtype=float)
        for i in range(len(jointName)):
            name=jointName[i]
            q, T,R = bvh_loader.joints[name].generateUDPdata()          
            T=T/1000
            d[0,i*7:i*7+4]=q.reshape(1,4)
            d[0,i*7+4:i*7+7]=T.reshape(1,3)
        data=np.r_[data,d]            
    data=data[1:,:]    
    np.savetxt(path_out,data)
    

if __name__ == '__main__':
    UDPsend("127.0.0.1",4321)
    #SaveUnity("Take Dance 1 Multi.bvh","SendUnity6Tracker1.txt")

