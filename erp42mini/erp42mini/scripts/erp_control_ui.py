#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from serial.serialposix import Serial #파이썬에서 시리얼 사용하게 만드는 모듈
import rospy   #파이썬에서 ros사용하는 모듈
import numpy as np #numpy모듈
import serial #시리얼 모듈
import cv2 #gui환경을 위해 opencv 

class SerialSender: #SerialSender 클래스 생성
    def __init__(self) -> None: #초기화
        rospy.init_node("erp42_mini_gui", anonymous=False) #ros master과 접속
        self.speed = 0 #속도만 만들어봄 
        #self.steer = 0 
        #self.brake = 0
        #self.direction = 0

        # launch 파일에 port 설정부분이 있으니 꼭 확인해주세요.
        self.port = rospy.get_param("/erp42_mini_gui/port")
        self.serial = serial.Serial(port=self.port, baudrate=115200)#시리얼 통신을 통해 외부로 정보 전송

    def serWrite(self):
        self.serial.write(self.writeBuffer())

    def writeBuffer(self):
        packet = []

        direction = self.direction

        # 타입 맞춰주기
        speed = np.uint16(self.speed)
        steer = np.uint16(self.steer)
        brake = np.uint16(self.brake)

        # 바이트 분할 작업
        speed_L = speed & 0xff #(십진수 255구나 and 연산을 한다고 보면 된다)
        speed_H = speed >> 8 #(오른쪽으로 8번 이동함) 이 작업을 통해서 바이트를 분할함

        steer_L = steer & 0xFF
        steer_H = steer >> 8

        brake_L = brake & 0xff
        brake_H = brake >> 8

        # CLC 계산을 위한 바이트 총합 구하기
        sum_c = direction + speed_L + speed_H + steer_L \
                    + steer_H + brake_L + brake_H + 13 + 10

        # CLC는 1 Byte
        clc = np.uint8(~sum_c)

        packet.append(0x53)
        packet.append(0x54)
        packet.append(0x58)
        packet.append(direction)
        packet.append(speed_L)
        packet.append(speed_H)
        packet.append(steer_L)
        packet.append(steer_H)
        packet.append(brake_L)
        packet.append(brake_H)
        packet.append(0x00)
        packet.append(0x0D)
        packet.append(0x0A)
        packet.append(clc)

        return packet        

    def run(self):
        self.sync_value()
        self.serWrite()

    def set_default(self):
        self.steer = 1550
        self.direction = 0
        self.speed = 0
        self.serWrite()


    # ==================================================================
    # 단순 GUI 구현부분이라 생략하셔도 됩니다.

    @staticmethod
    def nothing(self):
        pass

    def create_window(self):
        cv2.namedWindow("Control")
        cv2.createTrackbar("direction", "Control", 0, 1, self.nothing)
        cv2.createTrackbar("speed", "Control", 0, 800, self.nothing)
        cv2.createTrackbar("steer", "Control", 1300, 1800, self.nothing)
        cv2.createTrackbar("Brake", "Control", 1200, 1500, self.nothing)

        cv2.setTrackbarMax("steer", "Control", 1800)
        cv2.setTrackbarMin("steer", "Control", 1300)
        cv2.setTrackbarMax("Brake", "Control", 1500)
        cv2.setTrackbarMin("Brake", "Control", 1200)

        cv2.setTrackbarPos('steer', 'Control', 1550)
        cv2.setTrackbarPos('Brake', 'Control', 1500)
        cv2.setTrackbarPos('direction', 'Control', 0)
        cv2.setTrackbarPos('speed', 'Control', 0)

    def sync_value(self):
        self.direction = cv2.getTrackbarPos("direction", "Control")
        self.speed = cv2.getTrackbarPos("speed", "Control")
        self.steer = cv2.getTrackbarPos("steer", "Control")
        self.brake = cv2.getTrackbarPos("Brake", "Control")
    # ==================================================================


if __name__ == "__main__":
    ss = SerialSender()
    ss.create_window()
    while not rospy.is_shutdown():
        ss.run()
        cv2.waitKey(25)

