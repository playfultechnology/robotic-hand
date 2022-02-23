#!/usr/bin/python
#Ҫ /usr/bin/python ���Ŀ¼�� python ��ִ����Ľű�
# encoding: utf-8
import thread
import time
import sqlite3 as sql #����sqlite3һ������sql 
import pigpio
from LeServo import PWM_Servo #����LeServo�е�PWM_Servo
import os

Servos =( ) #����һ����Ԫ�飬����ʱ�ɲ�ָ��Ԫ�صĸ������൱�ڲ����������飬��һ�������Ͳ����޸�Ԫ��ĳ��ȡ�
runningAction = False #��ʼ���˶���ʶ
pi = None 
stopRunning = False #��ʼ��ֹͣ��ʶ

#####�ж�+��ֵ
def setServo(servoId,pos,time): #���ת������
    global runningAction
    if servoId < 1 or servoId > 6:
        return;
    if pos > 2500:
        pos = 2500
    elif pos < 500:
        pos = 500
    else:
        pass #ʲôҲ����
    if time > 30000:
        time = 30000
    elif time < 20:
        time = 20
    else:
        pass
    if runningAction is False: #���û�ж�����������
        Servos[servoId - 1].setPosition(pos, time)
        #-1����Ϊ�����Ǵ�0��ʼ�ģ���Python�Ľ������ڲ��������ǵ���Servos[servoId - 1].setPosition(pos, time)ʱ,
        #ʵ����Python���ͳ�PWM_Servo.setPosition(Servos[servoId - 1], pos, time)��Ҳ����˵��self�滻�����ʵ����

######��ȡ��ǰ���λ��
def setServo_CMP(servoId, pos, time):
    #print(servoId, pos, time)
    if servoId < 1 or servoId > 6:
        return;
    #print(Servos[servoId-1].getPosition())
    setServo(servoId, Servos[servoId - 1].getPosition() + pos, time);
    
#######ƫ�����
def setDeviation(servoId, d):
    global runningAction
    if servoId < 1 or servoId > 6:
        return
    if d < -300 or d > 300:
        return
    if runningAction is False:
        Servos[servoId -1].setDeviation(d) #ƫ������

def stopActionGroup(): #������ֹͣ����
    global stopRunning
    stopRunning = True

def runActionGroup(actNum, times) :
    global runningAction
    global stopRunning
    actNum = "./ActionGroups/" + actNum + ".d6a"
    print(actNum)
    if os.path.exists(actNum) is True: #������ڸö�����
        ag = sql.connect(actNum) #�����ݿ�actNum
        cu = ag.cursor() #������һ���α�
        cu.execute("select * from ActionGroup") #��ѯ
        if runningAction is False: #û�ж�����������
            runningAction = True #���иö�����
            while True:
                if stopRunning is True: #
                    stopRunning = False
                    runningAction = False
                    cu.close() #�ر�һ�����ݿ�����
                    ag.close() #�α�ر�
                    break;
                act = cu.fetchone() #�����б��еĵ�һ��ٴ�ʹ��,�򷵻صڶ���,������ȥ
                if act is not None:
                    print(act)
                    for i in range(0,6,1):
                        Servos[i].setPosition(act[2+i], act[1]) 
                    time.sleep(float(act[1])/1000.0) #����ʱ��
                else:
                    runningAction = False
                    cu.close()
                    ag.close()
                    break
    else:
        runningAction = False
        print("δ���ҵ��������ļ�")

    

def initLeArm(d):
    global Servos
    global pi 
    pi = pigpio.pi() #ʵ����
    servo1 = PWM_Servo(pi, 12,  deviation = d[0], control_speed = True) #��ʼ�������
    servo2 = PWM_Servo(pi, 16, deviation = d[1], control_speed = True)
    servo3 = PWM_Servo(pi, 20, deviation = d[2], control_speed = True)
    servo4 = PWM_Servo(pi, 21, deviation = d[3], control_speed = True)
    servo5 = PWM_Servo(pi, 19, deviation = d[4], control_speed = True)
    servo6 = PWM_Servo(pi, 13, deviation = d[5], control_speed = True)
    Servos = (servo1, servo2, servo3, servo4, servo5, servo6)
    for i in range(0,6,1) :
        Servos[i].setPosition(1500,1000)  #1-6�Ŷ��ת����λ
def stopLeArm():
    print("ֹͣ��е��")
    pi.stop() #�Ͽ�pwm����

if __name__ == "__main__":
    #��if __name__ == "__main__"��֮��������Ϊģ�鱻���õ�ʱ�����֮��Ĵ��벻ִ�У�
    #ֱ��ʹ�õ�ʱ�����֮��Ĵ���ִ�С�ͨ�������������ģ�������ʹ�á�
    initLeArm()
    time.sleep(2)
    print("ok")
    setServo(5,500,1000)
    time.sleep(20)
   # while True:
   #     ag = sql.connect("test.db")
   #     cu = ag.cursor()
   #     cu.execute("select * from ActionGroup")
   #     while True:
   #         act = cu.fetchone()
   #         if act is not None:
   #          #   print(act)
   #             setServo(0,act[2],act[1])
   #             setServo(1,act[3],act[1])
   #             setServo(2,act[4],act[1])
   #             setServo(3,act[5],act[1])
   #             setServo(4,act[6],act[1])
   #             setServo(4,act[7],act[1])
   #             time.sleep(act[1]/1000)
   #         else :
   #             cu.close()
   #             ag.close()
   #             break






