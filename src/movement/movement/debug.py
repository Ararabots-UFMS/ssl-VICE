#!usr/bin/python

# TODO: evaluate avoid obstacle field for multiple obstacles

import numpy as np
import random
import cv2
import math
import time

from robot.movement.univector.un_field import UnivectorField
# import rospy
LEFT = 0
RIGHT = 1

SIMULATION = 0 # turn on simulation
              # if you turn on the simulantion please make the RIGHTectories: erros and erros/log
EPOCH = 1 # how many simulations (min value is one)


w, h = 150, 130 # the width and height sizes of the field in centimiters
teamColor = (255, 0 , 0)
enemyColor = (0, 255, 255)
ballColor = (31, 136, 246)
pathColor = (255, 0, 255)

global RADIUS
global KR
global K0
global DMIN
global LDELTA

RADIUS = 5.0
KR = 15
K0 = 25
DMIN = 5
LDELTA = 4.5

def getObstacle():
    return np.array([75, -75])
    #return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getBall():
    return np.array([50, -50])
    #return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def getRobot():
    return np.array([random.randint(0, w-1), -random.randint(0, h-1)])

def printObstacles(_obstacle, plt):
    for i in range(_obstacle.shape[0]):
        plt.plot(_obstacle[i][0], -obstacle[i][1], 'go')

def cm2pixel(pos):
    posArray = np.array(pos)
    return 4*posArray

def drawRobot(img, robotPos, enemy=False):
    if enemy:
        color = enemyColor
    else:
        color = teamColor
    pos = cm2pixel([robotPos[0], -robotPos[1]])
    topLeft = (pos[0]-15, pos[1]-15)
    bottomRight = (pos[0]+15, pos[1]+15)
    cv2.rectangle(img, topLeft, bottomRight, color, -1)

def drawObstacles(img, obstacles):
    if obstacles.size:
        for i in range(obstacles.shape[0]):
            drawRobot(img, obstacles[i], enemy=True)

def drawBall(img, ballPos):
    cv2.circle(img, (ballPos[0], -ballPos[1]), 9, ballColor, -1)

def drawField(img, univetField):
    for l in range(0, h, 3):
        for c in range(0, w, 3):
            pos = [c, -l]
            v = univetField.get_vec(_robotPos=pos, _vRobot=[0, 0])

            s = cm2pixel(np.array([c, l]))
            new = cm2pixel(np.array(pos)) + 10*v

            new[1] = -new[1]

            cv2.arrowedLine(img, tuple(np.int0(s)), tuple(np.int0(new)), (50,50,50), 1)

def drawPath(img, start, end, univetField):
    currentPos = start
    _currentPos = cm2pixel(currentPos)

    newPos = None
    alpha = 2
    beta = 10

    t0 = time.time()

    while(np.linalg.norm(currentPos - end) >= beta):
        v = univetField.get_vec(_robotPos=currentPos, _vRobot=[0, 0])
        newPos = currentPos + (alpha*np.array(v))
        _newPos = cm2pixel(newPos).astype(int)

        cv2.line(img, (_currentPos[0], -_currentPos[1]), (_newPos[0], -_newPos[1]), pathColor, 3)

        cv2.imshow('field', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        if (time.time() - t0 > 5):
            return False, newPos

        currentPos = newPos
        _currentPos = _newPos
    return True, None


def on_trackbar(val, origin):
    constants[origin] = val 
    univetField.updateConstants(constants['RADIUS'], constants['KR'], constants['K0'], 
    constants['DMIN'], constants['LDELTA'])
    draw_field()

global original

global robot
global ball
global enemy
global constants

def draw_field():
    imgField2 = np.copy(original)

    obstacle = np.array([[15,-15], [110,-15] , [15,-110], [110,-110] ])
    vObstacle = np.array([[10,0],[0,-10],[0,10],[-10,0]])

    # Drawing components
    drawRobot(imgField2, robot)
    drawObstacles(imgField2, obstacle)
    drawBall(imgField2, cm2pixel(ball))

    drawField(imgField2, univetField)
    #ret, pos = drawPath(imgField2, robot, ball, univetField)

    cv2.imshow('field', imgField2)

if __name__ == "__main__":
    imgField = cv2.imread('img/vss-field.jpg')

    rep = EPOCH
    i = 0
    while rep > 0:
        imgField2 = np.copy(imgField)

        robot = getRobot()
        ball = getBall()

        obstacle = np.array([[]])
        vObstacle = np.array([[0, 0]])

        obstacle = np.array([getObstacle()])
        vObstacle = np.array([[0,0]])

        # Drawing components
        drawRobot(imgField2, robot)
        drawObstacles(imgField2, obstacle)
        drawBall(imgField2, cm2pixel(ball))

        # Creates the univector field
        univetField = UnivectorField()
        univetField.update_constants(RADIUS, KR, K0, DMIN, LDELTA)
        # univetField.update_ball(ball)
        univetField.update_obstacles(obstacle, vObstacle)


        drawField(imgField2, univetField)
        ret, pos = drawPath(imgField2, robot, ball, univetField)

        # display the path in the field
        if not SIMULATION:
            cv2.imshow('field', imgField2)
            cv2.waitKey(0)
            break
        else:
            if not ret:
                cv2.imwrite('./erros/Erro-'+ str(i)+'.jpg', imgField2)
                nomeArquivo = './erros/log/Erro-'+str(i)+'.txt'
                arquivo = open(nomeArquivo, 'w+')
                texto = "Obstacles: " + str(obstacle) + '\n'
                texto += "Ball: " + str(ball) + '\n'
                texto += "Robot: " + str(pos) + '\n'
                arquivo.writelines(texto)
                arquivo.close()

            rep -= 1
            print("SIMULATION", i)
            i += 1
