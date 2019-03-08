from math import sqrt
from Bipolar_Stepper_Motor_Class import Bipolar_Stepper_Motor
from numpy import arccos, pi, sin, cos
from ServoMotor import Servo
import time

#ark = open('square_0005.ngc')
#ark = open('I_0001.ngc')
#ark = open('I-I_0001.ngc')
#ark = open('otherI_0001.ngc')
#ark = open('S_0001.ngc')
#ark = open('FilledSquare_0001.ngc','r')
#ark = open('palavra_0001.ngc', 'r')
#ark = open('A_0001.ngc', 'r')
#ark = open('S2_0001.ngc')
#ark = open('intersection_0001.ngc', 'r')
ark = open('D_0001.ngc', 'r')
#ark = open('B_0001.ngc', 'r')
#ark = open('B2_0001.ngc', 'r')
#ark = open('2017_0001.ngc', 'r')
#ark = open('squares_0001.ngc', 'r')
#ark = open('swirl_0001.ngc', 'r')
#ark = open('ovo_0001.ngc', 'r')
#ark = open('1234_0001.ngc', 'r')
#ark = open('losango_0001.ngc', 'r')
#ark = open('bigSwirl_0001.ngc', 'r')
#ark = open('wand_0001.ngc', 'r')
#ark = open('6squares_0001.ngc', 'r')
#ark = open('sozenarede_0001.ngc', 'r')




code = ark.read().split('\n')
ark.close()
speed = 0
max_precision= 0.075
motorX = Bipolar_Stepper_Motor(35, 36, 37, 38)
motorY = Bipolar_Stepper_Motor(11, 12, 13, 15)
servo_motor = Servo()
servo_motor.sweep()
# TODO:REMOVE THE sweep method call above


def getCoordinates(line):
    x_pos = float(line.split(' ')[1].upper().lstrip('X'))
    y_pos = float(line.split(' ')[2].upper().lstrip('Y').rstrip('\n'))
    return x_pos, y_pos

def getDirection(x_pos, y_pos):
    x = int(round(float(x_pos) / max_precision)) - motorX.position
    y = int(round(float(y_pos) / max_precision)) - motorY.position

    if x < 0:
        direction_x = -1
    elif x > 0:
        direction_x = 1
    else:
        direction_x = 0
    if y < 0:
        direction_y = -1
    elif y > 0:
        direction_y = 1
    else:
        direction_y = 0

    return abs(x), direction_x, abs(y), direction_y

def getIJ(line):
    i_pos = float(line.split(' ')[4].upper().lstrip('I'))
    j_pos = float(line.split(' ')[5].upper().lstrip('J'))

    return i_pos, j_pos

def getGCD(x, y):
    while y:
        x, y = y, x%y
    return x

def getLCM(x, y):
    lcm = x*y / getGCD(x, y)
    return lcm

def getTotalSteps(x, y):
    if x == 0:
        total_steps = y
        step_y = 1
        step_x = total_steps + 100
    elif y == 0:
        total_steps = x
        step_x = 1
        step_y = total_steps + 100
    else:
        total_steps = getLCM(x, y)
        step_x = total_steps/x
        step_y = total_steps/y
    return total_steps, step_x, step_y

def runMotor(motorX, x, direction_x, step_x, motorY,
             y, direction_y, step_y, total_steps, speed):

    laps = True
    #print speed

    #if speed == 1:
    #    laps = False
    #
    speed *= 10
    print(speed)


    total_time = sqrt(x**2 + y**2)/speed
    #print total_time
    #print total_steps
    #print x
    #print y
    #print step_x
    #print step_y
    #print speed
    #print total_time

    if total_time != 0:
        delay = total_time/total_steps
        #print delay
        for i in range(1, total_steps +1):
            time_laps = 0

            if  i % step_x == 0:
                motorX.move(direction_x, 1, delay/4.0)
                time_laps += delay/4.0

            if i % step_y == 0:
                motorY.move(direction_y, 1, delay/4.0)
                time_laps += delay/4.0

            #if laps:
            #    print 'OK'
            #    print delay - time_laps
            #print (delay - time_laps) /2
            #print time_laps
                #time.sleep(0.0001)
            time.sleep((delay - time_laps)/1000000)

for line in code:
    #if 'M5' in line:
    #    break
    if 'M3' in line:
        print("starting program")
    if 'G21' in line:
        print("Using 'mm' as measure unity")
    if 'G20' in line:
        print("Using 'inch' as measure unity")
        max_precision *= 0.039370

    if 'Z5' in line and not servo_motor.isAtStart:
        """Moves z axis up"""
        servo_motor.toggle()
    if 'Z-' in line and servo_motor.istAtStart:
        """Moves z axis down"""
        servo_motor.toggle()

    if 'G' and ' X' and 'Y' in line:
        if 'G00' in line:
            x_pos, y_pos = getCoordinates(line)
            x, direction_x, y, direction_y = getDirection(x_pos, y_pos)
            speed = 1
            total_steps, step_x, step_y = getTotalSteps(x, y)
            runMotor(motorX, x, direction_x, step_x,
                     motorY, y, direction_y, step_y, total_steps, speed)

        if 'G01' in line:
            x_pos, y_pos = getCoordinates(line)
            x, direction_x, y, direction_y = getDirection(x_pos, y_pos)

            speed = 0.1/min(max_precision, max_precision)
            total_steps, step_x, step_y = getTotalSteps(x, y)
            runMotor(motorX, x, direction_x, step_x,
                     motorY, y, direction_y, step_y, total_steps, speed)


        if 'G02' in line or 'G03' in line:
            speed = 0.1/min(max_precision, max_precision)
            old_x_pos = x_pos
            old_y_pos = y_pos
            x_pos, y_pos = getCoordinates(line)
            i_pos, j_pos = getIJ(line)
            xcenter = old_x_pos + i_pos   #centro do circulo para interpolacao
            ycenter = old_y_pos + j_pos
            dx = x_pos - xcenter
            dy = y_pos - ycenter
            radius = sqrt(i_pos**2 + j_pos**2) #raio do circulo
            center_distance1 = [-i_pos, -j_pos]#distancia do centro

            if 'G02' in line: #sentido horario
                center_distance2 = [center_distance1[1], -center_distance1[0]]
            else:
                center_distance2 = [-center_distance1[1], center_distance1[0]]
            #Dx, Dy = e1*cos(theta) + e2*sin(theta), theta is the open angle
            cos_theta = (dx*center_distance1[0] + dy*center_distance1[1]) / radius**2
            sin_theta = (dx*center_distance2[0] + dy*center_distance2[1]) / radius**2
            #theta is the angule spanned by the circular interpolation curve
            if cos_theta > 1:
                cos_theta = 1
            elif cos_theta < -1:
                cos_theta = -1
            theta = arccos(cos_theta)

            if sin_theta < 0:
                theta = 2.0*pi - theta
            num_steps = int(round(radius*theta/max_precision/5.0)) # number of point for the circular interpolation

            for i in range(1, num_steps + 1):
                tmp_theta = i*theta/num_steps
                tmp_x = xcenter + center_distance1[0]*cos(
                    tmp_theta)+center_distance2[0]*sin(tmp_theta)
                tmp_y = ycenter+center_distance1[1]*cos(
                    tmp_theta) + center_distance2[1]*sin(tmp_theta)
                x, direction_x, y, direction_y = getDirection(tmp_x, tmp_y)
                total_steps, step_x, step_y = getTotalSteps(x, y)
                runMotor(motorX, x, direction_x, step_x,
                     motorY, y, direction_y, step_y, total_steps, speed)

motorX.unhold()
motorY.unhold()
servo_motor.clean()
