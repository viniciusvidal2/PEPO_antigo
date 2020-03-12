#!/usr/bin/env python
import rospy
import sys, time
import numpy as np
from math import sqrt, pow, atan2
from geometry_msgs.msg import Twist
from servos60kg.srv import position
from adafruit_servokit import ServoKit

# Variaveis de controle dos servos
pan_atual  = 0
tilt_atual = 0
# Entidade de comando para os servos
kit = ServoKit(channels=16)

def callback(comando):
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual
    # Realizar o que deve ser feito com as mensagens daqui pra frente
    


def controle():
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual

    # Servidor para o servico desejado
    s = rospy.Service('/mover_servos', position, callback)

    # Publisher para as mensagens do estado dos servos
    pub = rospy.Publisher('/servos', Twist, queue_size=10)
    # Mensagem
    pos = Twist()

    r = rospy.Rate(10)
    while rospy.ok():
        # Mensagem alterada e publicada
        pos.linear.x = pan_atual
        pos.linear.y = tilt_atual
        pub.publish(pos)

        rospy.spinOnce()
        r.sleep()


if __name__ == '__main__':
    controle()