#!/usr/bin/env python
import rospy
import sys, time
import numpy as np
from math              import sqrt, pow, atan2
from geometry_msgs.msg import Twist
from servos60kg.srv    import Position
from adafruit_servokit import ServoKit
from board import SCL, SDA
import busio
from adafruit_pca9685  import PCA9685

# Variaveis de controle dos servos
pan_atual  = 0
tilt_atual = 0
# Entidade de comando para os servos
kit = ServoKit(channels=16) # Indice 0 para pan, indice 1 para tilt

def callback(comando):
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual
    # Iniciar resposta com negativa
    comando.result = 0
    # Realizar o que deve ser feito com as mensagens daqui pra frente
    kit.servo[0].angle = comando.pan/2
    kit.servo[1].angle = comando.tilt/2
    pan_atual  = comando.pan/2
    tilt_atual = comando.tilt/2
    # Confirmar que deu certo o servico
    comando.result = 1

def controle():
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual

    # Acertando a frequencia da porta
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = 60

    # Servidor para o servico desejado
    s = rospy.Service('/mover_servos', position, callback)

    # Publisher para as mensagens do estado dos servos
    pub = rospy.Publisher('/servos60kg/estado', Twist, queue_size=10)
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