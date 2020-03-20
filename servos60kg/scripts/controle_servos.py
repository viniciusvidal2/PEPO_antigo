#!/usr/bin/env python3
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

def callbackServos(comando):
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual
    print('\n')
    rospy.loginfo("Movendo os servos:")
    rospy.loginfo("PAN: %.2f   TILT: %.2f", comando.pan, comando.tilt)
    # Realizar o que deve ser feito com as mensagens daqui pra frente
    kit.servo[0].angle = comando.pan/2
    kit.servo[1].angle = comando.tilt/2
    pan_atual  = comando.pan/2
    tilt_atual = comando.tilt/2

    return True

def controle():
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual

    rospy.init_node('controle_servos', anonymous=False, disable_signals=False)
    rospy.loginfo("Iniciando no de controle dos servos ...")

    # Acertando a frequencia da porta
    pwm_freq = 30
    rospy.loginfo("Ajustando frequencia de PWM de saida para %d Hz...", pwm_freq)
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = pwm_freq

    # Servidor para o servico desejado
    s = rospy.Service('/mover_servos', Position, callbackServos)

    # Publisher para as mensagens do estado dos servos
    pub = rospy.Publisher('/servos60kg/estado', Twist, queue_size=10)
    # Mensagem
    pos = Twist()

    taxa = 10
    rospy.loginfo("Rodando controle dos servos a %d Hz...", taxa)
    r = rospy.Rate(taxa)
    while not rospy.is_shutdown():
        # Mensagem alterada e publicada
        pos.linear.x = pan_atual
        pos.linear.y = tilt_atual
        pub.publish(pos)
        r.sleep()

    rospy.spin()
    rospy.loginfo("Finalizando no ...")
    
if __name__ == '__main__':
    controle()
