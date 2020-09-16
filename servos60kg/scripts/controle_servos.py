#!/usr/bin/env python3
import rospy
import sys, time
import numpy as np
from math              import sqrt, pow, atan2
from geometry_msgs.msg import Twist
from std_msgs.msg      import Bool
from servos60kg.srv    import Position
from servos60kg.srv    import LED
from adafruit_servokit import ServoKit
from board import SCL, SDA
import busio
from adafruit_pca9685  import PCA9685
from time import sleep

# Variaveis de controle dos servos
pan_atual  = 0
tilt_atual = 150
wait_step = 0.05
# Entidade de comando para os servos
kit = ServoKit(channels=16) # Indice 0 para pan, indice 1 para tilt
# Variavel que observa o LED
estado_led = 1

def callbackServos(comando):
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual
    #print('\n')
    #rospy.loginfo("Movendo os servos:")
    #rospy.loginfo("PAN: %.2f   TILT: %.2f", comando.pan, comando.tilt)
    #print('\n')
    # Realizar o que deve ser feito com as mensagens daqui pra frente
    kit.servo[0].angle = comando.pan/2
    kit.servo[1].angle = comando.tilt
    pan_atual  = comando.pan/2
    tilt_atual = comando.tilt

    sleep(wait_step)

    return True

def callbackLED(comando):
    # Variaveis globais
    global estado_led
    estado_led = comando.led

    return True


def controle():
    # Variaveis globais
    global kit
    global pan_atual
    global tilt_atual

    rospy.init_node('controle_servos', anonymous=False, disable_signals=False)
    rospy.loginfo("Iniciando no de controle dos servos ...")

    # Publisher para falar que o driver do servo ja iniciou
    pub_inicio = rospy.Publisher('/servos60kg/inicio', Bool, queue_size=10)
    inicio = Bool()
    
    inicio.data = False
    pub_inicio.publish(inicio)

    # Acertando a frequencia da porta
    pwm_freq = 30
    rospy.loginfo("Ajustando frequencia de PWM de saida para %d Hz...", pwm_freq)
    i2c_bus = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c_bus)
    pca.frequency = pwm_freq

    # Acertando range de angulo dos servos
    kit.servo[0].actuation_range = 180
    kit.servo[1].actuation_range = 180
    kit.servo[0].set_pulse_width_range(500, 2500)
    kit.servo[1].set_pulse_width_range(480, 2500)
    
    # Servidor para o servico desejado
    s = rospy.Service('/mover_servos', Position, callbackServos)

    # Servidor para controle do LED
    l = rospy.Service('/controle_led', LED, callbackLED)

    # Publisher para as mensagens do estado dos servos
    pub = rospy.Publisher('/servos60kg/estado', Twist, queue_size=10)
    
    # Mensagem
    pos = Twist()

    # Ja que inicia o driver no meio range de cada um, mover aos poucos para onde queremos
    rospy.loginfo("Movendo para a posicao inicial em PAN")
    for i in range(5, 0, -1): # PAN
        kit.servo[0].angle = i
        sleep(wait_step)
    rospy.loginfo("Movendo para a posicao inicial em TILT")
    for i in range(0, 135, 5): # TILT
        kit.servo[1].angle = i
        sleep(wait_step)

    taxa = 10
    rospy.loginfo("Rodando controle dos servos a %d Hz...", taxa)
    r = rospy.Rate(taxa)
    while not rospy.is_shutdown():
        # Confirmar que estamos bem
        inicio.data = True
        pub_inicio.publish(inicio)
        # Mensagem alterada e publicada
        pos.linear.x = pan_atual
        pos.linear.y = tilt_atual
        pub.publish(pos)

        # Variar segundo comando do LED
        if estado_led == 1: # continuo
            pca.channels[2].duty_cycle = 0xffff
        if estado_led == 2: # pisca rapido
            step = 100
            for i in range(0, 0xffff,  step):
                pca.channels[2].duty_cycle = i
            for i in range(0xffff, 0, -step):
                pca.channels[2].duty_cycle = i
        if estado_led == 3: # pisca lento
            step = 20
            for i in range(0, 0xffff,  step):
                pca.channels[2].duty_cycle = i
            sleep(0.5)
            for i in range(0xffff, 0, -step):
                pca.channels[2].duty_cycle = i
            sleep(0.5)

        # Rodar o ciclo ros
        r.sleep()

    rospy.spin()
    rospy.loginfo("Finalizando no ...")
    
if __name__ == '__main__':
    controle()
