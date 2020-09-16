#!/usr/bin/python3
import sys, time
from adafruit_servokit import ServoKit
from board import SCL, SDA
import busio
from adafruit_pca9685  import PCA9685
from time import sleep

# Iniciando tudo
i2c_bus = busio.I2C(SCL, SDA)
pwm_freq = 30
porta_led = 2
pca = PCA9685(i2c_bus)
pca.frequency = pwm_freq

# Entidade de comando para os servos
#kit = ServoKit(channels=16) # Indice 0 para pan, indice 1 para tilt
#kit.servo[0].angle = 0
#kit.servo[1].angle = 150

# Piscando LED rapida V vezes
step = 100
V    = 20
for v in range(1, V):
    for i in range(0, 0xffff,  step):
        pca.channels[2].duty_cycle = i
    for i in range(0xffff, 0, -step):
        pca.channels[2].duty_cycle = i

# Deixando o led totalmente aceso
pca.channels[porta_led].duty_cycle = 0xffff

