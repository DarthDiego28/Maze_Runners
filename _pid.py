from machine import Pin,PWM,SoftI2C,ADC
from PID import PID
import time

#Pines para controlar PWM (velocidad) de los motores
pwm_der=PWM(Pin(1))
pwm_izq=PWM(Pin(8))

#Configuración de parámetros iniciales de pines PWM
pwm_der.freq(500)
pwm_der.duty_u16(0)
pwm_izq.freq(500)
pwm_izq.duty_u16(0)

#Pines para controlar el sentido (adelante o atrás) del motor derecho
ad_der=Pin(15,Pin.OUT)
rev_der=Pin(14,Pin.OUT)

#Pines para controlar el sentido (adelante o atrás) del motor izquierdo
ad_izq=Pin(6,Pin.OUT)
rev_izq=Pin(7,Pin.OUT)

#Pin boton
boton = Pin(0,Pin.IN,Pin.PULL_DOWN)

#Configuración pines ADC
sen_izq=ADC(27)
sen_ad=ADC(28)
sen_der=ADC(29)
infrarrojo = ADC(26)

#Canal I2C
#bus_i2c=SoftI2C(sda=Pin(2),scl=Pin(3),freq=400000)

#Configuracion encoders
derecha_a=Pin(2,Pin.IN,Pin.PULL_DOWN)
derecha_b=Pin(5,Pin.IN,Pin.PULL_DOWN)
izquierda_a=Pin(3,Pin.IN,Pin.PULL_DOWN)
izquierda_b=Pin(4,Pin.IN,Pin.PULL_DOWN)

#Funciones
def pid_compute(error,motor):
    
    global error_1
    global error_2
    global error_3
    global error_4
    global error_5
    global error_6
    
    derivativo=error-last_prop
    integral=error_1+error_2+error_3+error_4+error_5+error_6
    last_prop=error
    error_6=error_5
    error_5=error_4
    error_4=error_3
    error_3=error_2
    error_2=error_1
    error_1=proporcional
    diferencial=int((proporcional*Kp) + (derivativo*Kd) + (integral*Ki))
    
    if(diferencial > vel):
        diferencial=vel
    elif(diferencial < -vel):
        diferencial=-vel
    if(diferencial < 0):
        motores(vel, vel+diferencial):motores(vel-diferencial, vel)
    if(motor == "derecha"):
        pwm_der.duty_u16(vel)
    else:
        pwm_izq.duty_u16(vel)

#Variables globales del programa

pid = PID(1, 0.1, 0.5, setpoint=0, scale='ms')
pid.sample_time = 10
pid.output_limits = (27000, 40000)
"""Colocar aquí el programa principal"""
#Inicialización de parámetros
ad_der.on()
ad_izq.on()
while boton.value()==0:
    pass
pwm_der.duty_u16(27000)
pwm_izq.duty_u16(27000)
while True:
    PID_input = sen_izq.read_u16() - sen_der.read_u16()
    vel = pid(PID_input)
    vel_derecha = 33000+int(vel)
    vel_izquierda = 33000-int(vel)
    if(vel_derecha > 65535):
        vel_derecha=65535
    if(vel_derecha < 27000):
        vel_derecha=0
    if(vel_izquierda > 65535):
        vel_izquierda=65535
    if(vel_izquierda < 27000):
        vel_izquierda=0
    pwm_der.duty_u16(vel_derecha)
    
    pwm_izq.duty_u16(vel_izquierda)
    print("motor derecha\t"+str(vel_derecha)+"\tmotor izquierda\t"+str(vel_izquierda)+"\n")
