#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import tan, radians

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

mapa_para_area_de_resgate = {}
mapa_para_saida = {}
mapa_para_vitimas = {}
comprimento_horiz = 0
comprimento_robo = 180
comprimento_vertical_robo = 200
# Create your objects here.
ev3 = EV3Brick()
ultra_frontal = UltrasonicSensor(Port.S1)
ultra_esquerda = UltrasonicSensor(Port.S2)
ultra_direita = UltrasonicSensor(Port.S3)
motor_esquerda = Motor(Port.B)
motor_direita = Motor(Port.C)
robot = DriveBase(motor_direita, motor_esquerda, wheel_diameter=55.5, axle_track=120)

def angulo_para_girar_x_cm_na_parede(cat_adj, cat_op=150):
    tgparagraus={}
    for n in range(1, 360):
        tgparagraus[round(tan(radians(n/10)), 2)]=n/10
    tg = round(cat_op/cat_adj, 2)
    return tgparagraus[tg]
def e_um_triangulo_retangulo(hip, cat_adj, cat_op): return True if hip**2==cat_adj**2+cat_op**2 else False
# Write your program here.
ev3.speaker.beep()

catchesq_1 = ultra_esquerda.distance()
catchdir_1 = ultra_direita.distance()
catchfre_1 = ultra_frontal.distance()
comprimento_horizontal_1 = catchesq_1+comprimento_robo+catchdir_1
ev3.screen.draw_text(80, 10, catchfre_1)
ev3.screen.draw_text(10, 30, catchesq_1)
ev3.screen.draw_text(100, 30, catchdir_1)
ev3.screen.draw_text(80, 30, comprimento_horizontal_1)
wait(5000)
ev3.screen.clear()

posicao_inicial = 'esquerda' if catchesq_1 < catchdir_1 else 'direita'
saida_a_frente= catchfre_1>1100
area_a_frente= catchfre_1<800
area_ao_lado= comprimento_horizontal_1<800
saida_ao_lado= comprimento_horizontal_1>1100
Area=False
Saida=False

if(area_ao_lado):
    Area=True
    mapa_para_area_de_resgate['robot.turn']=[(-90 if posicao_inicial=="direita" else 90)]
    mapa_para_area_de_resgate['robot.straight']=[(catchesq_1 if posicao_inicial=="direita" else catchdir_1)]
    mapa_para_area_de_resgate['robot.turn']=[(-30 if posicao_inicial=="direita" else 30)]

elif(saida_ao_lado):
    Saida=True
    mapa_para_saida['robot.turn']=[(-90 if posicao_inicial=="direita" else 90)]
    mapa_para_saida['robot.straight']=[(catchesq_1 if posicao_inicial=="direita" else catchdir_1)]

elif(saida_a_frente):
    Saida=True
    mapa_para_saida['robot.straight']=[(catchfre_1)]

while(not Area):
    robot.straight(catchfre_1/2)
    catchfre_2 = ultra_frontal.distance()
    catchesq_2 = ultra_esquerda.distance()
    catchdir_2 = ultra_direita.distance()
    comprimento_horizontal_2 = catchesq_2+comprimento_robo+catchdir_2
    comprimento_horizontal = comprimento_horizontal_2 if comprimento_horizontal_1 != comprimento_horizontal_2 else comprimento_horizontal_1
    comprimento_horiz = comprimento_horizontal
    if(comprimento_horizontal_1!=comprimento_horizontal_2):
        mapa_para_area_de_resgate['robot.turn']=[(-90 if posicao_inicial=="direita" else 90)]
        mapa_para_area_de_resgate['robot.straight']=[(catchesq_1 if posicao_inicial=="direita" else catchdir_1)]
        mapa_para_area_de_resgate['robot.turn']=[(-30 if posicao_inicial=="direita" else 30)]
        Area = True

    robot.turn(angulo_para_girar_x_cm_na_parede(catchfre_2))
    catchfre_3 = ultra_frontal.distance()

    if(e_um_triangulo_retangulo(catchfre_3, catchfre_2, 150)):
        mapa_para_area_de_resgate['robot.straight']=[(catchfre_3)]
        mapa_para_area_de_resgate['robot.turn']=[(30 if posicao_inicial=="direita" else -30)]
        comprimento_vertical = catchfre_1+comprimento_vertical_robo
        Area = True
        if(not Saida): 
            mapa_para_saida['robot.turn']=[('calculo_do_angulo' if posicao_inicial=="direita" else -'calculo_do_angulo')]
            mapa_para_saida['robot.straight']=[('calculo da distancia até a saída, usando o comprimento da sala, etc...')]
    else:
        mapa_para_area_de_resgate['robot.turn']=[('calculo_do_angulo' if posicao_inicial=="direita" else -'calculo_do_angulo')]
        mapa_para_area_de_resgate['robot.straight']=[('calculo da distancia até a área, usando o comprimento da sala, etc...')]
        Area = True

ev3.screen.draw_text(40, 50, mapa_para_area_de_resgate)
wait(3000)
ev3.screen.clear()
ev3.screen.draw_text(40, 50, mapa_para_saida)
wait(3000)
ev3.screen.clear()
ev3.screen.draw_text(40, 50, comprimento_horiz)
wait(3000)
ev3.screen.clear()
ev3.screen.draw_text(40, 50, "FUNCIONOOOU")
wait(5000)