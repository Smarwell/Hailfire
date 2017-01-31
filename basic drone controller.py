import serial
import pygame

import winreg as reg
from itertools import count

pygame.init()
clock=pygame.time.Clock()

thrust=0

running=True

key = reg.OpenKey(reg.HKEY_LOCAL_MACHINE, 'HARDWARE\\DEVICEMAP\\SERIALCOMM')
try:
    for i in count():
        device, port = reg.EnumValue(key, i)[:2]
        print("Device name \"%s\" found at %s" % (device, port))
except: pass

try:
    ser=serial.Serial(input(),9600,timeout=0.025)
except serial.serialutil.SerialException as e:
    print(str(e))
    running=False
    input()

screen=pygame.display.set_mode((100,100))
font=pygame.font.Font(None,40)

shift=False
ctrl=False

changed=True

while running:
    for event in pygame.event.get():
        if event.type==pygame.KEYDOWN:
            if event.key==pygame.K_LSHIFT:
                shift=True
            elif event.key==pygame.K_LCTRL:
                ctrl=True
            elif event.key==pygame.K_x:
                thrust=0
                changed=True
            elif event.key==pygame.K_z:
                thrust=255
                changed=True
        elif event.type==pygame.KEYUP:
            if event.key==pygame.K_LSHIFT:
                shift=False
            elif event.key==pygame.K_LCTRL:
                ctrl=False
        elif event.type==pygame.QUIT:
            running=False
    if ctrl and thrust>0:
        thrust-=1
        changed=True
    if shift and thrust<255:
        thrust+=1
        changed=True
    if changed:
        ser.write(str(thrust).encode("ascii"))
        screen.fill((0,0,0))
        screen.blit(font.render(str(thrust),True,(255,255,255)),(0,0))
        changed=False
    #if(ser.in_waiting):
    print(ser.read(2))
    pygame.display.update()
    clock.tick(20)
try:
    ser.close()
except:
    pass
pygame.quit()
