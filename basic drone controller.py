import serial
import pygame

import winreg as reg
from itertools import count

from time import sleep

pygame.init()
clock=pygame.time.Clock()

thrust=0

running=True

key = reg.OpenKey(reg.HKEY_LOCAL_MACHINE, 'HARDWARE\\DEVICEMAP\\SERIALCOMM')
try:
    choice=None
    for i in count():
        device, port = reg.EnumValue(key, i)[:2]
        print("Device name \"%s\" found at %s" % (device, port))
        if 'BthModem2' in device:
            print("choosing %s"%device)
            choice=port
except: pass

try:
    if choice==None:
        ser=serial.Serial(input(),9600,timeout=0.025)
    else:
        ser=serial.Serial(choice,9600,timeout=0.025)
except serial.serialutil.SerialException as e:
    print(str(e))
    running=False
    input()

screen=pygame.display.set_mode((100,100))
font=pygame.font.Font(None,40)

shift=False
ctrl=False

changed=True
count=0

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
            elif event.key==pygame.K_c:
                thrust=256
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
    count+=1
    #if(ser.in_waiting):
    out=ser.readline()
    if out!=b'':
        print(out)
    if not count%100:
        ser.flushInput()
    pygame.display.update()
    #clock.tick(40)
try:
    ser.close()
except:
    pass
pygame.quit()
