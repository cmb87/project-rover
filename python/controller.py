import pygame
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer


pygame.init()
window = pygame.display.set_mode((300, 300))

clock = pygame.time.Clock()
font = pygame.font.SysFont('Comic Sans MS', 30)

rect = pygame.Rect(0, 0, 20, 20)
rect.center = window.get_rect().center
vel = 2

run = True

# =========================
class CommandStruct(object):
    pitch = 1500
    yaw = 1500
    roll = 1500
    throttle = 1500
    aux1 = 1500
    aux2 = 1500
    aux3 = 1500
    aux4 = 1500

class SensorStruct(object):
    sonar1 = 0
    sonar2 = 0
    sonar3 = 0
    heading = 0
    ax = 0

cmdStruct = CommandStruct
# =========================

try:

    link = txfer.SerialTransfer('COM5')
    link.open()
    sleep(5)
    print("starting")
        
    while run:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            if event.type == pygame.KEYDOWN:
            # print(pygame.key.name(event.key))
                pass

        keys = pygame.key.get_pressed()
        
        cmdStruct.yaw += (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * vel
        cmdStruct.throttle += (keys[pygame.K_DOWN] - keys[pygame.K_UP]) * vel
            

        sendSize = 0
        sendSize += link.tx_obj([1500,cmdStruct.yaw,1500,cmdStruct.throttle,1500,1500,1500,1500], start_pos=sendSize)

        link.send(sendSize)
    
        ###################################################################
        # Wait for a response and report any errors while receiving packets
        ##################################################################
        while not link.available():
            if link.status < 0:
                if link.status == txfer.CRC_ERROR:
                    print('ERROR: CRC_ERROR')
                elif link.status == txfer.PAYLOAD_ERROR:
                    print('ERROR: PAYLOAD_ERROR')
                elif link.status == txfer.STOP_BYTE_ERROR:
                    print('ERROR: STOP_BYTE_ERROR')
                else:
                    print('ERROR: {}'.format(link.status))


        recSize = 0

        SensorStruct.sonar1 = link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
        
        SensorStruct.sonar2 = link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']
        
        SensorStruct.sonar3 = link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

        SensorStruct.heading = link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']

        SensorStruct.ax = link.rx_obj(obj_type='f', start_pos=recSize)
        recSize += txfer.STRUCT_FORMAT_LENGTHS['f']


        print(f"{cmdStruct.throttle}, {cmdStruct.yaw}, {SensorStruct.sonar1}, {SensorStruct.sonar2}, {SensorStruct.sonar3}, {SensorStruct.heading}, {SensorStruct.ax}")



except KeyboardInterrupt:
    link.close()


pygame.quit()
exit()