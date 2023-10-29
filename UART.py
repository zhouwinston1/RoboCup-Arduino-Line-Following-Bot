import sensor, image, time, math, tf, ustruct
from pyb import UART


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)

sensor.skip_frames(time = 2000)
uart = UART(3,9600, timeout_char = 1000)

clock = time.clock()

blackB = (0, 0, -128, 127, -128, 127)
blackB2 = ((16, 0, -128, 127, -128, 127))
blueCube = (0, 99, -128, 127, -128, -25)
greenW = (65, 40, -128, -10, -128, 127)
redW = (62, 26, 28, 127, -128, 127)

temp = 130
min_confidence = 0.01
labels, net = tf.load_builtin_model("fomo_face_detection")

while(True):
    clock.tick()
    img = sensor.snapshot()
    dataToSend = {}

    #Blue cube location
    blueBalls = img.find_blobs([blueCube, blueCube], area_threshold=200, merge=True)
    for blueBall in blueBalls:

        [x, y, w, h] = blueBall.rect()
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))
        #print(f"x {center_x}\ty {center_y}")
        if center_y > temp:
            img.draw_circle((center_x, center_y, 12), color=(255,   255,   0), thickness=2)
            dataToSend["blue"] = (center_x + 3000, center_y + 3000)
        #img.draw_cross(blueBall.cx(), blueBall.cy(), color=(0,255,0))

    #Black ball location
    blackBalls = img.find_blobs([blackB, blackB2], area_threshold=200, merge=True)
    for blackBall in blackBalls:

        #img.draw_rectangle(blackBall.rect(), color=(0,255,0))
        [x, y, w, h] = blackBall.rect()
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))
        if center_y > temp:

            img.draw_cross(blackBall.cx(), blackBall.cy(), color=(0,255,0))
            dataToSend["black"] = (center_x + 2000, center_y + 2000)

    redWall = img.find_blobs([redW, redW], area_threshold=200, merge=True)
    for red in redWall:

        #img.draw_rectangle(blackBall.rect(), color=(0,255,0))
        [x, y, w, h] = red.rect()
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))

        if center_y > temp:
            img.draw_cross(center_x, center_y, color=(0,255,0))
            dataToSend["red"] = (center_x + 5000, center_y + 5000)

    greenWall = img.find_blobs([greenW, greenW], area_threshold=200, merge=True)
    for green in greenWall:

        #img.draw_rectangle(blackBall.rect(), color=(0,255,0))
        [x, y, w, h] = green.rect()
        center_x = math.floor(x + (w / 2))
        center_y = math.floor(y + (h / 2))

        if center_y > temp:
            img.draw_cross(center_x, center_y, color=(0,255,0))
            dataToSend["green"] = (center_x + 6000, center_y + 6000)

     #Silver ball location
    for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
        if (i == 0): continue # background class) == 0)   : continue
        if (len(detection_list) == 0)   : continue

        for d in detection_list:
            [x, y, w, h] = d.rect()
            center_x = math.floor(x + (w / 2))
            center_y = math.floor(y + (h / 2))
            #print(f"x {center_x}\ty {center_y}")

            if center_y > 150:
                img.draw_circle((center_x, center_y, 12), color= (255, 0, 0), thickness=2)
                dataToSend["silver"] = (center_x + 1000, center_y + 1000)



    print(clock.fps())
    print(dataToSend)

    #num_balls = len(dataToSend)
    #uart.write(ustruct.pack("B", num_balls))
    #if not dataToSend:
        #num_balls = 1
        #uart.write(ustruct.pack("B", num_balls))
        #data = ustruct.pack("<hh", 1000, 1000)
        #uart.write(data)


    #else:
    num_balls = len(dataToSend)
    uart.write(ustruct.pack("B", num_balls))
    for color, coordinates in dataToSend.items():
        data = ustruct.pack("<hh", coordinates[0], coordinates[1])
        uart.write(data)

    time.sleep(0.1)
