import sensor, image, time, math, tf, pyb

pin_one = pyb.Pin("P1", pyb.Pin.OUT_PP)
pin_two = pyb.Pin("P4", pyb.Pin.OUT_PP)
pin_three = pyb.Pin("P6", pyb.Pin.OUT_PP)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((400, 400))
sensor.skip_frames(time = 2000)
#uart = UART(3,9600, timeout_char = 1000)
#uart.init(9600, bits=8,parity=None,stop=1,timeout_char=1000)


clock = time.clock()

#blackB = (0, 0, -128, 127, -128, 127)
#blueCube = (0, 99, -128, 127, -128, -25)
#greenW = (65, 40, -128, -10, -128, 127)
#redW = (62, 26, 28, 127, -128, 127)

#min_confidence = 0.1
#labels, net = tf.load_builtin_model("fomo_face_detection")

while(True):
    clock.tick()
    img = sensor.snapshot()
    pin_one.high()
    pin_two.low()
    pin_three.low()

    #print("testing")
    #time.sleep(100);

    #img = sensor.snapshot()
    #dataToSend = {}


    ##Blue cube location
    #blueBalls = img.find_blobs([blueCube, blueCube], area_threshold=200, merge=True)
    #for blueBall in blueBalls:

        #img.draw_rectangle(blueBall.rect(), color=(0,255,0))
        #[x, y, w, h] = blueBall.rect()
        #center_x = math.floor(x + (w / 2))
        #center_y = math.floor(y + (h / 2))
        ##print(f"x {center_x}\ty {center_y}")
        #img.draw_circle((center_x, center_y, 12), color=(255,   0,   0), thickness=2)

        #dataToSend["blue"] = (center_x + 1000, center_y + 1000)
        ##img.draw_cross(blueBall.cx(), blueBall.cy(), color=(0,255,0))

    ##Black ball location
    #blackBalls = img.find_blobs([blackB, blackB], area_threshold=200, merge=True)
    #for blackBall in blackBalls:

        ##img.draw_rectangle(blackBall.rect(), color=(0,255,0))
        #img.draw_cross(blackBall.cx(), blackBall.cy(), color=(0,255,0))
        #[x, y, w, h] = blackBall.rect()
        #center_x = math.floor(x + (w / 2))
        #center_y = math.floor(y + (h / 2))

        #dataToSend["black"] = (center_x + 5000, center_y + 5000)

     ##Silver ball location
    #for i, detection_list in enumerate(net.detect(img, thresholds=[(math.ceil(min_confidence * 255), 255)])):
        #if (i == 0): continue # background class) == 0)   : continue
        #if (len(detection_list) == 0)   : continue

        #for d in detection_list:
            #[x, y, w, h] = d.rect()
            #center_x = math.floor(x + (w / 2))
            #center_y = math.floor(y + (h / 2))
            ##print(f"x {center_x}\ty {center_y}")
            #img.draw_circle((center_x, center_y, 12), color= (255, 0, 0), thickness=2)

            #dataToSend["silver"] = (center_x, center_y)



    print(clock.fps())
    #print(dataToSend)



    #for color, coordinates in dataToSend.items():
        #x = int(coordinates[0])
        #y = int(coordinates[1])
        #data = struct.pack('ii', x, y)
        #uart.write(data)

        #uart.write(str(coordinates[0]))
        #uart.write(str(coordinates[1]))
