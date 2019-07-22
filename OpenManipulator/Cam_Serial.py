#%%
import cv2, time, serial
import numpy as np

def OCM_D_Check() :
    _read = serOCM.read().decode('utf-8')
    while _read != 'D' :
        _read = serOCM.read().decode('utf-8')
    print(_read)
def keyInput() :
    _input = input()
    return _input
def OCM_Send(_msg) :
    serOCM.write(_msg.encode('utf-8'))
def OCM_Read() :
    _read = serOCM.read().decode('utf-8')
    return _read

serOCM = serial.Serial(port='COM15', baudrate=115200)
serOCM.close()
time.sleep(0.5)
serOCM.open()
time.sleep(0.5)

freq   = 10
frame  = np.empty([2, 2])
# res    = np.empty([2, 2])
red_x  = float()
red_y  = float()
blue_x = float()
blue_y = float()

x_c = 320
y_c = 240
conf_range = 40 # +- 20 범위 까지는 Center 정렬로 받아들임

noise_threshold = 50

red_flag  = 1
blue_flag = 1

done_blue = False
done_red  = False

# Camera object
cap       = cv2.VideoCapture(0)
# define range of red color in HSV
# SV 값을 조절하여 Noise 를 Filtering 할 수 있음
lower_red = np.array([0, 150, 150])
upper_red = np.array([10, 255, 255])
# define range of blue color in HSV
lower_blue = np.array([110, 150, 150])
upper_blue = np.array([130, 255, 255])

# try :
key = keyInput()
while key != "0" :
    key = keyInput()
OCM_Send(key)
OCM_D_Check()
while key != "2" :
    key = keyInput()
OCM_Send(key)
OCM_D_Check()
while key != "5" :
    key = keyInput()
OCM_Send(key)
OCM_D_Check()

if key == "5" :
    _, frame = cap.read()
    
    hsv_blue  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
    blue_res  = cv2.bitwise_and(frame, frame, mask=mask_blue)
    blue_res4Pix = np.transpose(np.nonzero(blue_res))

    blue_pixSumx = 0
    blue_pixSumy = 0
    """
    if not done_red :
        if done_blue :
            hsv_red  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask_red = cv2.inRange(hsv_red, lower_red, upper_red)
            red_res  = cv2.bitwise_and(frame, frame, mask=mask_red)

            red_res4Pix = np.transpose(np.nonzero(red_res))
            red_pixSumx  = 0
            red_pixSumy  = 0
            if len(red_res4Pix) > noise_threshold :
                for i in range(len(red_res4Pix)) :
                    red_pixSumx += red_res4Pix[i][1]
                    red_pixSumy += red_res4Pix[i][0]
                try :
                    red_x = red_pixSumx/len(red_res4Pix)
                    red_y = red_pixSumy/len(red_res4Pix)
                    print(red_x, red_y)
                    if x_c - conf_range < red_x < x_c + conf_range :
                        if y_c - conf_range < red_y < y_c + conf_range :
                            done_red = True
                            red_flag = 0
                            serOCM.write(str(red_flag).encode('utf-8'))
                        else :
                            red_flag = 9 # x 센터는 맞지만, y 센터 안 맞음
                            serOCM.write(str(red_flag).encode('utf-8'))
                    elif y_c - conf_range < red_y < y_c + conf_range:
                        red_flag = 8 # y 센터는 맞지만, x 센터 안 맞음
                        serOCM.write(str(red_flag).encode('utf-8'))
                    else :
                        red_flag = 7 # x, y 둘 다 안 맞음
                except ZeroDivisionError :
                    print("Find Red Again1") # 하나도 못 찾음
                    red_flag = 6
                    serOCM.write(str(red_flag).encode('utf-8'))
            else :
                print("Find Red Again0") # 찾았는데 개수가 모자름
                red_flag = 6
                serOCM.write(str(red_flag).encode('utf-8'))
    """
    if not done_blue :
        # print(len(blue_res4Pix))
        if len(blue_res4Pix) > noise_threshold :
            for i in range(len(blue_res4Pix)) :
                blue_pixSumx += blue_res4Pix[i][1]
                blue_pixSumy += blue_res4Pix[i][0]
            try :
                blue_x = blue_pixSumx/len(blue_res4Pix)
                blue_y = blue_pixSumy/len(blue_res4Pix)
                print(int(blue_x), int(blue_y))
                if x_c - conf_range < blue_x < x_c + conf_range :
                    if y_c - conf_range < blue_y < y_c + conf_range :
                        done_blue = True
                        blue_flag = 'z' # Perfect
                        # serOCM.write(str(blue_flag).encode('utf-8'))
                    else :
                        if y_c < 240 : 
                            blue_flag = 'x' # x 센터는 맞으나, y 센터는 안 맞음
                        elif y_c > 240 :
                            blue_flag = 'c'
                        # serOCM.write(str(blue_flag).encode('utf-8'))
                elif y_c - conf_range < blue_y < y_c + conf_range :
                    if x_c < 320 :
                        blue_flag = 'v'
                    elif x_c > 320 :
                        blue_flag = 'b' # y 센터는 맞으나, x 센터는 안 맞음
                    # serOCM.write(str(blue_flag).encode('utf-8'))
                else :
                    blue_flag = 'n' # 둘 다 안 맞음
                    # serOCM.write(str(blue_flag).encode('utf-8'))
            except ZeroDivisionError :
                print("Find Blue Again1") # 하나도 못 찾음
                blue_flag = 'm' # Not Found
                # serOCM.write(str(blue_flag).encode('utf-8'))
        else :
            print("Find Blue Again0") # 찾았는데 개수가 모자름
            blue_flag = 'm'
            # serOCM.write(str(blue_flag).encode('utf-8'))
    # cv2.imshow('blue img', blue_res)
    '''
    try :
        cv2.imshow('red img', red_res)
    except :
        pass
    '''
    # _key = cv2.waitKey(100)
    # if _key == 27 :
    #     cv2.destroyAllWindows()


while not done_blue :
    '''
    cap.release()
    cap = cv2.VideoCapture(0)
    '''
    print(done_blue, blue_flag)
    _, frame = cap.read()
    hsv_blue  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
    blue_res  = cv2.bitwise_and(frame, frame, mask=mask_blue)
    blue_res4Pix = np.transpose(np.nonzero(blue_res))

    blue_pixSumx = 0
    blue_pixSumy = 0
    serOCM.write('6'.encode('utf-8'))
    OCM_D_Check()
    # print(blue_flag)
    serOCM.reset_input_buffer()
    serOCM.write(str(blue_flag).encode('utf-8'))
    _read = serOCM.read().decode('utf-8')
    '''
    print(_read)
    while not _read :
        _read = serOCM.read().decode('utf-8')
    print(_read)
    '''
    # print(len(blue_res4Pix))
    if len(blue_res4Pix) > noise_threshold :
        for i in range(len(blue_res4Pix)) :
            blue_pixSumx += blue_res4Pix[i][1]
            blue_pixSumy += blue_res4Pix[i][0]
        try :
            blue_x = blue_pixSumx/len(blue_res4Pix)
            blue_y = blue_pixSumy/len(blue_res4Pix)
            print(int(blue_x), int(blue_y))
            if x_c - conf_range < blue_x < x_c + conf_range :
                if y_c - conf_range < blue_y < y_c + conf_range :
                    done_blue = True
                    blue_flag = 'z' # Perfect
                    # serOCM.write(str(blue_flag).encode('utf-8'))
                else :
                    if y_c < 240 : 
                        blue_flag = 'x' # x 센터는 맞으나, y 센터는 안 맞음
                    elif y_c > 240 :
                        blue_flag = 'c'
                    # serOCM.write(str(blue_flag).encode('utf-8'))
            elif y_c - conf_range < blue_y < y_c + conf_range :
                if x_c < 320 :
                    blue_flag = 'v'
                elif x_c > 320 :
                    blue_flag = 'b' # y 센터는 맞으나, x 센터는 안 맞음
                # serOCM.write(str(blue_flag).encode('utf-8'))
            else :
                blue_flag = 'n' # 둘 다 안 맞음
                # serOCM.write(str(blue_flag).encode('utf-8'))
        except ZeroDivisionError :
            print("Find Blue Again1") # 하나도 못 찾음
            blue_flag = 'm' # Not Found
            # serOCM.write(str(blue_flag).encode('utf-8'))
    else :
        print("Find Blue Again0") # 찾았는데 개수가 모자름
        blue_flag = 'm'
        # serOCM.write(str(blue_flag).encode('utf-8'))
    OCM_D_Check()
    cv2.imshow('blue img', blue_res)
    '''
    try :
        cv2.imshow('red img', red_res)
    except :
        pass
    '''
    _key = cv2.waitKey(50)
    if _key == 27 :
        cv2.destroyAllWindows()

if done_blue :
    print("HERE")
    serOCM.write('6'.encode('utf-8'))
    OCM_D_Check()
    serOCM.write(str(blue_flag).encode('utf-8'))
    OCM_D_Check()

cv2.destroyAllWindows()
cap.release()
serOCM.close()
