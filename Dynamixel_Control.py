#%%
# Control Panel
from PyQt5 import QtWidgets, QtGui, QtCore
from math  import sin, cos, radians, asin, degrees, sqrt
import os, serial, time, cv2, threading, socket
import numpy as np
# import skimage

# CV 관련 변수 선언
frame     = np.empty([2,2])
res       = np.empty([2,2])
red_X     = float()
red_Y     = float()
blue_X    = float()
blue_Y    = float()
red_flag  = 1
blue_flag = 1

# Target & Gripper 위치 검출 관련 변수 선언
GoalX_min = 500
GoalX_max = 580
GoalY_min = 200
GoalY_max = 280

Grp1_PosY_min = int()
Grp1_PosY_max = int()
Grp2_PosY_min = int()
Grp2_PosY_min = int()

class StoppableThread(threading.Thread):
    # Thread class with a stop() method. The thread itself has to check
    # regularly for the stopped() condition.
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()

def video_function_Thread() :
    global frame, res, red_X, red_Y, blue_X, blue_Y, red_flag, blue_flag
    try :
        cap = cv2.VideoCapture(0)
        # define range of blue color in HSV
        lower_red  = np.array([0,100,100])
        upper_red  = np.array([10,255,255])        
        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])
        blue_count = 0
        Sum_PosX   = 0
        Sum_PosY   = 0
        Goal_PosX  = 0
        Goal_PosY  = 0

        while not threading.current_thread().stopped() :
            print(blue_count)
            # Take each frame
            time.sleep(0.025)
            _, frame    = cap.read()
            counted_pix = 0

            # Convert BGR to HSV
            hsv_red  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # Threshold the HSV image to get only blue colors
            mask_red = cv2.inRange(hsv_red, lower_red, upper_red)
            if blue_count < 50 :
                hsv_blue  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
                blue_res  = cv2.bitwise_and(frame,frame, mask=mask_blue) # 파란색 마스크로 파란색 위치 찾음
                blue_res4Pix   = np.transpose(np.nonzero(blue_res)) # 파란색 위치 행렬
                blue_pixelSumX = 0 # 파란색 X 방향 픽셀 총합
                blue_pixelSumY = 0 # 파란색 Y 방향 픽셀 총합
            elif Goal_PosX == 0 :
                Goal_PosX = round(Sum_PosX/50)
                Goal_PosY = round(Sum_PosY/50)
                blue_flag = 1
            if Goal_PosX != 0 :
                cv2.circle(frame, (Goal_PosX, Goal_PosY),
                           15, (0,0,255), -1)
                print("PASS?")
            try :
                mask = mask_red + mask_blue
            except :
                mask = mask_red

            # Bitwise-AND mask and original image
            res     = cv2.bitwise_and(frame,frame, mask=mask)
            red_res = cv2.bitwise_and(frame,frame, mask=mask_red)  # 빨간색 마스크로 빨간색 위치 찾음

            red_res4Pix   = np.transpose(np.nonzero(red_res))  # 빨간색 위치 행렬
            red_pixelSumX = 0 # 빨간색 X 방향 픽셀 총합
            red_pixelSumY = 0 # 빨간색 Y 방향 픽셀 총합            

            # 빨간색 픽셀 위치 평균값 찾기
            for i in range(len(red_res4Pix)) :
                red_pixelSumX += red_res4Pix[i][1]
                red_pixelSumY += red_res4Pix[i][0]
            try :
                red_X = red_pixelSumX/len(red_res4Pix)
                red_Y = red_pixelSumY/len(red_res4Pix)
                # print("Red Pos : %0.1f" %(red_pixelPos))
            except ZeroDivisionError :
                # red 색상 검출이 안 됨
                red_X    = 999
                red_Y    = 999
                red_flag = 999
            # 파란색 픽셀 위치 평균값 찾기
            if blue_count != 50 :
                for i in range(len(blue_res4Pix)) :
                    if GoalY_min < blue_res4Pix[i][0] < GoalY_max :
                        if GoalX_min < blue_res4Pix[i][1] < GoalX_max :
                            blue_pixelSumX += blue_res4Pix[i][1]
                            blue_pixelSumY += blue_res4Pix[i][0]
                            counted_pix += 1
                try :
                    blue_X      = round(blue_pixelSumX/counted_pix, 2)
                    blue_Y      = round(blue_pixelSumY/counted_pix, 2)
                    blue_flag   = 1 # Blue Flag
                    blue_count += 1
                    Sum_PosX  += blue_X
                    Sum_PosY  += blue_Y
                except ZeroDivisionError :
                    # blue 색상 검출 안 됨
                    blue_X      = 9999
                    blue_Y      = 9999
                    blue_flag   = 9999
                    blue_count -= 1
                    Sum_PosX  -= blue_X
                    Sum_PosY  -= blue_Y
            # 이미지 필터
            # dst = cv2.fastNlMeansDenoisingColored(res,None,10,10,7,21)
    except :
        pass

pos6d_send = list()
def URComm_SendPos(_sock) :
    """
        UR3 로 다음 포지션 위치를 전달하는 Thread
    """
    global pos6d_send

pos6d_recv = list()
def URComm_RecvPos(_sock) :
    """
        UR3 로부터 포지션 정보를 받는 Thread
    """
    global pos6d_recv

HOME = True
if HOME :
    wdir = '/Users/shetshield/Desktop/SMF'
    COM  = 'COM5'
else :
    wdir = '/Users/user/Desktop/SMF'
    COM  = 'COM6'

os.sys.path.append(wdir)
import SMF_2nd_UI_R4
from grid_world import negative_grid, standard_grid

class XDialog(QtWidgets.QDialog, SMF_2nd_UI_R4.Ui_Dialog) :
    def __init__(self):
        QtWidgets.QDialog.__init__(self)
        # setupUi() method shows dialog in display
        self.setupUi(self)
        """
        각 버튼 클릭 및 텍스트 필드 엔터 키 입력되었을 때 연결되는 기능 정의
        """
        self.CurPos_Button.clicked.connect(self.CurPos_onClicked)          # 현재 위치 표시
        self.pushButton.clicked.connect(self.pushButton_onClicked)         # 그리퍼 위치 초기화
        self.PatSize_Button.clicked.connect(self.PatSize_Button_onClicked) # 패턴 사이즈 입력
        self.NumExec_Button.clicked.connect(self.NumExec_Button_onClicked) # 시행 횟수 설정
        self.RunTask_Button.clicked.connect(self.RunTask_Button_onClicked) # TASK 실행
        self.NumExec.returnPressed.connect(self.NumExec_onPressed)         # 시행 횟수 입력창
        self.PatSize.returnPressed.connect(self.PatSize_onPressed)         # 패턴 사이즈 입력창
        self.Start_Button.clicked.connect(self.Start_Button_onClicked)     # 시작 버
        self.Stop_Button.clicked.connect(self.Stop_Button_onClicked)       # 정지 버튼

        # 기타 변수 선언
        self.cmd               = str()    # Serial 통신을 통하여 아두이노로 전달되는 명령어
        self.X_default         = 50.7     # X 디폴트 값
        self.Y_default         = 104.0    # Y 디폴트 값
        self.NumOfExec         = int()    # 몇 번 반복해서 실행할지를 나타내는 값
        self.PatternSize       = float()  # 입력되는 패턴 사이즈 저장
        self.PatSize_Low_Limit = 50       # 입력되는 패턴 사이즈는 왼쪽보다 커야함
        self.PatSize_Up_Limit  = 258      # 입력되는 패턴 사이즈는 왼쪽보다 작아야함
        self.recv_msg          = str()    # 아두이노로부터 받는 데이터
        self.en                = 'utf-8'  # encoding 방식 정의
        self.COM               = COM      # 통신 포트 설정
        self.BAUD              = 115200   # Baudrate 설정
        self.DXL_1             = int()    # Dynamixel Motor 1
        self.DXL_2             = int()    # Dynamixel Motor 2
        self.X_Pos             = float()  # 패턴 크기
        self.Y_Pos             = float()  # Y 방향으로의 변위
        self.X_InitLen         = 50.7     # 패턴 최소 크기
        self.Y_InitLen         = 104.0    # Y 방향 초기 길이
        self.Deg_per_Pos       = 0.088    # 0.088 deg / 1 값
        self.X_Delta           = float()  # X 방향 변화값
        self.Y_Delta           = float()  # Y 방향 변화값
        self.send_dxl1         = int()    # 다이나믹셀 1 골 포지션 값
        self.send_dxl2         = int()    # 다이나믹셀 2 골 포지션 값
        self.send_msg          = str()    # 아두이노로 보내는 메세지
        self.Noise_Collect     = 50       # Noise 를 많이 타기 때문에, 50개 데이터를 수집함
        self.Noise_Threshold   = 15       # Target 색상의 픽셀 위치 변화 정도 체크
                                          # Target 색상은 위치가 고정되어 움직이지 않음
        self.HOST       = '192.168.0.100' # Server IP
        self.PORT       = 30000           # Server Port
        self.tmp        = float()         # UR3 Current Position Temporary Value
        self.idx        = 1               # Position 값 시작 Index
        self.pos6d_recv = list()          # 전달 받은 UR3 포지션
        self.URCmd      = str()           # UR3 로 전달하는 명령
        self._flag      = False           # 동작 완료 여부 판단

        self.GoalX_min = 500              # Noise 를 줄이기 위하여 Goal 포지션 위치를 제한함
        self.GoalX_max = 580              # 위와 동일(In X 방향)
        self.GoalY_min = 200              # 위와 동일(In Y 방향)
        self.GoalY_max = 280              # 위와 동일(In Y 방향)
        
        """
        RL 변수
        """
        self.SMALL_ENOUGH = 1e-3 # 종료
        self.GAMMA = 0.9         # Discount Factor
        self.ALL_POSSIBLE_ACTIONS = ('U', 'D', 'L', 'R') # 가능한 Action set

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 윈도우 X Button 클릭 시
        finish = QtWidgets.QAction("Quit", self)
        finish.triggered.connect(self.closeEvent)
        # Video Stream object 생성
        self.cap = cv2.VideoCapture(0)

        try :
            self.serArdu = serial.Serial(port=self.COM, baudrate=self.BAUD)
            self.serArdu.close()
            time.sleep(1)
            self.serArdu.open()
            time.sleep(1)
        except :
            print("Port Not Connected")

        # 웹캠과 통신
        # self.video_thread = StoppableThread(target=video_function)
        # self.video_thread.start()

    def closeEvent(self, event):
        close = QtWidgets.QMessageBox.question(self,
                                     "프로그램 종료",
                                     "ㄹㅇ?",
                                      QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        if close == QtWidgets.QMessageBox.Yes:
            import sys
            event.accept()
            try : 
                self.serArdu.close()
            except :
                print("Port Already Closed")
            try :
                self.video_thread.stop()
            except :
                print("Video Thread already Stopped")
            try :
                app.quit()
                print("ui quit")
            except :
                pass
            os._exit(00)
            sys.exit(0)
        else:
            event.ignore()

    def Img_Show(self) :
        """
        컬러 필터링한 결과를 Label 에 출력
        """
        frameQt = QtGui.QImage(self.frame.data, self.frame.shape[1], self.frame.shape[0],
                               QtGui.QImage.Format_RGB888)
        resQt   = QtGui.QImage(self.res.data, self.res.shape[1], self.res.shape[0],
                               QtGui.QImage.Format_RGB888)
        frameQt = QtGui.QPixmap.fromImage(frameQt)
        resQt   = QtGui.QPixmap.fromImage(resQt)
        """
        frameSki = skimage.color.hsv2rgb(frame)
        resSki   = skimage.color.hsv2rgb(res)

        frameQt = QtGui.QImage(frameSki.data, frameSki.shape[1], frameSki.shape[0],
                               QtGui.QImage.Format_RGB888)
        resQt   = QtGui.QImage(resSki.data, resSki.shape[1], resSki.shape[0],
                               QtGui.QImage.Format_RGB888)
        frameQt = QtGui.QPixmap.fromImage(frameQt)
        resQt   = QtGui.QPixmap.fromImage(resQt)
        """
        framePixmap = QtGui.QPixmap(frameQt)
        resPixmap   = QtGui.QPixmap(resQt)

        resizedFrame = framePixmap.scaled(self.Orig_Img.frameGeometry().width(),
                                          self.Orig_Img.frameGeometry().height(),
                                          QtCore.Qt.KeepAspectRatio)
        resizedRes   = resPixmap.scaled(self.Target_Img.frameGeometry().width(),
                                        self.Target_Img.frameGeometry().height(),
                                        QtCore.Qt.KeepAspectRatio)
        # 여기까지, Capture 이미지를 Qt 포맷으로 바꾸는 코드
        app.processEvents()
        self.Orig_Img.setPixmap(resizedFrame)
        self.Target_Img.setPixmap(resizedRes)
        # self.show()

    def Pixel_Displacement(self, _red, _blue) :
        """
        컬러간 픽셀 거리 차이를 LCD 에 띄워주는 기능
        컬러 검출 여부에 따라 각기 다른 _flag 를,
        계산된 delta X, Y 와 target position 을 리턴
        """
        _goalPos = list()
        if self._goalPos != 50 :
            if _blue[2] != 9999 :
                # Blue 색상이 정상적으로 인식 되었을 때에만 값을 추가함
                _goalPos.append((_blue[0], _blue[1]))
        else :
            _goalPos = [self.goal_X, self.goal_Y]
        if _red[2] == 999 :
            if _blue[2] == 9999 :
                _flag = -1 # red & blue 둘 다 없음
            else :
                _flag = -2 # red 만 없음
        else :
            if _blue[2] == 9999 :
                _flag = -3 # blue 만 없음
            else :
                _flag = 0
                _delta_X = round(abs(_red[0] - _blue[0])) # 제대로 포착됨
                _delta_Y = round(abs(_red[1] - _blue[1]))
                _delta = (_delta_X, _delta_Y)

        return _flag, _delta, _goalPos

    def video_function(self) :
        try :
            blue_pixelSumX = 0 # 파란색 X 방향 픽셀 총합
            blue_pixelSumY = 0 # 파란색 Y 방향 픽셀 총합
            # cap = cv2.VideoCapture(0)
            # define range of blue color in HSV
            lower_red  = np.array([0,100,100])
            upper_red  = np.array([10,255,255])        
            # define range of blue color in HSV
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])

            time.sleep(0.01)
            _, self.frame = self.cap.read()
            counted_pix   = 0

            hsv_red  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
            mask_red = cv2.inRange(hsv_red, lower_red, upper_red)

            if self._count != 50 :
                hsv_blue  = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
                mask_blue = cv2.inRange(hsv_blue, lower_blue, upper_blue)
                blue_res  = cv2.bitwise_and(self.frame,self.frame, mask=mask_blue) # 파란색 마스크로 파란색 위치 찾음
                blue_res4Pix = np.transpose(np.nonzero(blue_res)) # 파란색 위치 행렬
            else :
                cv2.circle(self.frame, (int(self.goal_X), int(self.goal_Y)), 
                           15, (255,0,0), -1)
            try :
                mask = mask_red + mask_blue
            except :
                mask = mask_red

            self.res = cv2.bitwise_and(self.frame,self.frame, mask=mask)
            red_res  = cv2.bitwise_and(self.frame,self.frame, mask=mask_red)  # 빨간색 마스크로 빨간색 위치 찾음
            if self._count == 50 :
                cv2.circle(self.frame, (int(self.goal_X), int(self.goal_Y)),
                           20, (255,0,0), -1)
                cv2.circle(self.res, (int(self.goal_X), int(self.goal_Y)),
                           20, (255,0,0), -1)
            red_res4Pix   = np.transpose(np.nonzero(red_res))  # 빨간색 위치 행렬
            red_pixelSumX = 0 # 빨간색 X 방향 픽셀 총합
            red_pixelSumY = 0 # 빨간색 Y 방향 픽셀 총합    
            # 빨간색 픽셀 위치 평균값 찾기
            for i in range(len(red_res4Pix)) :
                red_pixelSumX += red_res4Pix[i][1]
                red_pixelSumY += red_res4Pix[i][0]
            try :
                red_X    = round(red_pixelSumX/len(red_res4Pix), 2)
                red_Y    = round(red_pixelSumY/len(red_res4Pix), 2)
                red_flag = 1
                # print("Red Pos : %0.1f" %(red_pixelPos))
            except ZeroDivisionError :
                # red 색상 검출이 안 됨
                print("Red ZeroDivision")
                red_X    = 999
                red_Y    = 999
                red_flag = 999
            # 파란색 픽셀 위치 평균값 찾기
            if self._count != 50 :
                for i in range(len(blue_res4Pix)) :
                    if self.GoalY_min < blue_res4Pix[i][0] < self.GoalY_max :
                        if self.GoalX_min < blue_res4Pix[i][1] < self.GoalX_max :
                            blue_pixelSumX += blue_res4Pix[i][1]
                            blue_pixelSumY += blue_res4Pix[i][0]
                            counted_pix += 1
                try :
                    blue_X       = round(blue_pixelSumX/counted_pix, 2)
                    blue_Y       = round(blue_pixelSumY/counted_pix, 2)
                    blue_flag    = 1 # Blue Flag
                except ZeroDivisionError :
                    # blue 색상 검출 안 됨
                    print("Blue ZeroDivision")
                    blue_X       = 9999
                    blue_Y       = 9999
                    blue_flag    = 9999
            else :
                blue_X    = self.goal_X
                blue_Y    = self.goal_Y
                blue_flag = 1
            return (blue_X, blue_Y, blue_flag), (red_X, red_Y, red_flag)
        except :
            return (999, 999, 999), (99, 99, 99)

    def Start_Button_onClicked(self) :
        """
        웹캠으로부터 데이터를 읽어오는 기능
        그리퍼 위치와 타겟 위치의 픽셀 거리차를 표시해주는 기능
        """
        # self.video_thread.start()
        self._goalPos   = list()
        self._error     = [-1, -2]
        self._noiseFilt = False
        self.goal_X = 0.0
        self.goal_Y = 0.0
        self._count = 0
        while True :
            try :
                _blue, _red = self.video_function()
                self.Img_Show()
                if len(self._goalPos) != self.Noise_Collect :    
                    _flag, _delta, _goalPos = self.Pixel_Displacement(_blue, _red)
                    if _blue[2] != 9999 :
                        self._goalPos.append(_goalPos)
                        self.goal_X += _blue[0]
                        self.goal_Y += _blue[1]
                        self._count += 1
                else :
                    if self._noiseFilt == False :
                        self._noiseFilt = True
                        self.goal_X = round(self.goal_X/self.Noise_Collect)
                        self.goal_Y = round(self.goal_Y/self.Noise_Collect)
                if self._noiseFilt == True :
                    _flag, _delta, _goalPos = self.Pixel_Displacement(_blue, _red)
                    if _flag not in self._error :
                        app.processEvents()
                        _distance = round(sqrt(_delta[0]**2 + _delta[1]**2))
                        self.Delta_LCD.display(_distance)
                    else :
                        app.processEvents()
                        self.Delta_LCD.display(float(_flag))
            except :
                self.cap.release()

    def Stop_Button_onClicked(self) :
        """
        Stop 버튼이 클릭 되었을 때 모두 정지함
        """
        try : 
            self.serArdu.close()
        except :
            print("Port Already Closed")
        try :
            self.video_thread.stop()
        except :
            print("Video Thread already Stopped")
        try :
            app.quit()
            print("ui quit")
        except :
            pass

    def CurPos_onClicked(self) :
        """
        현재 위치를 알려주는 기능 (LCD) 에다가 알려줌
        * 아두이노 Task
        * 각 모터의 현재 포지션 계산

        포지션을 Serial 통신으로 PC 에 전달
        command string "1"
        """
        self.cmd = "1"
        self.serArdu.write(self.cmd.encode(self.en))
        try :
            self.serArdu.write(self.cmd.encode(self.en))
            self.serArdu.reset_input_buffer()
        except :
            print("cmd-1 s1 Communication Error")
            self.serArdu.close()
        try :
            while self.cmd == "1" :
                self.recv_data = self.serArdu.readline().decode(self.en).rstrip('\n\r')
                for i in range(len(self.recv_data)) :
                    if self.recv_data[i] == ',' :
                        self.DXL_1 = int(self.recv_data[:i])
                        self.DXL_2 = int(self.recv_data[i+1:])
                self.X_Delta = self.Y_InitLen * (abs(sin(radians(self.DXL_1*self.Deg_per_Pos))) +
                                                     abs(sin(radians(self.DXL_2*self.Deg_per_Pos))))
                self.X_Pos = self.X_InitLen + self.X_Delta
                self.Y_Pos = 0.5*(self.Y_InitLen * (cos(radians(self.DXL_1*self.Deg_per_Pos)) +
                                  cos(radians(self.DXL_2*self.Deg_per_Pos))))
                app.processEvents()
                self.X_LCD.setProperty("value", round(self.X_Pos, 1))
                self.Y_LCD.setProperty("value", round(self.Y_Pos, 1))
        except :
            self.serArdu.close()
            print("cmd-1 s2 Communication Error")

    def pushButton_onClicked(self) :
        """
        그리퍼 위치 초기화 완료 신호를 아두이노로 전달
        * 아두이노 Task
        * 모터 토크 온 O
        * 그리퍼를 0 위치로 이동시킴 X
        * 시리얼 통신을 통하여 완료 메세지를 전달 O
        command string "2"
        """
        self.cmd = "2"
        try :
            # 두 번 전달함
            for i in range(2) :
                self.serArdu.write(self.cmd.encode(self.en))
                self.serArdu.reset_input_buffer()
        except :
            print("cmd-2 s1 Communication Error")
            self.serArdu.close()

        """
        아두이노로부터 완료 메세지를 받으면, 완료 메세지를 박스에 띄움
        """
        try :
            self.recv_data = self.serArdu.readline().decode(self.en).rstrip('\n\r')
            # print(self.recv_data)
            while self.recv_data != 'DONE' :
                app.processEvents()
                self.Done_Label.setText("...진행중")
                self.recv_data = self.serArdu.readline().decode(self.en).rstrip('\n\r')
                # print(self.recv_data)
        except :
            print("cmd-2 s2 Communication Error")
            self.serArdu.close()

        app.processEvents()
        self.X_LCD.setProperty("value", round(self.X_Pos, 1))
        self.Y_LCD.setProperty("value", round(self.Y_Pos, 1))
        """
        self.X_LCD.display(round(self.X_Pos, 1))
        self.Y_LCD.display(round(self.Y_Pos, 1))
        self.X_LCD.repaint()
        self.Y_LCD.repaint()
        """
        app.processEvents()
        self.Done_Label.setText("완료")
        # Done Label 의 Font 속성 정의
        app.processEvents()
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.Done_Label.setFont(font)
        self.Done_Label.setAutoFillBackground(False)
        self.Done_Label.setAlignment(QtCore.Qt.AlignCenter)
        app.processEvents()
        self.Done_Label.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color: rgb(0, 0, 255, 255);")
        self.pushButton.setEnabled(False)

    def PatSize_Button_onClicked(self) :
        """
        패턴 사이즈를 입력 가능하게 하는 버튼
        버튼 클릭시, 텍스트 필드 활성화
        command string "3"
        """
        self.cmd = "3"
        app.processEvents()
        self.PatSize.setEnabled(True) # 텍스트 입력창 활성화
        self.PatSize.clear()
        self.PatSize.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color:rgb(0, 0, 0);")

    def NumExec_Button_onClicked(self) :
        """
        Task 반복 횟수 입력 가능하게 하는 버튼
        버튼 클릭시, 텍스트 필드 활성화
        command string "4"
        """
        self.cmd = "4"
        app.processEvents()
        self.NumExec.setEnabled(True) # 텍스트 입력창 활성화
        self.NumExec.clear()
        self.NumExec.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color:rgb(0, 0, 0);")

    def RunTask_Button_onClicked(self) :
        """
        Task 반복 실행시킴
        * UR3 와 통신
        * NumExec 횟수를 UR3 에 전달
        * 그리퍼의 X, Y 좌표를 UR3 에 전달
        * Task 완료 시, UR3 로부터 완료 메세지 받음
        * 초기 위치로 이동(X, Y Default)
        * 아두이노 Task
        * Default 위치로 이동
        * Torque Enabled 해제
        command string "5"
        """
        self.cmd = "5"
        self.serArdu.write(self.cmd.encode(self.en))
        try :
            self.serArdu.write(self.cmd.encode(self.en))
        except :
            print("cmd-5 s1 Communication Error")
            self.serArdu.close()
        # UR3 와 통신하는 소켓을 연다
        self.sock.bind((self.HOST, self.PORT))
        self.sock.listen(5)
        self.conn, _ = self.sock.accept()
        self.msg = self.conn.recv(1024).decode(en)
        while  self.msg != 'COMCHECK' :
            self.msg = self.conn.recv(1024).decode(en)
            # time.sleep(0.005)
        self.conn.send("CONNECTED".encode(en))
        """
        이 단계에서 현재 Position 을 Variable 에 저장(UR3)
        """
        """
        self.msg = self.conn.recv(1024).decode(en)
        for i in range(len(self.msg)) :
            if self.msg[i] == ',' :
                self.tmp = float(self.msg[self.idx:i])
                self.pos.append(self.tmp)
                self.idx = i+1
            elif self.msg[i] == ']' :
                self.tmp = float(self.msg[self.idx:i])
                self.pos.append(self.tmp)
        print(self.pos)
        """
        """
            RL 을 위한 그리드 생성 및 처리
        """
        _blue, _red = self.video_function()
        self.grid = negative_grid((int(_red[0]), int(_red[1])),(self.goal_X, self.goal_Y))
        policy = {}
        for s in self.grid.actions.keys():
            policy[s] = np.random.choice(self.ALL_POSSIBLE_ACTIONS)

        V = {}
        states = self.grid.all_states()
        for s in states:
            # V[s] = 0
            if s in self.grid.actions:
                V[s] = np.random.random()
            else:
                # terminal state
                V[s] = 0

        # repeat until convergence - will break out when policy does not change
        while True:
            # policy evaluation step - we already know how to do this!
            while True:
                biggest_change = 0
                for s in states:
                    old_v = V[s]
                    # V(s) only has value if it's not a terminal state
                    if s in policy:
                        self.conn.send("START".encode(en))
                        a = policy[s]
                        """
                        UR3 로 현재 Action 전달
                        """
                        self.conn.send(str(a).encode(en))
                        """
                        UR3 로부터 이동 완료 메세지 수신
                        """
                        self.msg = self.conn.recv(1024).decode(en)
                        while self.msg != "COMPLETE" :
                            self.msg = self.conn.recv(1024).decode(en)
                        _blue, _red = self.video_function()
                        # Grid update
                        self.grid.set_state(s)
                        r = self.grid.move(a, (int(_red[0]), int(_red[1]))) # Reward
                        V[s] = r + self.GAMMA * V[self.grid.current_state()]
                        biggest_change = max(biggest_change, np.abs(old_v - V[s]))
                        self.conn.send("STEP_DONE".encode(en))
                if biggest_change < self.SMALL_ENOUGH:
                    break
            # policy improvement step
            is_policy_converged = True
            for s in states:
                if s in policy:
                    old_a = policy[s]
                    new_a = None
                    best_value = float('-inf')
                    # loop through all possible actions to find the best current action
                    for a in self.ALL_POSSIBLE_ACTIONS:
                        """
                        UR3 로 현재 Action 전달
                        """
                        self.conn.send(str(a).encode(en))
                        """
                        UR3 로부터 이동 완료 메세지 수신
                        """
                        self.msg = self.conn.recv(1024).decode(en)
                        while self.msg != "COMPLETE" :
                            self.msg = self.conn.recv(1024).decode(en)
                        _blue, _red = self.video_function()
                        self.grid.set_state(s)
                        r = self.grid.move(a, (int(_red[0]), int(_red[1])))
                        v = r + self.GAMMA * V[self.grid.current_state()]
                        if v > best_value:
                            best_value = v
                            new_a = a
                            policy[s] = new_a
                if new_a != old_a:
                    is_policy_converged = False
            if is_policy_converged:
                break

        app.processEvents()
        # Task 실행 시 모든 버튼 및 입력 Field 를 비활성화
        self.pushButton.setEnabled(False)
        self.PatSize_Button.setEnabled(False)
        self.NumExec_Button.setEnabled(False)
        # self.RunTask_Button.setChecked(True)
        self.RunTask_Button.setEnabled(False)
        app.processEvents()
        self.label_4.setText("...진행중")
        self.label_4.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color: rgb(255, 0, 0);")
        self.msg = self.conn.recv(1024).decode(en)
        try :
            while self._flag != True :
                self.msg = self.conn.recv(1024).decode(en)
                for i in range(len(self.msg)) :
                    if self.msg[i] == ',' :
                        self.tmp = float(self.msg[self.idx:i])
                        self.pos6d_recv.append(self.tmp)
                        self.idx = i+1
                    elif self.msg[i] == ']' :
                        self.tmp = float(self.msg[self.idx:i])
                        self.pos6d_recv.append(self.tmp)
                print(self.pos6d_recv)
        except :
            print("Error Occured. Disconnect & Close the socket")
            self.conn.close()
            self.sock.close()
        """
        # socket 통신 하는 코드 작성
        self.NumOfExec 를 전달
        self.PatternSize 를 전달
        """
        # app.processEvents()
        # UR3 로부터 완료 메세지를 받고난 이후에 아래 명령문 실행
        self.label_4.setText("완료")
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAutoFillBackground(True)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color: rgb(0, 0, 255);")
        try :
            self.serArdu.write(self.cmd.encode(self.en))
        except :
            print("cmd-5 s2 Communication Error")
            self.serArdu.close()

        self.CurPos_Button.setEnabled(True) # 포지션 확인 다시 가능하도록 만듦
        self.pushButton.setEnabled(True)
        self.PatSize_Button.setEnabled(True)
        self.NumExec_Button.setEnabled(True)
        self.RunTask_Button.setChecked(False)
        self.RunTask_Button.setEnabled(True)

        app.processEvents()
        self.NumOfExec   = int()    # 반복횟수 초기화
        self.PatternSize = float()  # 패턴 크기 초기화
        self.label_4.setText("")    # 완료 메세지 초기화
        self.Done_Label.setText("") # 완료 메세지 초기화
        self.X_LCD.setProperty("value", 0)
        self.Y_LCD.setProperty("value", 0)

    def NumExec_onPressed(self) :
        """
        텍스트 필드에 값이 입력되면 그 값을 저장함
        command string "6"
        """
        self.cmd = "6"
        value = self.NumExec.text()
        self.NumOfExec = int(value) # String 으로 저장되는 입력값을 정수로 변환하여 저장
        self.NumExec.setText("%d 번" %(self.NumOfExec))
        """
        font = QtGui.QFont()
        font.setPointSize(18)
        font.setBold(True)
        font.setWeight(75)
        self.NumExec.setFont(font)
        """
        app.processEvents()
        self.NumExec.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color: rgb(0, 0, 255);")
        self.NumExec.setEnabled(False) # 텍스트 입력창 비활성화

    def PatSize_onPressed(self) :
        """
        텍스트 필드에 값이 입력되면 그 값을 저장함
        command string "7"
        """
        self.cmd = "7"
        try :
            self.serArdu.write(self.cmd.encode(self.en))
        except :
            print("cmd-5 s1 communication Error")
            self.serArdu.close()
        self.PatternSize = float(self.PatSize.text()) # Sting 으로 저장된는 입력 값을 float 타입으로 저장
        self.PatSize.setEnabled(False) # 텍스트 입력창 비활성화
        if self.PatternSize < self.PatSize_Low_Limit :
            self.PatterSize = 0
            app.processEvents()
            self.PatSize.setText("범위 미달")
            self.PatSize.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color:rgb(255, 0, 0);")

            self.serArdu.write("INVALID".encode(self.en))
        elif self.PatternSize > self.PatSize_Up_Limit :
            self.PatternSize = 0
            app.processEvents()
            self.PatSize.setText("범위 초과")
            self.PatSize.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color:rgb(255, 0, 0);")

            self.serArdu.write("INVALID".encode(self.en))
        else :
            app.processEvents()
            self.PatSize.setText("%0.1f mm" %(round(self.PatternSize, 1)))
            """
            font = QtGui.QFont()
            font.setPointSize(18)
            font.setBold(True)
            font.setWeight(75)
            self.PatSize.setFont(font)
            """
            self.PatSize.setStyleSheet("background-color: rgb(255, 255, 255, 255);\n" "color: rgb(0, 0, 255);")
            self.PatternSize = round(self.PatternSize, 1)
            self.send_dxl1, self.send_dxl2 = self.Inverse_Calc()
            # self.send_msg = str(abs(self.dxl1))
            self.send_msg = str(self.dxl1) + " " + str(self.dxl2)
            try :
                self.serArdu.write(self.send_msg.encode(self.en))
            except :
                print("cmd-5 s2 communication Error")
                self.serArdu.close()

    def Inverse_Calc(self) :
        _dx   = 0.5*(self.PatternSize - self.X_InitLen)
        _dxl1 = -int(degrees(asin(_dx/self.Y_InitLen))/self.Deg_per_Pos) # 마이너스
        _dxl2 = int(degrees(asin(_dx/self.Y_InitLen))/self.Deg_per_Pos)  # 플러스
        self.dxl1 = _dxl1
        self.dxl2 = _dxl2
        print(self.dxl1, self.dxl2)
        return self.dxl1, self.dxl2

    def URComm_Send(self) :
        pass
    
    def URComm_Recv(self) :
        pass

app    = QtWidgets.QApplication([])
dialog = XDialog()
dialog.show()
app.exec_()

#%%
import socket, time

def main() :        
    HOST = "192.168.0.100"
    PORT = 30000
    en   = 'utf-8'
    print("Starting Program")
    idx = 1
    pos = list()
    tmp = float()
    try : 
        # Socket Object 생성 및 Server 연결
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((HOST, PORT))
        sock.listen(5)
        conn, addr = sock.accept()
        # 정상적으로 통신이 되고 있는지 확인
        msg = conn.recv(1024).decode(en)
        try :
            while  msg != 'COMCHECK' :
                msg = conn.recv(1024).decode(en)
            # time.sleep(0.005)
        except :
            print("NOT Connected to UR3")
        conn.send("DONE".encode(en))
        while True :
            msg = conn.recv(1024).decode(en)
            for i in range(len(msg)) :
                if msg[i] == ',' :
                    tmp = float(msg[idx:i])
                    pos.append(tmp)
                    idx = i+1
                elif msg[i] == ']' :
                    tmp = float(msg[idx:i])
                    pos.append(tmp)
            
        conn.close()
        sock.close()

    except :
        print("KeyBoardInterrupt")
        conn.close()
        sock.close()

if __name__ == "__main__" :
    main()
#%%
"""
    1회 테스트를 진행하는 섹션
"""
HOST = '192.168.0.100'
PORT = 30000
en   = 'utf-8'

tmp = float()
idx = 1
pos = list()

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((HOST, PORT))
sock.listen(5)
conn, addr = sock.accept()
msg = conn.recv(1024).decode(en)
while  msg != 'COMCHECK' :
    msg = conn.recv(1024).decode(en)
    # time.sleep(0.005)
conn.send("DONE".encode(en))

msg = conn.recv(1024).decode(en)
for i in range(len(msg)) :
    if msg[i] == ',' :
        tmp = float(msg[idx:i])
        pos.append(tmp)
        idx = i+1
    elif msg[i] == ']' :
        tmp = float(msg[idx:i])
        pos.append(tmp)
print(pos)