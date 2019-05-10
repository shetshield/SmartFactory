# SmartFactory 과제에 사용 중인 코드 업로드
# 1) UI(Python-PyQt)
# 2) UR3 / Motor / PC 간 통신 및 제어프로그램(Python)
# 3) Motor 제어 코드(OpenCM-arduino)
# 4) 스마트팩토리 UI
# UI 파일 변환(.ui >> .py)
# cmd or anaconda prompt 에서
# cd "ui_file_path" >> .ui 파일이 있는 위치로 디렉토리 변경
# pyuic5 -x SMF_2nd_UI.ui -o SMF_2nd_UI.py
# 원하는 이름으로 생성해주면 됨
# UI 파일 수정: (PyQt) Designer
# 현재 업로드 된 코드는 UI 파일의 절대 경로를 참조하고 있으므로, 경로 수정 필요함
