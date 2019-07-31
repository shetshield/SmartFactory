#!/usr/bin/env python
import cv2, time, rospy
from std_msgs.msg import String
from keras.models import model_from_json
import numpy as np
from keras import datasets
from keras.utils import np_utils
from keras import layers, models
from collections import Counter
from sklearn.externals import joblib
from skimage.feature import hog

def data_preprocessing() :
	(_X_trn, _Y_trn), (_X_tst, _Y_tst) = datasets.mnist.load_data()
	_Y_trn  = np_utils.to_categorical(_Y_trn)
	_Y_tst  = np_utils.to_categorical(_Y_tst)
	L, W, H = _X_trn.shape
	_X_trn  = _X_trn.reshape(-1, W*H)
	_X_tst  = _X_tst.reshape(-1, W*H)
	_X_trn  = _X_trn/255.
	_X_tst  = _X_tst/255.
	return(_X_trn, _Y_trn), (_X_tst, _Y_tst)

class ANN_model(models.Sequential) :
	def __init__(self, Nin, Nh, Nout) :
		super().__init__()
		self.add(layers.Dense(Nh, activation='relu', input_shape=(Nin,)))
		self.add(layers.Dense(Nout, activation='softmax'))
		self.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
def train() :
	Nin     = 784
	Nh      = 64
	Nout    = 10 # Number of class
	num_epo = 10
	model   = ANN_model(Nin, Nh, Nout)
	(X_train, Y_train), (X_test, Y_test) = data_preprocessing()
	history = model.fit(X_train, Y_train, epochs = num_epo, batch_size = 64, validation_split=0.2)
	perf_test = model.evaluate(X_test, Y_test, batch_size = 40)
	print('Performance_test:', perf_test)
	model_json = model.to_json()
	with open("model.json", "w") as json_file :
		json_file.write(model_json)
	model.save_weights("model.h5")
	print("Saved model to disk")
	return model

def main() :
	# Load the digits classifier
	clf = joblib.load("digits_cls.pkl")
	trained = False
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
	# For ROS
	rospy.init_node('number_notifier', anonymous=True)
	pub  = rospy.Publisher('chatter', String, queue_size=10)
	rate = rospy.Rate(5) # 5 Hz
#	# List for appending classification result
#	res_lst = list()
#	hog_lst = list()
#	num_el  = 0
	if not trained :
		model   = train() # model train once
		trained = True
	try :
		while True :
			_temp    = list()
			roi_list = list()
			_, frame = cap.read()
			# Image Processing
			gray        = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			gray_blur   = cv2.GaussianBlur(gray, (5, 5), 0)
			ret, im_th  = cv2.threshold(gray_blur, 90, 255, cv2.THRESH_BINARY_INV)
			img, ctrs, hier = cv2.findContours(im_th.copy(), mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
			rects = [cv2.boundingRect(ctr) for ctr in ctrs]
#			num_el += 1
			for rect in rects :
				# cv2.rectangle(frame, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0), 3)
				leng = int(rect[3] * 1.6)
				pt1  = int(rect[1] + rect[3] // 2 - leng // 2)
				pt2  = int(rect[0] + rect[2] // 2 - leng // 2)
				roi  = im_th[pt1:pt1+leng, pt2:pt2+leng] # roi.shape[1] < roi.shape[0]
				if roi.shape[1] < roi.shape[0] and roi.shape[0] > 15 and roi.shape[1] > 15 :
					cv2.rectangle(frame, (rect[0], rect[1]), (rect[0]+rect[2], rect[1]+rect[3]), (0, 255, 0), 3)
					roi  = cv2.resize(roi, dsize=(28, 28), interpolation=cv2.INTER_AREA)
					roi  = cv2.dilate(roi, (3, 3), iterations=1)
					roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualize=False)
					nbr  = clf.predict(np.array([roi_hog_fd], 'float64'))
					number_str = str(int(nbr[0]))
#					_temp = [roi, number_str, num_el]
					_temp = [roi, number_str]
					cv2.putText(frame, str(int(nbr[0])), (rect[0], rect[1]), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)
					roi_list.append(_temp)
#					hog_lst.append(number_str)
#			if len(roi_list) < 1 :
#				# In case of Element Not appended
#				num_el -= 1
			cv2.imwrite("HOG_Extract.png", frame)
			try :
				for i in range(len(roi_list)) :
					cv2.imwrite("digits_" + str(i+1) + ".png", roi_list[i][0])
#					if roi_list[i][1] == "7" :
#						cv2.imwrite("digits_7.png", roi_list[i][0])
#					else :
#						cv2.imwrite("digits_"+str(i+1)+".png", roi_list[i][0])
			except :
				pass
			try :
				for i in range(len(roi_list)) :
					X_vec   = roi_list[i][0].reshape(1, 784)
					hog_num = roi_list[i][1]
					pred    = model.predict(X_vec)
					ann_num = np.argmax(pred[0])
#					print("Number - ", "Learning:", ann_num, "HOG:", hog_num, "i:", i)
					if hog_num == str(ann_num) :
						# HOG classifier and ANN classifier result comparison
						# If both results are same, send msg
						rospy.loginfo(hog_num)
						pub.publish(hog_num)
#				for i in range(len(roi_list)) :
#					X_vector    = roi_list[i][0].reshape(1, 784)
#					cur_num     = roi_list[i][1]
#					predictions = model.predict(X_vector)
#					# Apply Vote Algorithm
#					# print(predictions[0])
#					res = np.argmax(predictions[0])
#					_temp = [res, num_el]
#					res_lst.append(_temp)
#					print("Number - ","Learning:", res,"HOG:" ,cur_num, "i:", i)
#					# number_str = str(res)
#					if num_el > 4 :
#						count_hog_res = Counter(hog_lst).most_common()
#						count_ann_res = Counter(res_lst).most_common()
#						if count_hog_res[0][1] == 4 and count_ann_res[0][1]==4 and count_hog_res[0][0] == count_ann_res[0][0]:
#							rospy.loginfo(hog_lst[0][0])
#							# rospy.loginfo(cur_num)
#							pub.publish(hog_lst[0][0])
#						del hog_lst[0], res_lst[0]
#				print("\n")
#				num_el -= 1
			except :
				print("NONE")
			# res_lst.append(res)
			# set Voting Threshold
			# if len(res_lst) > 8 :
			#	count_res = Counter(res_lst).most_common()
			# 	num_class = count_res[0][0]
			# 	res_lst   = list() # reset
			# 	num_str   = str(num_class)
			#	rospy.loginfor(num_str)
			# 	pub.publish(num_str)
			# rate.sleep()
			# print("Number:", res)
			# for ROS message
			# number_str = str(res)
			# rospy.loginfo(number_str)
			# pub.publish(number_str)
			rate.sleep()
	except :
		print("exit")
		cap.release()
		exit()

if __name__=='__main__' :
	main()
