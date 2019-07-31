#!/usr/bin/env python
import cv2, time, rospy
from std_msgs.msg import String
from keras.models import model_from_json
import numpy as np
from keras import datasets
from keras.utils import np_utils
from keras import layers, models
from collections import Counter

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
	Nh      = 30
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
	trained = False
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
	# For ROS
	rospy.init_node('number_notifier', anonymous=True)
	pub  = rospy.Publisher('chatter', String, queue_size=10)
	rate = rospy.Rate(10) # 10 Hz
	# rospy.init_node('number_notifier', anonymous=True)
	# List for appending classification result
	res_lst = list()
	if not trained :
		model   = train() # model train once
		trained = True
	try :
		while True :
			# Sleep for 100ms
			# time.sleep(0.1) <- replaced by rate.sleep()
			_, frame = cap.read()
			# Image Processing
			# 1) Edge Detection or Feature Extraction & Cropping
			# 2) Reshape after cropping
			gray        = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			gray_edge   = cv2.Canny(gray, 500, 600)
			X_matrix    = cv2.resize(gray_edge, dsize=(28, 28), interpolation=cv2.INTER_AREA)
			# cv2.imwrite("resized_gray.png", X_matrix)
			cv2.imwrite("edge_image.png", gray_edge)
			cv2.imwrite("resized_image.png", X_matrix)
			X_vector    = X_matrix.reshape(1, 784)
			predictions = model.predict(X_vector)
			# Apply Vote Algorithm
			print(predictions[0])
			res = np.argmax(predictions[0])
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
			print("Number:", res)
			# for ROS message
			number_str = str(res)
			rospy.loginfo(number_str)
			pub.publish(number_str)
			rate.sleep()
	except :
		print("exit")
		cap.release()
		exit()

if __name__=='__main__' :
	main()
