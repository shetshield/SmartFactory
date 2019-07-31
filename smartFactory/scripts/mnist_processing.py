#!/usr/bin/env python
import numpy as np
from keras import datasets
from keras.utils import np_utils

print("pkg import")

def data_preprocessing() :
	(_X_train, _Y_train), (_X_test, _Y_test) = datasets.mnist.load_data()
	_Y_train = np_utils.to_categorical(_Y_train)
	_Y_test  = np_utils.to_categorical(_Y_test)
	L, W, H  = _X_train.shape
	_X_train = _X_train.reshape(-1, W*H)
	_X_test  = _X_test.reshape(-1, W*H)
	_X_train = _X_train/255.
	_X_test  = _X_test/255.
	return (_X_train, _Y_train), (_X_test, _Y_test)

from keras import layers, models
class ANN_MDL(models.Sequential) :
	def __init__(self, Nin, Nh, Nout) :
		super().__init__()
		self.add(layers.Dense(Nh, activation='relu', input_shape=(Nin,)))
		self.add(layers.Dense(Nout, activation='softmax'))
		self.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

Nin = 784
Nh  = 30
num_class = 10
Nout  = num_class
model = ANN_MDL(Nin, Nh, Nout)
(X_train, Y_train), (X_test, Y_test) = data_preprocessing()
history = model.fit(X_train, Y_train, epochs = 30, batch_size = 64, validation_split=0.2)
performance_test = model.evaluate(X_test, Y_test, batch_size  = 40)
print('Performance_test:', performance_test)
model_json = model.to_json()
with open("model.json", "w") as json_file :
	json_file.write(model_json)
model.save_weights("model.h5")
print("Saved model to disk")

