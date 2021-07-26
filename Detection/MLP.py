import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from keras import optimizers
from numpy import array
from keras.models import Sequential
from keras.layers import Dense, LSTM

FFT_Hz = 10+1
train_min = 200
train_max = 7000
test_min = 9500
test_max = 11500
epoch = 10

# Loading data
data = np.loadtxt("Data_10.csv", delimiter=",")

data_FFTCov = data[:, 0:FFT_Hz]
data_max_FFT = data_FFTCov[:, 0:FFT_Hz-1].max()  
data_max_Cov = data_FFTCov[:, FFT_Hz-1].max()
train_sample_num = train_max - train_min
data[:, 0:FFT_Hz-1] = data[:, 0:FFT_Hz-1]/data_max_FFT
data[:, FFT_Hz-1] = data[:, FFT_Hz-1]/data_max_Cov

# Filtering data
x_train_data = data[train_min:train_max, 0:FFT_Hz]
#np.random.shuffle(x_train_data)
y_train_data = data[train_min:train_max, FFT_Hz]
y_train_data = y_train_data.reshape(train_sample_num, 1)
#np.random.shuffle(y_train_data)

# Preprocessing Y data
for i in range(train_max-train_min):
	if y_train_data[i] != 0.0:
		y_train_data[i] = 1
	else:
		y_train_data[i] = 0

# Organizing test data
x_test_data = data[test_min:test_max, 0:FFT_Hz]
y_test_data = data[test_min:test_max, FFT_Hz]

for i in range(test_max-test_min):
	if y_test_data[i] != 0.0:
		y_test_data[i] = 1
	else:
		y_test_data[i] = 0

# Defining model
model = Sequential()
model.add(Dense(1024, input_shape=(FFT_Hz,), activation='relu'))
model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.summary()

sgd = optimizers.SGD(lr=0.1)
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
history = model.fit(x_train_data, y_train_data, epochs=epoch, batch_size=10, verbose=1)

# Displaying loss history
plt.plot(history.history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

plt.plot(history.history['accuracy'])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.show()

# Evaluating results
evaluation = model.evaluate(x_test_data, y_test_data)
