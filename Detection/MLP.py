import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt
from numpy import array
from keras.models import Sequential
from keras.layers import Dense, LSTM

FFT_Hz = 50
train_min = 4500
train_max = 5500
test_min = 7750
test_max = 8000
epoch = 100

# Loading data
data = np.loadtxt("Data_.csv", delimiter=",")

# Filtering data
x_train_data = data[train_min:train_max, 0:FFT_Hz]
#np.random.shuffle(x_train_data)
y_train_data = data[train_min:train_max, FFT_Hz]
#np.random.shuffle(y_train_data)

# Preprocessing Y data
for i in range(train_max-train_min):
        if y_train_data[i] != 0.0:
                y_train_data[i] = 1.0

# Organizing test data
x_test_data = data[test_min:test_max, 0:FFT_Hz]
y_test_data = data[test_min:test_max, FFT_Hz]

for i in range(test_max-test_min):
	if y_test_data[i] != 0.0:
		y_test_data[i] = 1.0

# Defining model
model = Sequential()
model.add(Dense(1024, input_shape=(FFT_Hz,), activation='relu'))
model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.summary()

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
