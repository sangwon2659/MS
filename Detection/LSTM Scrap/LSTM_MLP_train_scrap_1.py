import numpy as np
import matplotlib.pyplot as plt
import keras
from keras.layers import Dense
from keras.layers import Dropout
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import load_model
from pandas import read_csv

## Loading data ###
data = read_csv("Data.csv", header=None)
# Data normalization
data_norm = np.array(data)
data_max, data_min = data_norm[:,0:500].max(), data_norm[:,0:500].min()
max_min_diff = data_max-data_min
data_norm[:,0:500] = data_norm[:,0:500]/max_min_diff
# Shuffle data
np.random.shuffle(data_norm)
# Reshape data to fit into format
timesteps, n_features = data_norm.shape
data_norm = data_norm.reshape(timesteps, n_features,1)

# Train data
x_train_data = data_norm[4500:5500, 0:500]
y_train_data = data_norm[4500:5500, 500]
y_train_data = y_train_data.reshape((1000,1,1))

# Test data
x_test_data = data_norm[7750:8000, 0:500]
y_test_data = data_norm[7750:8000, 500]
y_test_data = y_test_data.reshape((250,1,1))

### Loading encoder ###
encoder = load_model('encoder.h5', compile=False)

### Encoding the train data ###
'''
for i in range(1000):
        x_train_data_temp = x_train_data[i,0:500]
        x_train_data = encoder.predict(x_train_data_temp)
'''

x_train_data_array = x_train_data[0,0:500]
x_train_data_array = x_train_data_array.reshape((1, 500, 1))
x_train_data_array = encoder.predict(x_train_data_array)
x_train_data_array = x_train_data_array.flatten()
x_train_data_array = x_train_data_array.reshape(1, 1, 32000)

for i in range(999):
        x_train_data_temp = x_train_data[i,0:500]
        x_train_data_temp = x_train_data_temp.reshape((1, 500, 1))
        x_train_data_temp = encoder.predict(x_train_data_temp)
        x_train_data_temp = x_train_data_temp.flatten()
        x_train_data_temp = x_train_data_temp.reshape(1, 1, 32000)
        x_train_data_array = np.vstack([x_train_data_array, x_train_data_temp])

print(np.shape(x_train_data_array))

### Creating the MLP model ###
# MLP with 1024 -> 512 -> 256 -> 1 layers
model = keras.Sequential()
model.add(keras.Input(shape=(32000,1)))
model.add(Dense(1024, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(512, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(256, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(1, activation='softmax'))
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['binary_accuracy'])
# Extracting a summary of the model
model.summary()

### Fitting the model ###
epoch = 100
history = model.fit(x_train_data_temp, y_train_data, epochs = epoch, verbose=1)

### Plotting history ###
# Plotting loss
plt.plot(history.history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

x_test_data_temp = x_train_data[0,0:500]
x_test_data_temp = x_test_data_temp.reshape((1, n_features, 1))
yhat = model.predict(x_test_data_temp, verbose=1)
test_data_yhat = np.colum_stack((y_train_data[0], yhat))
print(test_data_yhat)

