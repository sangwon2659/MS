import numpy as np
import matplotlib.pyplot as plt
import keras
from keras.layers import Dense
from keras.layers import Dropout
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import load_model
from pandas import read_csv

### Manually Preprocessing data ###
# Dividing data into x_train, y_train, x_test and y_test
# Data starting from 3000 to remove noise
data = read_csv("/home/a283/DetectionAlgorithm/Data.csv", header=None)
# Data normalization
data_norm = np.array(data)
data_norm = data_norm[4000:4700,:]
data_max, data_min = data_norm.max(), data_norm.min()
max_min_diff = data_max-data_min
data_norm[:,0:500] = data_norm[:,0:500]/max_min_diff
# Shuffle data
np.random.shuffle(data_norm)
print(np.shape(data_norm))

x_train_data = data_norm[0:600,0:500]
x_train_timesteps, x_train_n_features = np.shape(x_train_data)
x_train_data = x_train_data.reshape(1, x_train_timesteps, x_train_n_features)

y_train_data = data_norm[0:600,500]
y_train_timesteps = len(y_train_data)
y_train_data = y_train_data.reshape(1, 1, y_train_timesteps)

x_test_data = data_norm[600:700,0:500]
x_test_timesteps, x_test_n_features = np.shape(x_test_data)
x_test_data = x_test_data.reshape(1, x_test_timesteps, x_test_n_features)

y_test_data = data_norm[600:700,500]
y_test_timesteps = len(y_test_data)
y_test_data = y_test_data.reshape(1, 1, y_test_timesteps)

### Loading encoder ###
encoder = load_model('encoder_.h5')

### Creating the MLP model ###
# MLP with 1024 -> 512 -> 256 -> 2 layers
model = keras.Sequential()
model.add(keras.Input(shape=(1,64)))
model.add(Dense(1024, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(512, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(256, activation='relu'))
model.add(Dropout(0.3))
model.add(Dense(2, activation='softmax'))
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
# Extracting a summary of the model
model.summary()

# Defining history
history = {"loss": [], "accuracy": []};

### Fitting data into the model ###
Training_data_row_num = 4000
for i in range(Training_data_row_num):
        # Reshaping data to be put into encoder
        x_train_data_temp = x_train_data[0,i,0:500].reshape(1,1,500)
        x_train_enc = encoder.predict(x_train_data_temp)
        print("y value: {}".format(y_train_data[0,0,i]))
        if y_train_data[0,0,i] == 1.0:
                y_train_data_temp = np.array([[[1, 0]]])
        else:
                y_train_data_temp = np.array([[[0, 1]]])
        history_temp = model.fit(x_train_enc, y_train_data_temp, epochs = 300, verbose = 1)
        # Adding history_temp data to history
        # Extend function adds the history_temp list as single elements
        history["loss"].extend(history_temp.history['loss'])
        history["accuracy"].extend(history_temp.history['accuracy'])

### Plotting history ###
# Plotting loss
plt.plot(history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

# Plotting accuracy
plt.plot(history['accuracy'])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.show()

# Prediction from encoded test data through the MLP model
#yhat = model.predict(x_test_enc)

# Printing accuracy using the y test data
#acc = accuracy_score(y_test_data, yhat)
#print(acc)
