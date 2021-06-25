import numpy as np
import matplotlib.pyplot as plt
from keras.layers import LSTM
from keras.layers import RepeatVector
from keras.layers import TimeDistributed
from keras.layers import Dense
from keras.layers import Input
from tensorflow import keras
from pandas import read_csv

'''
input_data = np.array([[1,2], [1.1,2.1],[1.01, 2.13]])
timesteps, n_features = np.shape(input_data)
input_data = input_data.reshape(1,3,2)
test_data = np.array([[1.2,2.1],[1.01,1.9],[1.23,2]])
test_data = test_data.reshape(1,3,2)
model = keras.Sequential()
model.add(LSTM(128, activation='relu', input_shape=(timesteps, n_features), return_sequences=True))
model.add(LSTM(64, activation='relu', return_sequences=False))
model.add(RepeatVector(timesteps))
model.add(LSTM(64, activation='relu', return_sequences=True))
model.add(LSTM(128, activation='relu', return_sequences=True))
model.add(TimeDistributed(Dense(n_features)))
model.compile(optimizer='adam', loss='mse')
model.summary()
model.fit(input_data, input_data, epochs=300, verbose=1)
'''

'''
Preprocessing data
input_data = input_data[0,0,0:500].reshape(1,1,500)
print(input_data)
test_data = test_data[0,0,0:500].reshape(1,1,500)
'''

### Loading data ###
data = read_csv("/home/a283/DetectionAlgorithm/Data.csv", header=None)
# Data normalization
data_norm = np.array(data)
data_max, data_min = data_norm.max(), data_norm.min()
max_min_diff = data_max-data_min
data_norm = (data_norm/max_min_diff)*100
# Shuffle data
np.random.shuffle(data_norm)

# Input data from 3000 to remove noise
input_data = data_norm[4000:4600,0:500]
timesteps, n_features = np.shape(input_data)
# Reshaping data fit for learning
input_data = input_data.reshape(1, timesteps, n_features)

# Dividing into test data
test_data = data_norm[12000:15000,0:500]
test_timesteps, test_n_features = np.shape(test_data)
test_data = test_data.reshape(1, test_timesteps, test_n_features)

### Defining model ###
# Defining input shape
visible = Input(shape=(1, n_features,))
# LSTM encoder of shapes 128 and 64
encoder = LSTM(128, activation='relu', input_shape=(1, n_features), return_sequences=True)(visible)
encoder = LSTM(64, activation='relu', return_sequences=True)(encoder)

# Bottleneck
#n_bottleneck = 64
#bottleneck = Dense(n_bottleneck)(encoder)

# Reconstruction decoder of shapes 64 and 128
#decoder = RepeatVector(timesteps)(encoder)
decoder = LSTM(64, activation='relu', return_sequences=True)(encoder)
decoder = LSTM(128, activation='relu', return_sequences=True)(decoder)
# Organizing output back into the shape of input
decoder = TimeDistributed(Dense(n_features))(decoder)

# Tying encoder and decoder into one model
model = keras.Model(inputs=visible, outputs=decoder)
model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])
# Printing a summary of the model
model.summary()

# Defining history
history = {"loss": [], "accuracy": []};

### Fitting model ###
# Fitting each row of input data
Training_data_row_num = 600
epochs_ = 50
for i in range(Training_data_row_num):
	print("---------------------------------------{}th Row".format(i))
	# Slicing input_data row by row
	input_data_temp = input_data[0,i,0:500].reshape(1,1,500)
	history_temp = model.fit(input_data_temp, input_data_temp, epochs=epochs_, verbose=1)
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

### Prediction ###
# Slicing test_data into one row
test_data_ = data_norm[4000,0:500].reshape(1,1,500)
# Predicting using the whole model
yhat = model.predict(test_data_, verbose=1)
# Printing the test_data put in and the prediction from the whole model
print(test_data_)
print(yhat)

### Saving h5 file ###
# Defining enc_save as a model from the input upto the encoder
enc_save = keras.Model(inputs=visible, outputs=encoder)
enc_save.save('encoder.h5')

for i in range(50):
	input_data_temp = input_data[0,i,0:500].reshape(1,1,500)
	enc = enc_save.predict(input_data_temp, verbose=1)
	print(enc)

