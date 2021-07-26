import numpy as np
import matplotlib.pyplot as plt
from keras.layers import LSTM
from keras.layers import RepeatVector
from keras.layers import TimeDistributed
from keras.layers import Dense
from keras.layers import Input
from tensorflow import keras
from pandas import read_csv

###################### Parameter Variables ######################

FFT_Hz = 10+1
train_min = 200
train_max = 7000
test_min = 9500
test_max = 11500
epoch = 5

#################################################################

################ Loading and Normalizing Raw Data ################

#Loading Data
data = read_csv("Data_10.csv", header=None)
data = np.array(data)

#Stating necessary variables
total_samples = data.shape[0]
train_samples = train_max - train_min
test_samples = test_max - test_min

#Preparation for FFT data normalization
data_FFT = data[:, 0:FFT_Hz]
#data_FFT_max, data_FFT_min = data_FFT.max(), data_FFT.min()
#data_FFT_max_min_diff = data_FFT_max - data_FFT_min

#Slip data into binary classes
for i in range(total_samples):
        if data[i,FFT_Hz] != 0.0:
                data[i,FFT_Hz] = 1.0

##################################################################

###################### Preprocessing Data ########################

#Declaring training data
training_data = data[train_min:train_max, :]
#np.random.shuffle(training_data)

#Declaring x_train and y_train data
x_train_data, y_train_data = training_data[:, 0:FFT_Hz], training_data[:, FFT_Hz]

#Data normalization for x_train data
#x_train_data = (x_train_data/data_FFT_max_min_diff)*100

#Reshaping x_train and y_train data
x_train_data = x_train_data.reshape(train_samples, FFT_Hz, 1)
y_train_data = y_train_data.reshape(train_samples, 1, 1)

#Declaring test data
test_data = data[test_min:test_max, :]
#np.random.shuffle(test_data)

#Declaring x_test and y_test data
x_test_data, y_test_data = test_data[:, 0:FFT_Hz], test_data[:, FFT_Hz]

#Data normalization for x_test data
#x_test_data = (x_test_data/data_FFT_max_min_diff)*100

#Reshaping x_test and x_test data
x_test_data = x_test_data.reshape(test_samples, FFT_Hz, 1)
y_test_data = y_test_data.reshape(test_samples, 1, 1)

### Defining model ###
# Defining input shape
visible = Input(shape=(FFT_Hz, 1))
# LSTM encoder of shapes 128 and 64
encoder = LSTM(128, activation='relu', input_shape=(FFT_Hz, 1), return_sequences=True)(visible)
encoder = LSTM(64, activation='relu', return_sequences=True)(encoder)

# Reconstruction decoder of shapes 64 and 128
#decoder = RepeatVector(64)(encoder)
decoder = LSTM(64, activation='relu', return_sequences=True)(encoder)
decoder = LSTM(128, activation='relu', return_sequences=True)(decoder)
# Organizing output back into the shape of input
decoder = TimeDistributed(Dense(1))(decoder)

# Tying encoder and decoder into one model
model = keras.Model(inputs=visible, outputs=decoder)
model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])
# Printing a summary of the model
model.summary()

### Fitting model with the same input and output data ###
history = model.fit(x_train_data, y_train_data, epochs = epoch, verbose=1)

### Plotting history ### 
# Plotting loss
plt.plot(history.history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

# Evaluating the results qualitatively
'''
for i in range(5):
        test_data_temp = input_data[i,0:500]
        test_data_temp = test_data_temp.reshape((1,n_features,1))
        yhat = model.predict(test_data_temp, verbose=1)
        test_data_yhat = np.column_stack((input_data[i,0:500], yhat[0,0:500]))
'''

# Saving the encoder
enc_save = keras.Model(inputs = visible, outputs = encoder)
enc_save.save('encoder.h5')

'''
test = enc_save.predict(test_data_temp)
print(np.shape(test))
print(test)
'''
