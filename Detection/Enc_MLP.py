import numpy as np
import matplotlib.pyplot as plt
import keras
from keras.layers import Dense
from keras.layers import Dropout
from keras.layers import TimeDistributed
from keras.layers import Permute
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import load_model
from pandas import read_csv

##################### Parameter Variables ######################

FFT_Hz = 10+1
train_min = 200
train_max = 7000
test_min = 9500
test_max = 11500
epoch = 10

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

### Loading encoder ###
encoder = load_model('encoder.h5')

### Creating the MLP model ###
# MLP with 1024 -> 512 -> 256 -> 2 layers
model = keras.Sequential()
model.add(keras.Input(shape=(FFT_Hz,64)))
model.add(TimeDistributed(Dense(1)))
model.add(Permute((2,1), input_shape=(FFT_Hz,1)))
model.add(Dense(1024, input_shape=(FFT_Hz,), activation='relu'))
model.add(Dense(512, activation='relu'))
model.add(Dense(256, activation='relu'))
model.add(Dense(64, activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(1, activation='sigmoid'))
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['binary_accuracy']) 
# Extracting a summary of the model
model.summary()

# Defining history
history = {"loss": [], "accuracy": []};

x_train_enc = encoder.predict(x_train_data)
hitory = model.fit(x_train_enc, y_train_data, epochs = epoch, verbose=1)

'''
### Fitting data into the model ###
Training_data_row_num = 50
epochs_ = 30
for i in range(Training_data_row_num):
	# Reshaping data to be put into encoder
	x_train_data_temp = x_train_data[0,i,0:500].reshape(1,1,500)
	x_train_enc = encoder.predict(x_train_data_temp)
	if y_train_data[0,0,i] == 0.0:
		# [[[1,0]]] == slip
		y_train_data_temp = np.array([[[0, 1]]])
	else:
		y_train_data_temp = np.array([[[1, 0]]])	
	history_temp = model.fit(x_train_enc, y_train_data_temp, epochs = epochs_, verbose = 1)
	print(model.predict(x_train_enc))
	# Adding history_temp data to history
        # Extend function adds the history_temp list as single elements
	history["loss"].extend(history_temp.history['loss'])
	#history["accuracy"].append()
'''

### Plotting history ###
# Plotting loss
plt.plot(history["loss"])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

plt.plot(history["accuracy"])
plt.title('Model Accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.show()

'''
# Prediction from encoded test data through the MLP model
for j in range(500):
	x_test_data_temp = x_test_data[0,j,0:500].reshape(1,1,500)
	x_test_enc = encoder.predict(x_test_data_temp)
	yhat = model.predict(x_test_enc)
	print("groundtruth: {}".format(y_test_data[0,0,j]))
	print("yhat: {}".format(yhat))
'''

# Printing accuracy using the y test data
#acc = accuracy_score(y_test_data, yhat)
#print(acc)

x_test_enc = encoder.predict(x_test_data)

# Evaluating results
evaluation = model.evaluate(x_test_enc, y_test_data)
