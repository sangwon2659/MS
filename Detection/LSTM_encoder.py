import numpy as np
import matplotlib.pyplot as plt
from keras.layers import LSTM
from keras.layers import RepeatVector
from keras.layers import TimeDistributed
from keras.layers import Dense
from keras.layers import Input
from tensorflow import keras
from pandas import read_csv

### Loading data ###
data = read_csv("Data.csv", header=None)
# Data normalization
data_norm = np.array(data)
data_norm = data_norm[:,0:500]
data_max, data_min = data_norm.max(), data_norm.min()
max_min_diff = data_max-data_min
data_norm = data_norm/max_min_diff
# Shuffle data
np.random.shuffle(data_norm)
# Reshape data to fit into format
timesteps, n_features = data_norm.shape
data_norm = data_norm.reshape(timesteps, n_features,1)

# Input data
input_data = data_norm[4500:5500,0:500]

# Test data
test_data = data_norm[7750:8000,0:500]

### Defining model ###
# Defining input shape
visible = Input(shape=(n_features, 1))
# LSTM encoder of shapes 128 and 64
encoder = LSTM(128, activation='relu', input_shape=(n_features, 1), return_sequences=True)(visible)
encoder = LSTM(64, activation='relu', return_sequences=True)(encoder)

# Reconstruction decoder of shapes 64 and 128
#decoder = RepeatVector(timesteps)(encoder)
decoder = LSTM(64, activation='relu', return_sequences=True)(encoder)
decoder = LSTM(128, activation='relu', return_sequences=True)(decoder)
# Organizing output back into the shape of input
decoder = TimeDistributed(Dense(1))(decoder)

# Tying encoder and decoder into one model
model = keras.Model(inputs=visible, outputs=decoder)
model.compile(optimizer='adam', loss='mse', metrics=['accuracy'])
# Printing a summary of the model
model.summary()

epoch = 5
history = model.fit(input_data, input_data, epochs = epoch, verbose=1)

### Plotting history ### 
# Plotting loss
plt.plot(history.history['loss'])
plt.title('Model Loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.show()

# Evaluating the results qualitatively
test_data_temp = test_data[0,0:500]
yhat = model.predict(test_data_temp, verbose=1)
print(test_data_temp)
print(yhat)

# Saving the encoder
enc_save = keras.Model(inputs = visible, outputs = encoder)
enc_save.save('encoder.h5')
