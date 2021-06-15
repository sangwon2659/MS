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

# Load Data
data = read_csv("/home/a283/DetectionAlgorithm/Data.csv", header=None)
input_data = np.array(data.iloc[0:100,0:500])
timesteps, n_features = np.shape(input_data)
input_data = input_data.reshape(1, timesteps, n_features)

test_data = np.array(data.iloc[100:200,0:500])
test_timesteps, test_n_features = np.shape(test_data)
test_data = test_data.reshape(1, test_timesteps, test_n_features)

# Define encoder
visible = Input(shape=(timesteps, n_features,))
encoder = LSTM(128, activation='relu', input_shape=(timesteps, n_features), return_sequences=True)(visible)
encoder = LSTM(64, activation='relu', return_sequences=False)(encoder)

# Bottleneck
#n_bottleneck = 64
#bottleneck = Dense(n_bottleneck)(encoder)
bottleneck = encoder

# Reconstruction decoder
decoder = RepeatVector(timesteps)(encoder)
decoder = LSTM(64, activation='relu', return_sequences=True)(decoder)
decoder = LSTM(128, activation='relu', return_sequences=True)(decoder)
decoder = TimeDistributed(Dense(n_features))(decoder)

# Tie together
model = keras.Model(inputs=visible, outputs=decoder)
model.compile(optimizer='adam', loss='mse')

# Fit model
history = model.fit(input_data, input_data, epochs=300, verbose=1)

# Demonstrate prediction
yhat = model.predict(test_data, verbose=1)
print(yhat)

plt.plot(history.history['loss'])
plt.show()

enc_save = keras.Model(inputs=visible, outputs=bottleneck)
enc_save.save('encoder.h5')
