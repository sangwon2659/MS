import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import load_model
from pandas import read_csv

# Load data
data = read_csv("/home/a283/DetectionAlgorithm/Data.csv", header=None)
x_train_data = np.array(data.iloc[0:100,0:500])
x_train_timesteps, x_train_n_features = np.shape(x_train_data)
x_train_data = x_train_data.reshape(1, x_train_timesteps, x_train_n_features)

y_train_data = np.array(data.iloc[0:100,500])
y_train_timesteps, y_train_n_features = np.shape(y_train_data)
y_train_data = y_train_data.reshape(1, y_train_timesteps, y_train_n_features)

x_test_data = np.array(data.iloc[100:200,0:500])
x_test_timesteps, x_test_n_features = np.shape(x_test_data)
x_test_data = x_test_data.reshape(1, x_test_timesteps, x_test_n_features)

y_test_data = np.array(data.iloc[100:200,500])
y_test_timesteps, y_test_n_features = np.shape(y_test_data)
y_test_data = y_test_data.reshape(1, y_test_timesteps, y_test_n_features)

print(x_train_data)
print(y_train_data)
print(x_test_data)
print(y_test_data)

# Load encoder
encoder = load_model('encoder.h5')

# Prediction from train and test data through the loaded encoder
x_train_enc = encoder.predict(x_train_data)

x_test_enc = encoder.predict(x_test_data)

# Creating the MLP model
model =

# Fitting the encoded train data
model.fit(x_train_enc, y_train)

# Prediction from encoded test data through the MLP model
yhat = model.predict(x_test_enc)

# Printing accuracy using the y test data
acc = accuracy_score(y_test_data, yhat)
print(acc)
