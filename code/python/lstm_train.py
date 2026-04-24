import pandas as pd
import numpy as np
from sklearn.preprocessing import LabelEncoder, MinMaxScaler
from sklearn.model_selection import train_test_split
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense

# =========================
# 1. Load dataset
# =========================
data = pd.read_csv("dataset/control_data.csv")

# =========================
# 2. Encode categorical data
# =========================
le_place = LabelEncoder()
le_control = LabelEncoder()
le_status = LabelEncoder()

data['place'] = le_place.fit_transform(data['place'])
data['control_type'] = le_control.fit_transform(data['control_type'])
data['status'] = le_status.fit_transform(data['status'])

# =========================
# 3. Features & Target
# =========================
X = data[['load_g', 'place', 'control_type', 'stabilization_time']].values
y = data['status'].values

# =========================
# 4. Normalize data
# =========================
scaler = MinMaxScaler()
X = scaler.fit_transform(X)

# =========================
# 5. Reshape for LSTM
# =========================
X = X.reshape((X.shape[0], 1, X.shape[1]))

# =========================
# 6. Train/Test split
# =========================
X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)

# =========================
# 7. Build LSTM Model
# =========================
model = Sequential()

model.add(LSTM(64, input_shape=(1, X.shape[2]), return_sequences=False))
model.add(Dense(32, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# =========================
# 8. Train model
# =========================
history = model.fit(
    X_train, y_train,
    epochs=50,
    batch_size=4,
    validation_data=(X_test, y_test)
)

# =========================
# 9. Save model
# =========================
model.save("lstm_control_model.h5")

print("Training completed and model saved!")
