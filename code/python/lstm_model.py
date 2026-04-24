import pandas as pd
import numpy as np
from sklearn.preprocessing import LabelEncoder, MinMaxScaler
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense

# ======================
# 1. Load dataset
# ======================
df = pd.read_csv("data.csv")

# ======================
# 2. Encode categorical data
# ======================
le_place = LabelEncoder()
le_control = LabelEncoder()
le_status = LabelEncoder()

df['place'] = le_place.fit_transform(df['place'])
df['control_type'] = le_control.fit_transform(df['control_type'])
df['status'] = le_status.fit_transform(df['status'])  # stable=1 unstable=0

# ======================
# 3. Features & Labels
# ======================
X = df[['load_g', 'place', 'control_type', 'stabilization_time']].values
y = df['status'].values

# ======================
# 4. Normalize data
# ======================
scaler = MinMaxScaler()
X = scaler.fit_transform(X)

# ======================
# 5. Reshape for LSTM
# (samples, timesteps, features)
# هنا بنعتبر كل عينة timestep واحد
# ======================
X = X.reshape((X.shape[0], 1, X.shape[1]))

# ======================
# 6. Build LSTM model
# ======================
model = Sequential()
model.add(LSTM(64, input_shape=(1, X.shape[2]), activation='relu'))
model.add(Dense(32, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# ======================
# 7. Train model
# ======================
model.fit(X, y, epochs=50, batch_size=4, verbose=1)

# ======================
# 8. Test prediction
# ======================
sample = np.array([[200, 0, 1, 1.5]])  # مثال
sample = scaler.transform(sample)
sample = sample.reshape((1,1,4))

pred = model.predict(sample)
print("Prediction:", pred)
