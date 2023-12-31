#Requirement Library
#import cv2
#import sklearn
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score
from sklearn.metrics import mean_squared_error
import tensorflow as tf
import keras
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import rosbag
import math
import os
import time
import subprocess

#========================================================
# Functions
#========================================================
def wait_for_key():
    input("Press any key to continue...")
    print("Continuing...")
def linear_map(x, x_min, x_max, y_min, y_max):
    return (x - x_min) / (x_max - x_min) * (y_max - y_min) + y_min
wait_for_key()
def preprocess_lidar(arr, ratio):
    num_to_remove = int(len(arr) * ratio)
    trimmed_arr = arr[num_to_remove:-num_to_remove]
    return trimmed_arr
#========================================================
#Get Data
#========================================================
total_number_samples = 0
overall_lidar_samples = []
overall_servo_samples = []
overall_speed_samples = []
lidar = []
servo = []
speed = []
test_lidar = []
test_servo = []
test_speed = []
max_speed = 0
temp_cnt = 1

for pth in ['../rp_lidar_out.bag']:
    if(not os.path.exists(pth)):
        print(f"out.bag doesn't exists in {pth}")
        exit(0)
    good_bag = rosbag.Bag(pth)

    lidar_data = []
    servo_data = []
    speed_data = []

    for topic, msg, t in good_bag.read_messages():
        if topic == 'Lidar':
            ranges = msg.ranges
            total_number_samples+=1
            # Remove quandrant of LIDAR directly behind us
            ratio = 45/360
            l_ranges = ranges#preprocess_lidar(ranges, ratio)
            lidar_data.append(l_ranges)

        if topic == 'Ackermann':
            data = msg.drive.steering_angle
            s_data = msg.drive.speed
            servo_data.append(data)
            if s_data>max_speed:
                max_speed = s_data
            s_data = linear_map(s_data, 0, 5, 0, 1)
            speed_data.append(s_data)

    overall_lidar_samples.extend(lidar_data)
    overall_servo_samples.extend(servo_data)
    overall_speed_samples.extend(speed_data)


    lidar_data = np.array(lidar_data) 
    servo_data = np.array(servo_data)
    speed_data = np.array(speed_data)
    data = np.concatenate((servo_data[:, np.newaxis], speed_data[:, np.newaxis]), axis=1)

    from sklearn.utils import shuffle
    shuffled_data = shuffle(data, random_state = 72)
    shuffled_lidar_data = shuffle(lidar_data, random_state = 72)
    # Check the shape of shuffled_data to ensure it has two columns
    print("Shape of shuffled_data:", shuffled_data.shape)

    # Split the shuffled data into train and test sets
    train_ratio = 0.85  # Adjust the ratio as needed
    train_samples = int(train_ratio * len(shuffled_lidar_data))
    x_train_bag, x_test_bag = shuffled_lidar_data[:train_samples], shuffled_lidar_data[train_samples:]

    # Check the shape of x_train_bag and x_test_bag
    print("Shape of x_train_bag:", x_train_bag.shape)
    print("Shape of x_test_bag:", x_test_bag.shape)

    # Extract servo and speed values from y_train_bag and y_test_bag
    y_train_bag = shuffled_data[:train_samples]
    y_test_bag = shuffled_data[train_samples:]

    # Check the shape of y_train_bag and y_test_bag
    print("Shape of y_train_bag:", y_train_bag.shape)
    print("Shape of y_test_bag:", y_test_bag.shape)

    # Extend the appropriate lists
    lidar.extend(x_train_bag)
    servo.extend(y_train_bag[:, 0])  # Extract servo values
    speed.extend(y_train_bag[:, 1])  # Extract speed values

    test_lidar.extend(x_test_bag)
    test_servo.extend(y_test_bag[:, 0])  # Extract servo values for test
    test_speed.extend(y_test_bag[:, 1])  # Extract speed values for test
    print(f'\nData in {pth}:')
    print(f'Shape of Train Data --- Lidar: {len(lidar)}, Servo: {len(servo)}, Speed: {len(speed)}')
    print(f'Shape of Test Data --- Lidar: {len(test_lidar)}, servo: {len(test_servo)}, Speed: {len(test_speed)}')

    wait_for_key()

print(f'Overall Samples = {total_number_samples}')
lidar = np.asarray(lidar)
servo = np.asarray(servo)
speed = np.asarray(speed)
print(f'Loaded {len(lidar)} Training samples ---- {(len(lidar)/total_number_samples)*100:0.2f}% of overall')
test_lidar = np.asarray(test_lidar)
test_servo = np.asarray(test_servo)
test_speed = np.asarray(test_speed)
print(f'Loaded {len(test_lidar)} Testing samples ---- {(len(test_lidar)/total_number_samples)*100:0.2f}% of overall\n')
overall_lidar_samples = np.asarray(overall_lidar_samples)
overall_servo_samples = np.asarray(overall_servo_samples)
overall_speed_samples = np.asarray(overall_speed_samples)



assert len(lidar) == len(servo) == len(speed)
assert len(test_lidar) == len(test_servo) == len(test_speed)
assert len(overall_lidar_samples) == len(overall_servo_samples) == len(overall_speed_samples)
wait_for_key()

#======================================================
# Split Dataset
#======================================================
print('Spliting Data to Train/Test')

train_data = np.concatenate((servo[:, np.newaxis], speed[:, np.newaxis]), axis=1)
test_data =  np.concatenate((test_servo[:, np.newaxis], test_speed[:, np.newaxis]), axis=1)
overall_data_samples = np.concatenate((overall_servo_samples[:, np.newaxis], overall_speed_samples[:,np.newaxis]), axis=1)
#validation_data = np.concatenate((validation_servo[:, np.newaxis], validation_speed[:, np.newaxis]), axis=1)

print(f'Train Data(lidar): {lidar.shape}')
print(f'Train Data(servo,speed): {train_data.shape}')
#print(f'Validation Data(servo, speed): {validation_data.shape}')
print(f'Test Data(lidar): {test_lidar.shape}')
print(f'Test Data(servo, speed): {test_data.shape}')


#x_train, x_test, y_train, y_test = train_test_split(lidar, train_data, test_size = 0.15, shuffle=False)
#x_train, x_test, y_train, y_test = train_test_split(lidar, train_data, test_size = 0.15,  stratify = train_data)

#print(f'Train Size: {len(x_train)}')
#print(f'Train Size, y_train: {y_train.shape}')
#print(f'Validation Size: {len(x_test)}')
#print(f'y_test.shape{y_test.shape}\n')
wait_for_key()

#======================================================
# DNN Arch
#======================================================
'''
num_lidar_range_values = len(lidar[0])
print(f'num_lidar_range_values: {num_lidar_range_values}')
print(f'lidar[0]: {lidar[0]}')
# Mess around with strides, kernel_size, Max Pooling
model = tf.keras.Sequential([
    tf.keras.layers.Conv1D(filters=24, kernel_size=10, strides=4, activation='relu', input_shape=(num_lidar_range_values, 1)),
    tf.keras.layers.Conv1D(filters=36, kernel_size=8, strides=4, activation='relu'),
    tf.keras.layers.Conv1D(filters=48, kernel_size=4, strides=2, activation='relu'),
    tf.keras.layers.Conv1D(filters=64, kernel_size=3, activation='relu'),
    tf.keras.layers.Conv1D(filters=64, kernel_size=3, activation='relu'),
    tf.keras.layers.Flatten(),
    tf.keras.layers.Dense(100, activation='relu'),
    tf.keras.layers.Dense(50, activation='relu'),
    tf.keras.layers.Dense(10, activation='relu'),
    tf.keras.layers.Dense(2, activation='tanh')
])
#======================================================
# Model Compilation
#======================================================
#lr = 3e-4
lr = 5e-5

optimizer = tf.keras.optimizers.Adam(lr)
model.compile(optimizer=optimizer, loss='huber')#, metrics = [r2]) 

#huber is noisy data else 'mean_squared_error'

print(model.summary())
wait_for_key()

#======================================================
# Model Fit
#======================================================
##See Data Balance in DeepPiCar

batch_size = 64
num_epochs = 40

start_time = time.time()
history = model.fit(overall_lidar_samples, overall_data_samples, epochs=num_epochs, batch_size=batch_size, validation_data=(test_lidar, test_data))
#history = model.fit(lidar, train_data, epochs=num_epochs, batch_size=batch_size, validation_data=(test_lidar, test_data))
#history = model.fit(lidar, train_data, epochs=num_epochs, batch_size=batch_size, validation_data=(x_test, y_test))
#history = model.fit(lidar, test_data, epochs=num_epochs, batch_size=batch_size, validation_data=(x_test, y_test))
#history = model.fit(x_train, y_train, epochs=num_epochs, batch_size=batch_size, validation_data=(x_test, y_test))

# Plot training and validation losses 
print(history.history.keys())
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'test'], loc='upper left')
plt.savefig('./graphs/loss_curve.png')
plt.close()
    
#plt.show()

print(f'=============>{int(time.time() - start_time)} seconds<=============')

wait_for_key()


#======================================================
# Model Evaluation
#======================================================
print("==========================================")
print("Model Evaluation")
print("==========================================")


test_loss = model.evaluate(test_lidar, test_data)
#test_loss = model.evaluate(x_test, y_test)
print(f'test_loss = {test_loss}')

#y_pred = model.predict(x_test)

y_pred = model.predict(test_lidar)
#y_test = t_test_data
#print(f'y_pred.shape: {y_pred.shape}')
#accuracy = np.mean(y_pred == y_test)

#r2 = r2_score(y_test, y_pred)
r2 = r2_score(test_data, y_pred)
#error = mean_squared_error(y_test, y_pred)
error = mean_squared_error(test_data, y_pred)

print('\nOverall Evaluation:')
print(f'r2: {r2:.3f}')
print(f'Error: {error}')

speed_test_loss = model.evaluate(test_lidar, test_data)
x_test = test_lidar
y_test = test_data
speed_y_pred = model.predict(x_test)[:, 1]  # Extracting the speed predictions

# Calculate R-squared (R2) score for Speed
speed_r2 = r2_score(y_test[:, 1], speed_y_pred)

# Calculate Mean Squared Error (MSE) for Speed
speed_error = mean_squared_error(y_test[:, 1], speed_y_pred)

print("\nSpeed Evaluation:")
print(f"Speed Test Loss: {speed_test_loss}")
print(f"Speed R-squared (R2) Score: {speed_r2}")
print(f"Speed Mean Squared Error (MSE): {speed_error}")

# Model Evaluation for Servo
servo_y_pred = model.predict(x_test)[:, 0]  # Extracting the servo predictions

# Calculate R-squared (R2) score for Servo
servo_r2 = r2_score(y_test[:, 0], servo_y_pred)

# Calculate Mean Squared Error (MSE) for Servo
servo_error = mean_squared_error(y_test[:, 0], servo_y_pred)

print("\nServo Evaluation:")
print(f"Servo R-squared (R2) Score: {servo_r2}")
print(f"Servo Mean Squared Error (MSE): {servo_error}\n")
wait_for_key()

#======================================================
# Prune
#======================================================

#======================================================
# Save Model
#======================================================
model_file = 'rp_f1_tenth_model'
model.save(model_file+'.h5')
print("Model Saved")


# Set TensorFlow logging level to debug
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0'
tf.debugging.set_log_device_placement(True)


converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

quantized_tflite_model = converter.convert()

with open(model_file+".tflite", 'wb') as f:
    f.write(quantized_tflite_model)
    print (model_file+".tflite is saved. copy this file to the robot")
print('Tf_lite Model also saved')
wait_for_key()
'''
model_file = 'rp_f1_tenth_model'
#model.save(model_file+'.h5')
#print("Model Saved")


# Set TensorFlow logging level to debug
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '0'
tf.debugging.set_log_device_placement(True)

model = tf.keras.models.load_model(model_file+'.h5')

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# Assuming 'lidar' is your NumPy array
rep_32 = overall_lidar_samples.astype(np.float32)
rep_32 = np.expand_dims(rep_32, -1)

# Convert the NumPy array to a TensorFlow dataset
dataset = tf.data.Dataset.from_tensor_slices(rep_32)

# Define the representative_data_gen function
def representative_data_gen():
    for input_value in dataset.batch(1).take(100):
        # Print information about each input value
        print(f'Type: {type(input_value)}, Shape: {input_value.shape}')
        # Yield the tensor directly
        yield [input_value]


# Print dtype and shape of the lidar data
print(lidar.dtype)
print(lidar.shape)
#print(representative_data_gen().shape)
converter.representative_dataset = representative_data_gen
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]

quantized_tflite_model = converter.convert()

with open(model_file+"_micro.tflite", 'wb') as f:
    f.write(quantized_tflite_model)
    print (model_file+".tflite is saved. copy this file to the robot")
print('Tf_lite Model also saved')



tflite_model_file = './rp_f1_tenth_model_micro.tflite'
output_header_file = './rp_f1_tenth_model_micro.cc'

# Execute xxd -i command to generate C header file from the TFLite model
#try:
#    subprocess.run(["xxd", "-i", tflite_model_file], stdout=open(output_header_file, "w"))
#    print("C header file generated successfully:", output_header_file)
#except subprocess.CalledProcessError as e:
#    print("Error:", str(e))



wait_for_key()

#======================================================
# Evaluated TfLite Model
#======================================================

print("==========================================")
print("TFLite Evaluation")
print("==========================================")
model_name = './rp_f1_tenth_model_micro'
interpreter = tf.lite.Interpreter(model_path=model_name+'.tflite')
interpreter.allocate_tensors()
input_index = interpreter.get_input_details()[0]["index"]
output_details = interpreter.get_output_details()

output_lidar = test_lidar
output_servo = []
output_speed = []
y_test = test_data

hz = 100
period = 1.0 / hz


# Initialize a list to store inference times in microseconds
inference_times_micros = []

start = time.time()

# Iterate through the lidar data
for lidar_data in output_lidar:
    # Preprocess lidar data for inference
    lidar_data = np.expand_dims(lidar_data, axis=-1).astype(np.float32)
    lidar_data = np.expand_dims(lidar_data, axis=0)

    # Check for empty lidar data
    if lidar_data is None:
        continue

    # Measure inference time
    ts = time.time()
    interpreter.set_tensor(input_index, lidar_data)
    interpreter.invoke()
    output = interpreter.get_tensor(output_details[0]['index'])
    dur = time.time() - ts

    # Convert inference time to microseconds
    inference_time_micros = dur * 1e6
    inference_times_micros.append(inference_time_micros)

    # Print inference time information
    if dur > period:
        print("%.3f: took %.2f microseconds - deadline miss." % (ts - start, int(dur * 1000000)))
    #else:
        #print("%.3f: took %.2f microseconds" % (ts - start, dur * 1000000))

    # Extract servo and speed output from the model
    servo = output[0, 0]
    speed = output[0, 1]

    # Append output servo and speed
    output_servo.append(servo)
    output_speed.append(speed)

# Calculate average and maximum inference times in microseconds
average_inference_time_micros = np.mean(inference_times_micros)
max_inference_time_micros = np.max(inference_times_micros)

# Print inference time statistics
print("Average Inference Time: %.2f microseconds" % average_inference_time_micros)
print("Maximum Inference Time: %.2f microseconds" % max_inference_time_micros)

# Plot inference times
arr = np.array(inference_times_micros)
perc99 = np.percentile(arr, 99)
arr = arr[arr < perc99]
plt.plot(arr)
plt.xlabel('Inference Iteration')
plt.ylabel('Inference Time (microseconds)')
plt.title('Inference Time per Iteration')

# Save the plot as an image
plt.savefig('./graphs/inference_time_plot.png')
# Close the plot to free up resources
plt.close()
print(f"Plot saved")
#plt.show()

output_lidar = np.asarray(output_lidar)
output_servo = np.asarray(output_servo)
output_speed = np.asarray(output_speed)
assert len(output_lidar) == len(output_servo) == len(output_speed)
output = np.concatenate((output_servo[:, np.newaxis], output_speed[:, np.newaxis]), axis=1)

y_pred = output
print(f'y_pred.shape: {y_pred.shape}')
r2 = r2_score(test_data, y_pred)
error = mean_squared_error(test_data, y_pred)
print(f'r2: {r2:.3f}')
print(f'Error: {error}')


print('End')



