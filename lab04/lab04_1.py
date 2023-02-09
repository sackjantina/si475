# Jack Santina
# SI475 Lab04_1

import os
os.environ['TF_CPP_MIN_LOG_LEVEL']='3' #Removing this will result in much more introductory output

import tensorflow as tf 
from tensorflow import keras
tf.get_logger().setLevel('ERROR')

img_size=(240,240)
batch_size=32

# create dataset from image directory using keras
train_ds = keras.utils.image_dataset_from_directory(
        "/data/PetImages",
        labels='inferred',
        label_mode='int',
        class_names=['Cat', 'Dog'],
        batch_size=batch_size,
        image_size=img_size,
        )

# sanity check to read first 9 images
import matplotlib.pyplot as plt

plt.figure(figsize=(10, 10))
for images, labels in train_ds.take(1):
    for i in range(9):
        ax = plt.subplot(3, 3, i + 1)
        plt.imshow(images[i].numpy().astype("uint8"))
        plt.title(int(labels[i]))
        plt.axis("off")
plt.show()

# Data Augmentation
data_augmentation = keras.Sequential ([
    keras.layers.RandomFlip("horizontal"),
    keras.layers.RandomRotation(0.3),
    keras.layers.RandomBrightness(0.3),
    keras.layers.RandomZoom(height_factor=(0.0,0.3)),
]) 

plt.figure(figsize=(10, 10))
for images, _ in train_ds.take(1):
    for i in range(9):
        augmented_images = data_augmentation(images)
        ax = plt.subplot(3, 3, i + 1)
        plt.imshow(augmented_images[0].numpy().astype("uint8"))
        plt.axis("off")
plt.show() 

# Apply `data_augmentation` to the training images.
train_ds = train_ds.map(
    lambda img, label: (data_augmentation(img), label),
    num_parallel_calls=tf.data.AUTOTUNE,
)
# Prefetching samples in GPU memory helps maximize GPU utilization.
train_ds = train_ds.prefetch(tf.data.AUTOTUNE)

model = keras.Sequential()
model.add(keras.Input(shape=(img_size[0], img_size[1], 3)))
model.add(keras.layers.Rescaling(scale=1./255))
model.add(keras.layers.Conv2D(16, 3, padding='same', activation='relu'))
model.add(keras.layers.MaxPooling2D())
model.add(keras.layers.Conv2D(32, 3, padding='same', activation='relu'))
model.add(keras.layers.MaxPooling2D())
model.add(keras.layers.Conv2D(64, 3, padding='same', activation='relu'))
model.add(keras.layers.MaxPooling2D())
model.add(keras.layers.Flatten())
model.add(keras.layers.Dense(1))

model.compile(optimizer='adam', 
        loss=keras.losses.SparseCategoricalCrossentropy(), 
        metrics=['accuracy'])

model.summary()

model_checkpoint_callback = keras.callbacks.ModelCheckpoint(
        filepath="./checkpoint",
        save_weight_only=False,
        save_best_only=True)

epochs=10
history = model.fit(
        train_ds,
        epochs=epochs)
        #callbacks=[model_checkpoint_callback])

print('#####DONE#####')
