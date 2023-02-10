# Jack Santina
# SI475 Lab04_2

import matplotlib.pyplot as plt
import tensorflow as tf
import tensorflow_datasets as tfds
from tensorflow import keras

ds_train, ds_info = tfds.load('rock_paper_scissors', split='train', with_info=True, as_supervised=True)
ds_test, ds_info = tfds.load('rock_paper_scissors', split='test', with_info=True, as_supervised=True)

def format_label(label):
    string_label = label_info.int2str(label)
    return string_label.split("-")[0]

label_info=ds_info.features["label"]
for i, (image, label) in enumerate(ds_train.take(9)):
    ax = plt.subplot(3, 3, i + 1)
    plt.imshow(image.numpy().astype("uint8"))
    plt.title("{}".format(format_label(label)))
    plt.axis("off")
plt.show() 

print(ds_info.features)
NUM_CLASSES=ds_info.features['label'].num_classes
IMG_SIZE=224
batch_size=32

# One-hot / categorical encoding
def input_preprocess(image, label):
    label = tf.one_hot(label, NUM_CLASSES)
    return image, label


ds_train = ds_train.map(
    input_preprocess, num_parallel_calls=tf.data.AUTOTUNE
)
ds_train = ds_train.batch(batch_size=batch_size, drop_remainder=True)
ds_train = ds_train.prefetch(tf.data.AUTOTUNE)

ds_test = ds_test.map(input_preprocess)
ds_test = ds_test.batch(batch_size=batch_size, drop_remainder=True)

# Data Augmentation
data_augmentation = keras.Sequential ([
    tf.image.resize(),
    keras.layers.RandomFlip("horizontal"),
    keras.layers.RandomRotation(0.3),
    keras.layers.RandomBrightness(0.3),
    keras.layers.RandomZoom(height_factor=(0.0,0.3)),
]) 

inputs = keras.layers.Input(shape=(IMG_SIZE, IMG_SIZE, 3))
x = data_augmentation(inputs)
model = keras.applications.efficientnet.EfficientNetB0(include_top=False, input_tensor=x, weights="imagenet")

# freeze the pretrained weights 
model.trainable = False 

model = keras.layers.Flatten()(model.output)
outputs = keras.layers.Dense(NUM_CLASSES, activation='softmax')(model)

model = keras.Model(inputs, outputs)

model.compile(optimizer='adam', 
        loss=keras.losses.SparseCategoricalCrossentropy(), 
        metrics=['accuracy'])

model.summary()

ds_train = tf.image.resize(ds_train, [IMG_SIZE, IMG_SIZE])
history = model.fit(ds_train, epochs=100)
