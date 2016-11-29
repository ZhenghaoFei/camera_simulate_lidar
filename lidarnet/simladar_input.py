# Copyright 2015 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================


from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import glob
import os

from six.moves import xrange  # pylint: disable=redefined-builtin
import tensorflow as tf

# Global constants describing the  data set.
NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN = 100
NUM_EXAMPLES_PER_EPOCH_FOR_EVAL = 100

IMAGE_HEIGHT = 16
IMAGE_WIDTH = 64

ORIGINAL_IMAGE_HEIGHT = 128
ORIGINAL_IMAGE_WIDTH = 672
IMAGE_DEPTH = 3
LIDAR_DIM =120

# Constants used for dealing with the files, matches convert_to_records.
TRAIN1_FILE = 'train1.tfrecords'
TRAIN2_FILE = 'train2.tfrecords'

VALIDATION_FILE = 'validate.tfrecords'


def read_and_decode(filename_queue):
  reader = tf.TFRecordReader()
  _, serialized_example = reader.read(filename_queue)
  features = tf.parse_single_example(
      serialized_example,
      # Defaults are not specified since both keys are required.
      features={
          'image_raw_left': tf.FixedLenFeature([], tf.string),
          'image_raw_right': tf.FixedLenFeature([], tf.string),
          'lidar_label': tf.FixedLenFeature([], tf.string),
      })

  # Convert from a scalar string tensor (whose single string has
  # length mnist.IMAGE_PIXELS) to a uint8 tensor with shape
  # [mnist.IMAGE_PIXELS].
  image_left = tf.decode_raw(features['image_raw_left'], tf.uint8)
  image_right = tf.decode_raw(features['image_raw_right'], tf.uint8)
  lidar = tf.decode_raw(features['lidar_label'], tf.float64)
  print(lidar)

  image_left = tf.reshape(image_left, [ORIGINAL_IMAGE_HEIGHT, ORIGINAL_IMAGE_WIDTH, IMAGE_DEPTH])
  image_right = tf.reshape(image_right, [ORIGINAL_IMAGE_HEIGHT, ORIGINAL_IMAGE_WIDTH, IMAGE_DEPTH])
  lidar = tf.reshape(lidar, [LIDAR_DIM])
  # Convert from uint8 to float32
  image_left = tf.cast(image_left, tf.float32)
  image_right = tf.cast(image_right, tf.float32)
  lidar = tf.cast(lidar, tf.float32)

  # # Image processing for evaluation.
  # # Crop the central [height, width] of the image.
  resized_image_left = tf.image.resize_images(image_left, [IMAGE_HEIGHT, IMAGE_WIDTH])
  resized_image_right = tf.image.resize_images(image_right, [IMAGE_HEIGHT, IMAGE_WIDTH])
  print(lidar)
  # # Convert label from a scalar uint8 tensor to an int32 scalar.
  # label = tf.cast(features['lidar_label'], tf.int32)

  return resized_image_left, resized_image_right, lidar


def _generate_image_and_label_batch(left_image, right_image, lidar, min_queue_examples, batch_size, shuffle):
  """Construct a queued batch of images and labels.

  Args:
    left_image: 3-D Tensor of [height, width, 3] of type.float32.
    right_image: 3-D Tensor of [height, width, 3] of type.float32.
    lidar: 1-D Tensor of [lidar_size] type.float32
    min_queue_examples: int32, minimum number of samples to retain
      in the queue that provides of batches of examples.
    batch_size: Number of images per batch.
    shuffle: boolean indicating whether to use a shuffling queue.

  Returns:
    left_image_batch: Images. 4D tensor of [batch_size, height, width, 3] size.
    right_image_batch: Images. 4D tensor of [batch_size, height, width, 3] size.
    lidar: Labels. 2D tensor of [batch_size, lidar_size] size.
  """
  # Create a queue that shuffles the examples
  # min_queue_examples = 100
  num_preprocess_threads = 8
  if shuffle:
    left_batch, right_batch, lidar_batch = tf.train.shuffle_batch(
        [left_image, right_image, lidar],
        batch_size=batch_size,
        num_threads=num_preprocess_threads,
        capacity=min_queue_examples + 3 * batch_size,
        min_after_dequeue=min_queue_examples)
  else:
    left_batch, right_batch, lidar_batch = tf.train.batch(
        [left_image, right_image, lidar],
        batch_size=batch_size,
        num_threads=num_preprocess_threads,
        capacity=min_queue_examples + 3 * batch_size)

  # Display the training images in the visualizer.
  tf.image_summary('left_images', left_batch)
  tf.image_summary('right_images', right_batch)

  return left_batch, right_batch, lidar_batch


def inputs(eval_data, data_dir, batch_size):
  """Construct input for  evaluation using the Reader ops.

  Args:
    eval_data: bool, indicating if one should use the train or eval data set.
    data_dir: Path to the data directory.
    batch_size: Number of images per batch.

  Returns:
    images: Images. 4D tensor of [batch_size, IMAGE_SIZE, IMAGE_SIZE, 3] size.
    labels: Labels. 1D tensor of [batch_size] size.
  """
  if not eval_data:
    filename1 = os.path.join(data_dir, TRAIN1_FILE)
    filename2 = os.path.join(data_dir, TRAIN2_FILE)
    num_examples_per_epoch = NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN
  else:
    filename = os.path.join(data_dir, VALIDATION_FILE)
    num_examples_per_epoch = NUM_EXAMPLES_PER_EPOCH_FOR_EVAL

  with tf.name_scope('input'):
    if not eval_data:
      filename_queue = tf.train.string_input_producer([filename1,filename2], num_epochs=None)
    else:
      filename_queue = tf.train.string_input_producer([filename], num_epochs=None)

    resized_image_left, resized_image_right, lidar = read_and_decode(filename_queue)

    # Ensure that the random shuffling has good mixing properties.
    min_fraction_of_examples_in_queue = 0.4
    min_queue_examples = int(num_examples_per_epoch *
                             min_fraction_of_examples_in_queue)

    # Generate a batch of left_images right_images and labels by building up a queue of examples.
    return _generate_image_and_label_batch(resized_image_left, resized_image_right, lidar, 
      min_queue_examples, batch_size, shuffle=True)
