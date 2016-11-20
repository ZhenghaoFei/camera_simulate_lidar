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
NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN = 1
NUM_EXAMPLES_PER_EPOCH_FOR_EVAL = 1

IMAGE_HEIGHT = 8
IMAGE_WIDTH = 128

ORIGINAL_IMAGE_HEIGHT = 32
ORIGINAL_IMAGE_WIDTH = 672
IMAGE_DEPTH = 3

def get_filename_queue(file_numbers, data_dir):
  # reconstruce filename for three source
  left_image_filename_list = []
  right_image_filename_list = []
  lidar_filename_list = []

  for i in range(len(file_numbers)):
    left_image_filename = data_dir +  file_numbers[i] + "left.png"
    right_image_filename = data_dir +  file_numbers[i] + "right.png"
    lidar_filename = data_dir + file_numbers[i] + ".txt"
    left_image_filename_list.append(left_image_filename)
    right_image_filename_list.append(right_image_filename)
    lidar_filename_list.append(lidar_filename)

  # list to tensor queue
  left_image_filename_queue = tf.train.string_input_producer(left_image_filename_list)
  right_image_filename_queue = tf.train.string_input_producer(right_image_filename_list)
  lidar_filename_queue = tf.train.string_input_producer(lidar_filename_list)

  return [left_image_filename_queue, right_image_filename_queue, lidar_filename_queue]

def read_data(left_image_filename_queue, right_image_filename_queue, lidar_filename_queue):


  # creat three class 
  class Left_Image_Record(object):
    pass
  Left_Image = Left_Image_Record()

  class Right_Image_Record(object):
    pass
  Right_Image = Right_Image_Record()

  class Lidar_Record(object):
    pass
  Lidar = Lidar_Record()

  # read and decode image    
  image_reader = tf.WholeFileReader()
  Left_Image.key, left_value = image_reader.read(left_image_filename_queue)
  # Left_Image.image = tf.image.decode_png(left_value)
  Left_Image.uint8image  = tf.reshape(tf.image.decode_png(left_value), [ORIGINAL_IMAGE_HEIGHT, ORIGINAL_IMAGE_WIDTH, IMAGE_DEPTH])

  Right_Image.key, right_value = image_reader.read(right_image_filename_queue)
  # Right_Image.image = tf.image.decode_png(right_value)
  Right_Image.uint8image  = tf.reshape(tf.image.decode_png(right_value), [ORIGINAL_IMAGE_HEIGHT, ORIGINAL_IMAGE_WIDTH, IMAGE_DEPTH])

  # read and decode lidar    
  lidar_reader = tf.TextLineReader()
  Lidar.key, lidar_value = lidar_reader.read(lidar_filename_queue)
  record_defaults = [[] for col in range(120)]
  lidar_samples = tf.decode_csv(lidar_value, record_defaults=record_defaults)
  Lidar.info = tf.pack(lidar_samples)

  return [Left_Image, Right_Image, Lidar]



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
  num_preprocess_threads = 16
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

# only get serial number
def get_filenumber(mypath):
    # mypath = './data/data1/'
    data_lidar = sorted(glob.glob(mypath + '*txt'), key=os.path.basename)
    print (mypath)

    file_numbers = []
    for i in range(len(data_lidar)):
        number = data_lidar[i]
        number = number.replace(mypath,"")
        number = number.replace(".txt","")
        file_numbers.append(number)
    print ("number of sample: ", len(file_numbers))
    return file_numbers


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
    file_numbers = get_filenumber(data_dir)
    num_examples_per_epoch = NUM_EXAMPLES_PER_EPOCH_FOR_TRAIN
  else:
    file_numbers = get_filenumber(data_dir) # need change later
    num_examples_per_epoch = NUM_EXAMPLES_PER_EPOCH_FOR_EVAL
  # print (data_dir)
  # print (file_numbers) 

  # for f in filenames:
  #   if not tf.gfile.Exists(f):
  #     raise ValueError('Failed to find file: ' + f)

  # Create a queue that produces the file_numbers to read.

  left_image_filename_queue, right_image_filename_queue, lidar_filename_queue = get_filename_queue(file_numbers, data_dir)

  # Read examples from files in the filename queue.
  Left_Image, Right_Image, Lidar = read_data(left_image_filename_queue, right_image_filename_queue, lidar_filename_queue)
  left_image = tf.cast(Left_Image.uint8image, tf.float32)
  right_image = tf.cast(Right_Image.uint8image, tf.float32)

  # height = IMAGE_SIZE
  # width = IMAGE_SIZE

  # # Image processing for evaluation.
  # # Crop the central [height, width] of the image.
  height = IMAGE_HEIGHT = 32
  width = IMAGE_WIDTH = 128
  resized_left_image = tf.image.resize_images(left_image, [height, width])
  resized_right_image = tf.image.resize_images  (right_image, [height, width])

  # # Subtract off the mean and divide by the variance of the pixels.
  # float_image = tf.image.per_image_whitening(resized_image)

  # Ensure that the random shuffling has good mixing properties.
  min_fraction_of_examples_in_queue = 0.4
  min_queue_examples = int(num_examples_per_epoch *
                           min_fraction_of_examples_in_queue)

  # Generate a batch of left_images right_images and labels by building up a queue of examples.
  return _generate_image_and_label_batch(resized_left_image, resized_right_image, Lidar.info, 
    min_queue_examples, batch_size, shuffle=False)
