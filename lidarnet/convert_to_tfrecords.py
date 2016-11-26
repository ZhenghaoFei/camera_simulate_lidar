# Copyright 2015 Google Inc. All Rights Reserved.
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

"""Converts MNIST data to TFRecords file format with Example protos."""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import os
import glob
import numpy as np
import tensorflow as tf
from scipy import misc


tf.app.flags.DEFINE_string('directory', '../data/data_train/',
                           'Directory to download data files and write the '
                           'converted result')
tf.app.flags.DEFINE_string('directory_valid', '../data/data_valid/',
                           'Directory to download data files and write the '
                           'converted result')
tf.app.flags.DEFINE_integer('validation_size', 100,
                            'Number of examples to separate from the training '
                            'data for the validation set.')
FLAGS = tf.app.flags.FLAGS


def _int64_feature(value):
  return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))

def _float_feature(value):
  return tf.train.Feature(float_list=tf.train.FloatList(value=[value]))

def _bytes_feature(value):
  return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))

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

def get_filename_list(file_numbers, data_dir):
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

  return [left_image_filename_list, right_image_filename_list, lidar_filename_list]

def image_filename_list_to_nparray(filename_list):
    image_nparray = []
    for k in filename_list:
        image_nparray.append(misc.imread(k))
    image_nparray = np.asarray(image_nparray)
#     print(image_nparray.shape)
    return image_nparray

def lidar_filename_list_to_nparray(filename_list):
    lidar_nparray = []
    for k in filename_list:
        lidar_nparray.append(np.loadtxt(k, delimiter=','))
    lidar_nparray = np.asarray(lidar_nparray)
    # print(lidar_nparray.shape)
    return lidar_nparray

def convert_to(left_images, right_images, lidar_labels, name):
  num_examples = lidar_labels.shape[0]
  if left_images.shape[0] != num_examples:
    raise ValueError("Images size %d does not match label size %d." %
                     (left_images.shape[0], num_examples))

  rows_left = left_images.shape[1]
  cols_left = left_images.shape[2]
  depth_left = left_images.shape[3]

  rows_right = left_images.shape[1]
  cols_right = left_images.shape[2]
  depth_right = left_images.shape[3]

  filename = os.path.join(FLAGS.directory, name + '.tfrecords')
  print('Writing', filename)
  writer = tf.python_io.TFRecordWriter(filename)
  for index in range(num_examples):
    image_raw_left = left_images[index].tostring()
    image_raw_right = right_images[index].tostring()
    lidar_raw = lidar_labels[index].tostring()
    example = tf.train.Example(features=tf.train.Features(feature={
        'height_left': _int64_feature(rows_left),
        'width_left': _int64_feature(cols_left),
        'depth_left': _int64_feature(depth_left),
        'height_right': _int64_feature(rows_right),
        'width_right': _int64_feature(cols_right),
        'depth_right': _int64_feature(depth_right),
        'image_raw_left': _bytes_feature(image_raw_left),
        'image_raw_right': _bytes_feature(image_raw_right),
        'lidar_label': _bytes_feature(lidar_raw)
        }))
    writer.write(example.SerializeToString())


def main(argv):
  # Get the data.
  file_numbers = get_filenumber(FLAGS.directory)
  file_numbers_valid = get_filenumber(FLAGS.directory_valid)

  left_image_filename_list, right_image_filename_list, lidar_filename_list = get_filename_list(file_numbers, FLAGS.directory)
  left_image_filename_list_valid, right_image_filename_list_valid, lidar_filename_list_valid = get_filename_list(file_numbers_valid, FLAGS.directory_valid)
  # Extract it into numpy arrays.
  left_images = image_filename_list_to_nparray(left_image_filename_list)
  right_images = image_filename_list_to_nparray(right_image_filename_list)
  lidar_labels = lidar_filename_list_to_nparray(lidar_filename_list)

  left_images_valid = image_filename_list_to_nparray(left_image_filename_list_valid)
  right_images_valid = image_filename_list_to_nparray(right_image_filename_list_valid)
  lidar_labels_valid = lidar_filename_list_to_nparray(lidar_filename_list_valid)

  # Convert to Examples and write the result to TFRecords.
  convert_to(left_images, right_images, lidar_labels, 'train')
  convert_to(left_images_valid, right_images_valid, lidar_labels_valid, 'validate')

if __name__ == '__main__':
  tf.app.run()