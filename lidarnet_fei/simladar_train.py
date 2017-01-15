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

from datetime import datetime
import os.path
import time

import numpy as np
from six.moves import xrange  # pylint: disable=redefined-builtin
import tensorflow as tf

import simladar

FLAGS = tf.app.flags.FLAGS

tf.app.flags.DEFINE_string('train_dir', '../data/data_train/',
                           """Directory where to write event logs """
                           """and checkpoint.""")
tf.app.flags.DEFINE_string('save_dir', './save/',
                           """Directory where to write event logs """
                           """and checkpoint.""")
tf.app.flags.DEFINE_integer('max_steps', 1000000,
                            """Number of batches to run.""")
tf.app.flags.DEFINE_boolean('log_device_placement', False,
                            """Whether to log device placement.""")

DROPOUT_PROB = 0.75 # Dropout, probability to keep units


def do_eval(sess):
  """Runs one evaluation against the full epoch of data.

  Args:
    sess: The session in which the model has been trained.
    eval_correct: The Tensor that returns the number of correct predictions.
 
  """
  left_batch_eval, right_batch_eval, lidar_batch_eval = simladar.inputs(eval_data=True)
  logits_eval = simladar.inference(left_batch_eval, right_batch_eval)
  loss_eval = simladar.eval_loss(logits_eval, lidar_batch_eval)

  # And run one epoch of eval.
  rmse = sess.run(loss_eval)
  return rmse

def train():
  """Train CIFAR-10 for a number of steps."""
  with tf.Graph().as_default():
    global_step = tf.Variable(0, trainable=False)

    # Get images and labels .
    left_batch, right_batch, lidar_batch = simladar.inputs()
    keep_prob = tf.constant(DROPOUT_PROB) #dropout (keep probability)

    # Build a Graph that computes the logits predictions from the
    # inference model.
    logits = simladar.inference(left_batch, right_batch, keep_prob)

    # Calculate loss.
    loss = simladar.loss(logits, lidar_batch)

    # Build a Graph that trains the model with one batch of examples and
    # updates the model parameters.
    train_op = simladar.train(loss, global_step)

    # Create a saver.
    saver = tf.train.Saver(tf.global_variables())

    # Build the summary operation based on the TF collection of Summaries.
    summary_op = tf.summary.merge_all()

    # Build an initialization operation to run below.
    init = tf.global_variables_initializer()

    # Start running operations on the Graph.
    config = tf.ConfigProto()
    config.gpu_options.allow_growth = True
    config.log_device_placement = FLAGS.log_device_placement
    sess = tf.Session(config=config)
    sess.run(init)

    # load model
    checkpoint = tf.train.get_checkpoint_state(FLAGS.save_dir)

    if checkpoint and checkpoint.model_checkpoint_path:
      saver.restore(sess, checkpoint.model_checkpoint_path)
      print ("Successfully loaded:", checkpoint.model_checkpoint_path)
      # print("global step: ", global_step.eval())
    else:
      print ("Could not find old network weights")

    # Start the queue runners.
    tf.train.start_queue_runners(sess=sess)

    # summary_writer = tf.train.SummaryWriter(FLAGS.save_dir, sess.graph)
    summary_writer = tf.summary.FileWriter(FLAGS.save_dir, sess.graph)
    for step in xrange(FLAGS.max_steps):
      start_time = time.time()
      _, loss_value = sess.run([train_op, loss])
      duration = time.time() - start_time

      assert not np.isnan(loss_value), 'Model diverged with loss = NaN'

      if step % 10 == 0:
        num_examples_per_step = FLAGS.batch_size
        examples_per_sec = num_examples_per_step / duration
        sec_per_batch = float(duration)

        format_str = ('%s: step %d, loss = %.2f (%.1f examples/sec; %.3f '
                      'sec/batch)')
        print (format_str % (datetime.now(), step, loss_value,
                             examples_per_sec, sec_per_batch))

      if step % 100 == 0:
        # eval_rmse = do_eval(sess)
        # print('eval rmse: ',eval_rmse )
        summary_str = sess.run(summary_op)
        summary_writer.add_summary(summary_str, step)

      # Save the model checkpoint periodically.
      if step % 1000 == 0 or (step + 1) == FLAGS.max_steps:
        checkpoint_path = os.path.join(FLAGS.save_dir, 'model.ckpt')
        saver.save(sess, checkpoint_path, global_step=step)


def main(argv=None):  # pylint: disable=unused-argument
  train()


if __name__ == '__main__':
  tf.app.run()
