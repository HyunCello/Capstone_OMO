#!/usr/bin/env python
# object_detector.py

import cv2
import os
import numpy as np
import tensorflow as tf
import tarfile
import six.moves.urllib as urllib
import time
import rospy

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from object_detection.utils import ops as utils_ops
from std_msgs.msg import String

topic_msg=''

class ObjectDetector():
    DOWNLOAD_BASE = 'http://download.tensorflow.org/models/object_detection/'
    GRAPH_FILE_NAME = 'frozen_inference_graph.pb'
    NUM_CLASSES = 3

    def download_model(self, model_name):
        model_file = model_name + '.tar.gz'
        print("downloading model", model_name, "...")
        opener = urllib.request.URLopener()
        opener.retrieve(self.DOWNLOAD_BASE + model_file, model_file)
        print("download completed");
        tar_file = tarfile.open(model_file)
        for file in tar_file.getmembers():
            file_name = os.path.basename(file.name)
            if self.GRAPH_FILE_NAME in file_name:
                tar_file.extract(file, os.getcwd())
                print(self.graph_file, "is extracted");

    def __init__(self, model_name, label_file='data/mscoco_label_map.pbtxt'):
        # Initialize some variables
        print("ObjectDetector('%s', '%s')" % (model_name, label_file))
        self.process_this_frame = True

        # download model
        self.graph_file = model_name + '/' + self.GRAPH_FILE_NAME
        if not os.path.isfile(self.graph_file):
            self.download_model(model_name)

        # Load a (frozen) Tensorflow model into memory.
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            graph = self.detection_graph

            ops = graph.get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            tensor_dict = {}
            for key in [
                  'num_detections', 'detection_boxes', 'detection_scores',
                  'detection_classes', 'detection_masks'
              ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    tensor_dict[key] = graph.get_tensor_by_name(tensor_name)

            if 'detection_masks' in tensor_dict:
                # The following processing is only for single image
                detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
                detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
                # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
                real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
                detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
                detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
                detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                    detection_masks, detection_boxes, 480, 640)
                detection_masks_reframed = tf.cast(
                    tf.greater(detection_masks_reframed, 0.5), tf.uint8)
                # Follow the convention by adding back the batch dimension
                tensor_dict['detection_masks'] = tf.expand_dims(
                    detection_masks_reframed, 0)


            self.tensor_dict = tensor_dict




        self.sess = tf.Session(graph=self.detection_graph)

        # Loading label map
        # Label maps map indices to category names,
        # so that when our convolution network predicts `5`,
        # we know that this corresponds to `airplane`.
        # Here we use internal utility functions,
        # but anything that returns a dictionary mapping integers to appropriate string labels would be fine
        label_map = label_map_util.load_labelmap(label_file)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=self.NUM_CLASSES, use_display_name=True)

        # for key in detection_boxes.items():
        #     print(key, ":", value)

        self.category_index = label_map_util.create_category_index(categories)
        self.output_dict = None

        self.last_inference_time = 0

    def run_inference(self, image_np):
        sess = self.sess
        graph = self.detection_graph
        with graph.as_default():
            image_tensor = graph.get_tensor_by_name('image_tensor:0')

            # Run inference
            output_dict = sess.run(self.tensor_dict,
                                 feed_dict={image_tensor: np.expand_dims(image_np, 0)})

            # all outputs are float32 numpy arrays, so convert types as appropriate
            output_dict['num_detections'] = int(output_dict['num_detections'][0])
            output_dict['detection_classes'] = output_dict[
              'detection_classes'][0].astype(np.uint8)
            output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
            output_dict['detection_scores'] = output_dict['detection_scores'][0]
            if 'detection_masks' in output_dict:
                output_dict['detection_masks'] = output_dict['detection_masks'][0]

        return output_dict

    def time_to_run_inference(self):

        unixtime = int(time.time())
        if self.last_inference_time != unixtime:
            self.last_inference_time = unixtime
            return True
        return False

    def detect_objects(self, frame):
        global topic_msg
        time1 = time.time()
        # Grab a single frame of video

        # Resize frame of video to 1/4 size for faster face recognition processing
        small_frame = cv2.resize(frame, (0, 0), fx=0.1, fy=0.1)
        #small_frame = frame

        # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
        rgb_small_frame = small_frame[:, :, ::-1]

        time2 = time.time()

        # Only process every other frame of video to save time
        if self.time_to_run_inference():
            self.output_dict = self.run_inference(rgb_small_frame)

        time3 = time.time()

        vis_util.visualize_boxes_and_labels_on_image_array(
          frame,
          self.output_dict['detection_boxes'],
          self.output_dict['detection_classes'],
          self.output_dict['detection_scores'],
          self.category_index,

          instance_masks=self.output_dict.get('detection_masks'),
          use_normalized_coordinates=True,
          line_thickness=5          
        )
        
        classes = self.output_dict['detection_classes']
        boxes = self.output_dict['detection_boxes']


        for i in range(len(boxes)):
            #print('classes: ', classes[i])
            #print('boxes: ', boxes[i])
            height = (boxes[0][2] - boxes[0][0]) * 480
            width= (boxes[0][3] - boxes[0][1]) * 640
            area= width * height
            #print("area: %0.1f" % area )
            mid_height = (boxes[0][2] + boxes[0][0])/2 * 480
            mid_width = (boxes[0][3] + boxes[0][1])/2 * 640

            # print('mid_width: ', mid_width)
            # print('mid_height: ', mid_height)
            if (classes == 1).any():

                if (area > 130000) :
                    print('Person too close !!')
                    topic_msg = 'Stop'

                elif (mid_width > 320) & (area > 80000) :
                    print('Turn left to avoid person')
                    topic_msg = 'Left'

                elif (mid_width < 320) & (area > 80000) :
                    print('Turn right to avoid person')
                    topic_msg = 'Right'

                else :
                    print("person detected & area: %0.1f" % area )
                    topic_msg = 'Keep going'


        time4 = time.time()

        #print("%0.1f, %0.1f, %0.1f sec" % (time2 - time1, time3 - time2, time4 - time3))
        return frame

    def get_jpg_bytes(self):
        frame = self.get_frame()
        # We are using Motion JPEG, but OpenCV defaults to capture raw images,
        # so we must encode it into JPEG in order to correctly display the
        # video stream.
        ret, jpg = cv2.imencode('.jpg', frame)
        return jpg.tobytes()


if __name__ == '__main__':
    import camera
    from object_detection.utils import visualization_utils as vis_util

    #global topic_msg

    detector = ObjectDetector('ssd_mobilenet_v1_coco_2017_11_17')
    pub = rospy.Publisher('lidar_obstacle', String, queue_size=10)
    rospy.init_node('hail_detect', anonymous=True)
    rate = rospy.Rate(30)

    #detector = ObjectDetector('mask_rcnn_inception_v2_coco_2018_01_28')
    #detector = ObjectDetector('pet', label_file='data/pet_label_map.pbtxt')

    # Using OpenCV to capture from device 0. If you have trouble capturing
    # from a webcam, comment the line below out and use a video file
    # instead.
    cam = camera.VideoCamera()

    print("press `q` to quit")
    while not rospy.is_shutdown():
        frame = cam.get_frame()
        frame = detector.detect_objects(frame)
        pub.publish(topic_msg)

        # show the frame

        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        #vis_util.xmax-xmin

        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break
        rate.sleep()
    # do a bit of cleanup
    cv2.destroyAllWindows()
    print('finish')