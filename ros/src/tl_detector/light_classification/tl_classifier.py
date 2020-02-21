from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import rospy


class TLClassifier(object):
    def __init__(self, is_carla):
        # TODO load classifier

        self.cur_ls = TrafficLight.UNKNOWN

        if is_carla:
            PATH_TO_GRAPH = r'light_classification/models/ssd_inc_v2_real_finetune/frozen_inference_graph.pb'
        else:
            PATH_TO_GRAPH = r'light_classification/models/ssd_inc_v2_sim_finetune/frozen_inference_graph.pb'

        self.detection_graph = tf.Graph()
        self.threshold = .5

        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=self.detection_graph)

        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction

        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                feed_dict={self.image_tensor: image_expanded})

            end = datetime.datetime.now()
            c = end - start
            # print('Time: ', c.total_seconds())

        # print('SCORES: ', scores[0, 0])
        # print('CLASSES: ', classes[0, 0])

        if scores[0, 0] > self.threshold:
            if (classes[0, 0] == 1):
                self.cur_ls = TrafficLight.GREEN
                rospy.logwarn("Current light: Green")
            elif (classes[0, 0] == 2):
                self.cur_ls = TrafficLight.RED
                rospy.logwarn("Current light: Red")
            elif (classes[0, 0] == 3):
                self.cur_ls = TrafficLight.YELLOW
                rospy.logwarn("Current light: Yellow")
            else:
                self.cur_ls = TrafficLight.UNKNOWN

        return self.cur_ls

        # return TrafficLight.UNKNOWN
