# Reused load graph code from 
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

import rospy
from styx_msgs.msg import TrafficLight

import numpy as np
import tensorflow as tf


class TLClassifier(object):
    def __init__(self, model_path):
        # TODO load classifier
        self.DETECTION_CONFIDENCE = 0.6
        self.TL_index = {1: TrafficLight.GREEN, 2: TrafficLight.RED, 3: TrafficLight.YELLOW, 4: TrafficLight.UNKNOWN}

        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            ops = tf.get_default_graph().get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            self.tensor_dict = {}
            for key in [
                'num_detections', 'detection_boxes', 'detection_scores', 'detection_classes'
            ]:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    self.tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                        tensor_name)

            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            config = tf.ConfigProto()
            config.gpu_options.allow_growth = True
            self.sess = tf.Session(config=config)

    def get_classification(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(cv_image, axis=0)

        output_dict = self.sess.run(self.tensor_dict,
                                    feed_dict={self.image_tensor: image_np_expanded})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        num_detections = int(output_dict['num_detections'][0])
        detection_classes = output_dict['detection_classes'][0].astype(np.uint8)
        detection_scores = output_dict['detection_scores'][0]

        if detection_scores[0] > self.DETECTION_CONFIDENCE:
            detection_class = detection_classes[0]
            for index in range(num_detections - 1):
                if detection_scores[index] > self.DETECTION_CONFIDENCE:
                    if not detection_classes[index] == detection_class:
                        detection_class = 4  # Unknown
                        break
                else:
                    break
        else:
            detection_class = 4  # Unknown

        return self.TL_index[detection_class]


if __name__ == '__main__':
    try:
        from PIL import Image
        from cv_bridge import CvBridge
        from glob import glob
        import os

        tl_state = TLClassifier(model_path='frozen_inference_graph.pb')
        # tl_state=TLClassifier(model_path='eightbit_graph.pb')
        
        for image_path in glob(os.path.join('sample_images', '*.jpg')):
            print(image_path)
            image = Image.open(image_path)

            (im_width, im_height) = image.size
            image_np = np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

            bridge = CvBridge()
            image_np = bridge.cv2_to_imgmsg(image_np, 'bgr8')
            image_np = bridge.imgmsg_to_cv2(image_np, 'bgr8')

            print(tl_state.get_classification(image_np))
            print("Above 0:RED, 1:YELLOW, 2:GREEN, 4:UNKNOWN \n")

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
