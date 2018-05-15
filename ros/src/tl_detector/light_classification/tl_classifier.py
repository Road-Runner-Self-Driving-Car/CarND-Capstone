# Reused most code from 
# https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

import rospy
from styx_msgs.msg import TrafficLight

import numpy as np
import tensorflow as tf
import cv2
from PIL import Image
from cv_bridge import CvBridge

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        self.DETECTION_CONFIDENCE = 0.7
        self.model_path = 'light_classification/frozen_inference_graph.pb'
        self.bridge = CvBridge()

    def load_graph(self):
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.model_path, 'rb') as fid:
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
    

    def get_classification(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        self.load_graph()
        
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(cv_image, axis=0)

        with tf.Session(graph=self.detection_graph) as sess:
            output_dict = sess.run(self.tensor_dict,
                             feed_dict={self.image_tensor: image_np_expanded})
            # all outputs are float32 numpy arrays, so convert types as appropriate
            num_detections = int(output_dict['num_detections'][0])
            detection_classes = output_dict['detection_classes'][0][0].astype(np.uint8)
            detection_boxes = output_dict['detection_boxes'][0][0]
            detection_scores = output_dict['detection_scores'][0][0]

            print("num_detections: ",num_detections)
            print("\n")
            print("classes: 1-Green, 2-Yellow, 3-Red, 4-off")
            print("detection_classes: ",detection_classes)
            print("detection_boxes: ",detection_boxes)
            print("detection_scores: ",detection_scores)

            if detection_scores > self.DETECTION_CONFIDENCE:
                if detection_classes == 1:
                    return TrafficLight.GREEN
                if detection_classes == 2:
                    return TrafficLight.YELLOW
                if detection_classes == 3:
                    return TrafficLight.RED
            return TrafficLight.UNKNOWN
            

if __name__ == '__main__':
    try:
        tl_state=TLClassifier()
        tl_state.model_path='frozen_inference_graph.pb'
        image = Image.open('sample_images/left0000.jpg')
        
        (im_width, im_height) = image.size
        image_np = np.array(image.getdata()).reshape((im_height, im_width, 3)).astype(np.uint8)

        image_np = tl_state.bridge.cv2_to_imgmsg(image_np, 'bgr8')
        image_np = tl_state.bridge.imgmsg_to_cv2(image_np, 'bgr8')
        print(tl_state.get_classification(image_np))

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
