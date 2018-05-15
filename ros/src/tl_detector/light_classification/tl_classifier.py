import rospy
from styx_msgs.msg import TrafficLight

import numpy as np
import tensorflow as tf
import cv2
from PIL import Image

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # Reused code from 
        # https://github.com/tensorflow/models/blob/master/research/object_detection/object_detection_tutorial.ipynb

        CKPT = 'light_classification/frozen_inference_graph.pb'
        PATH_TO_LABELS = 'light_classification/label_map.pbtxt'
        NUM_CLASSES = 14
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(CKPT, 'rb') as fid:
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
            #detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            #detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            #detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            #num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        pass
    
    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def get_classification(self, image_path):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]

        image = Image.open(image_path)
        image_np = self.load_image_into_numpy_array(image)
        image_np_expanded = np.expand_dims(image_np, axis=0)

        with tf.Session(graph=self.detection_graph) as sess:
            output_dict = sess.run(self.tensor_dict,
                             feed_dict={self.image_tensor: image_np_expanded})
            # all outputs are float32 numpy arrays, so convert types as appropriate
            output_dict['num_detections'] = int(output_dict['num_detections'][0])
            output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
            output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
            output_dict['detection_scores'] = output_dict['detection_scores'][0]

            print("num_detections: ",output_dict['num_detections'])
            print("\n")
            print("classes: 1-Green, 2-Yellow, 3-Red, 4-off")
            print("detection_classes: ",output_dict['detection_classes'][0])
            print("detection_boxes: ",output_dict['detection_boxes'][0])
            print("detection_scores: ",output_dict['detection_scores'][0])

        return TrafficLight.GREEN

if __name__ == '__main__':
    try:
        tl_state=TLClassifier()
        print(tl_state.get_classification('light_classification/left0000.jpg'))
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
