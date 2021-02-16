import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Pose, Point
from ament_index_python.packages import get_package_share_directory
from yolact_edge.inference import YOLACTEdgeInference
import numpy as np
import time
import cv2


class YOLACTEdgeDetector(Node):
    ''' use Detectron2 to detect object masks from 2D 
        image and estimate 3D position with Pointcloud2 data
    '''

    def __init__(self):
        super().__init__('yolact_edge_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pointcloud2_topic', "/d435i/camera/pointcloud"),
                ('model_weights_file', "model.trt"),
                # TODO:
                ('pc_downsample_factor', 16),
                ('min_mask', 20),
                ('categories', [0]),
                ('nms_filter', 0.3),
                ('outlier_thresh', 0.5)
            ])

        # TODO: load this model
        # weights_path = os.path.join(
        #     get_package_share_directory('yolact_edge_detector'),
        #     "weights",
        #     self.get_parameter("model_weights_file").value
        # )

        weights = "/home/ernestas/dyn_obs/src/navigation2_dynamic/yolact_edge_detector/weights/yolact_edge_youtubevis_resnet50_847_50000.pth"
        self.model = YOLACTEdgeInference(weights)

        self.pc_downsample_factor = int(
            self.get_parameter("pc_downsample_factor").value)
        self.min_mask = self.get_parameter("min_mask").value
        self.categories = self.get_parameter("categories").value
        self.nms_filter = self.get_parameter("nms_filter").value
        self.outlier_thresh = self.get_parameter("outlier_thresh").value

        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud2_topic").value,
            self.pcloud_callback, 1)

        self.detect_obj_pub = self.create_publisher(
            ObstacleArray, 'detection', 2)
        self.detect_img_pub = self.create_publisher(Image, 'image', 2)

    def outlier_filter(self, x, y, z, idx):
        ''' simple outlier filter, assume Gaussian distribution and 
            drop points with low probability (too far away from center)
        '''
        mean = [np.mean(x), np.mean(y), np.mean(z)]
        cov = np.diag([np.var(x), np.var(y), np.var(z)])
        rv = np.random.multivariate_normal(
            mean, cov)  # TODO: test numpy version
        points = np.dstack((x, y, z))
        p = rv.pdf(points)
        return idx[p > self.outlier_thresh]

    def pcloud_callback(self, msg):
        # TODO:
        # if self.detect_obj_pub.get_subscription_count() == 0 and self.detect_img_pub.get_subscription_count() == 0:
        #     return

        # TODO: project non ordered pcloud?

        height = msg.height
        width = msg.width
        points = np.array(msg.data, dtype='uint8')
        rgb_offset = msg.fields[3].offset
        point_step = msg.point_step
        r = points[rgb_offset::point_step]
        g = points[(rgb_offset+1)::point_step]
        b = points[(rgb_offset+2)::point_step]
        img = np.concatenate([r[:, None], g[:, None], b[:, None]], axis=-1)
        img = img.reshape((height, width, 3))

        # TODO: only 5Hz ??? while model can perform at 50Hz
        pred = self.model.predict(img, False)

        # TODO: stops functioning after some time

        if pred == None:
            return

        # # TODO: implement
        # detections = self.detect(img)

        # # decode point cloud data TODO: separate function
        # if msg.fields[0].datatype < 3:
        #     byte = 1
        # elif msg.fields[0].datatype < 5:
        #     byte = 2
        # elif msg.fields[0].datatype < 8:
        #     byte = 4
        # else:
        #     byte = 8

        # points = points.view('<f' + str(byte))
        # x = points[0::int(self.pc_downsample_factor * point_step / byte)]
        # y = points[1::int(self.pc_downsample_factor * point_step / byte)]
        # z = points[2::int(self.pc_downsample_factor * point_step / byte)]
        # points = [x, y, z]

        # # process pointcloud to get 3D position and size TODO: implement
        # obstacles = self.process_points(detections, points)

        # # publish detection result
        # obstacle_array = ObstacleArray()
        # obstacle_array.header = msg.header
        # if self.detect_obj_pub.get_subscription_count() > 0:
        #     obstacle_array.obstacles = obstacles
        #     self.detect_obj_pub.publish(obstacle_array)

        # visualize detection
        # if self.detect_img_pub.get_subscription_count() > 0:
        out_img = pred["img"]
        out_img_msg = Image()
        out_img_msg.header = msg.header
        out_img_msg.height = out_img.shape[0]
        out_img_msg.width = out_img.shape[1]
        out_img_msg.encoding = 'rgb8'
        out_img_msg.step = 3 * out_img.shape[1]
        out_img_msg.data = out_img.flatten().tolist()
        self.detect_img_pub.publish(out_img_msg)

    def process_points(self, detections, points):
        ''' estimate 3D positions
        '''
        pass

        # TODO: this will change in accordace with predictions output

        # x, y, z = self.points

        # # map mask to point cloud data
        # num_classes = outputs['instances'].pred_classes.shape[0]
        # if num_classes == 0:
        #     self.detect_obj_pub.publish(ObstacleArray())
        #     return

        # masks = outputs["instances"].pred_masks.cpu().numpy().astype(
        #     'uint8').reshape((num_classes, -1))[:, ::self.pc_downsample_factor]
        # scores = outputs["instances"].scores.cpu().numpy().astype(np.float)

        # # estimate 3D position with simple averaging of obstacle's points
        # detections = []
        # for i in range(num_classes):
        #     # if user does not specify any interested category, keep all; else select those interested objects
        #     if (len(self.categories) == 0) or (outputs["instances"].pred_classes[i] in self.categories):
        #         idx = np.where(masks[i])[0]
        #         idx = self.outlier_filter(x[idx], y[idx], z[idx], idx)
        #         if idx.shape[0] < self.min_mask:
        #             continue
        #         obstacle_msg = Obstacle()
        #         # pointcloud2 data has a different coordinate, swap y and z
        #         # use (max+min)/2 can avoid the affect of unbalance of points density instead of average
        #         x_max = x[idx].max()
        #         x_min = x[idx].min()
        #         y_max = y[idx].max()
        #         y_min = y[idx].min()
        #         z_max = z[idx].max()
        #         z_min = z[idx].min()
        #         obstacle_msg.score = scores[i]
        #         obstacle_msg.position.x = np.float((x_max + x_min) / 2)
        #         obstacle_msg.position.y = np.float((y_max + y_min) / 2)
        #         obstacle_msg.position.z = np.float((z_max + z_min) / 2)
        #         obstacle_msg.size.x = np.float(x_max - x_min)
        #         obstacle_msg.size.y = np.float(y_max - y_min)
        #         obstacle_msg.size.z = np.float(z_max - z_min)
        #         detections.append(obstacle_msg)

        # return detections

    def detect(self, img):
        pass
        # TODO: implement
        # return self.predictor(img)


def main():
    rclpy.init(args=None)
    node = YOLACTEdgeDetector()
    node.get_logger().info("start spining detectron_node...")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
