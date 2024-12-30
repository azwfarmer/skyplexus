import time
import torch
import numpy as np
import ros2_numpy
import rclpy
import cv2
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

class MapGenOld(Node):

    def __init__(self):
        super().__init__('map_gen_old')
        qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.subscription_tf = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        self.subscription_tf

        self.subscription_lidar = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.points_callback,
            qos_profile)

        self.subscription_lidar
        
        self.subscription_camera = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.subscription_camera

        self.publication_camera = self.create_publisher(Image, '/overlay', 10)

        self.image = Image()

        self.device = torch.device("cuda")
        self.lidar_to_camera_extrinsic = torch.empty((3,4), device=self.device)
        self.intrinsic = torch.tensor([[1252.348876953125, 0, 971.9642944335938], 
                                      [0, 1252.5963134765625, 542.4281616210938], 
                                      [0, 0, 1]], device=self.device)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_to_camera_extrinsic_received = False
        self.img_received = False

    def tf_callback(self, tf_msg):
        # for msg in tf_msg.transforms:
        #     if msg.child_frame_id == 'base_link' and msg.header.frame_id == 'map':
        #         from_frame = 'os_sensor'
        #         to_frame = 'camera_color_optical_frame'
                
        #         try:
        #             lidar_to_camera_frame_tf = self.tf_buffer.lookup_transform(
        #                 to_frame,
        #                 from_frame,
        #                 rclpy.time.Time())
                    
        #         except TransformException:
        #             return

        #         homogeneous_matrix = ros2_numpy.numpify(lidar_to_camera_frame_tf.transform)[:3]
        #         self.get_logger().info(f'cam \n{homogeneous_matrix}')

        #         self.lidar_to_camera_extrinsic = torch.tensor(homogeneous_matrix, dtype=torch.float32, requires_grad=True, device=self.device)
        #         if(self.lidar_to_camera_extrinsic_received != True):
        #             self.lidar_to_camera_extrinsic_received = True
        #         break
        try:
            lidar_to_camera_frame_tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link  ',
                rclpy.time.Time())            
        except TransformException:
            return
        homogeneous_matrix = ros2_numpy.numpify(lidar_to_camera_frame_tf.transform)[:3]
        self.get_logger().info(f'???? \n{homogeneous_matrix}')

    def hsv_to_rgb(self, h, s, v):
        fC = v * s
        fHPrime = h * 6.0 % 6
        fX = fC * (1 - torch.abs(fHPrime % 2 - 1))
        fM = v - fC
    
        fR = torch.zeros_like(h, device=self.device)
        fG = torch.zeros_like(h, device=self.device)
        fB = torch.zeros_like(h, device=self.device)

        mask0 = ((0 <= fHPrime) & (fHPrime < 1))
        mask1 = ((1 <= fHPrime) & (fHPrime < 2))
        mask2 = ((2 <= fHPrime) & (fHPrime < 3))
        mask3 = ((3 <= fHPrime) & (fHPrime < 4))
        mask4 = ((4 <= fHPrime) & (fHPrime < 5))
        mask5 = ((5 <= fHPrime) & (fHPrime < 6))

        fR[mask0] = fC[mask0]
        fG[mask0] = fX[mask0]
        fB[mask0] = 0

        fR[mask1] = fX[mask1]
        fG[mask1] = fC[mask1]
        fB[mask1] = 0

        fR[mask2] = 0
        fG[mask2] = fC[mask2]
        fB[mask2] = fX[mask2]

        fR[mask3] = 0
        fG[mask3] = fX[mask3]
        fB[mask3] = fC[mask3]

        fR[mask4] = fX[mask4]
        fG[mask4] = 0
        fB[mask4] = fC[mask4]

        fR[mask5] = fC[mask5]
        fG[mask5] = 0
        fB[mask5] = fX[mask5]

        fB += fM
        fR += fM;
        fG += fM;

        fR = torch.clamp(fR * 255, 0, 255).to(torch.uint8)
        fG = torch.clamp(fG * 255, 0, 255).to(torch.uint8)
        fB = torch.clamp(fB * 255, 0, 255).to(torch.uint8)

        rgb = torch.stack((fR, fG, fB), dim=1).squeeze()
        return rgb

    def project_points(self, pcl_msg):
        raw_points_dict = ros2_numpy.numpify(pcl_msg)
        raw_points = torch.tensor(raw_points_dict['xyz'], device=self.device, requires_grad=True).T
        raw_intensity = torch.tensor((raw_points_dict['intensity'] / 65535.0).transpose(), dtype=torch.float32, device=self.device, requires_grad=True)

        not_nan_mask = torch.isfinite(raw_points).all(axis=0)
        intensity = raw_intensity.T[not_nan_mask]
        points_xyz = raw_points[:, not_nan_mask]
        points_xyz = torch.vstack((points_xyz, torch.ones((1, points_xyz.shape[1]), device=self.device)))

        proj_pts = self.intrinsic @ self.lidar_to_camera_extrinsic @ points_xyz
        proj_pts = (proj_pts / proj_pts[2, :].unsqueeze(0))[0:2]

        cam_mask = (proj_pts[0, :] > 1) & (proj_pts[0, :] < 1918) & (proj_pts[1, :] > 1) & (proj_pts[1, :] < 1078)
        proj_pts_in_frame = proj_pts[:, cam_mask]
        filtered_intensity = intensity[cam_mask]

        return proj_pts_in_frame, filtered_intensity

    def display_image(self,filtered_points, filtered_intensity):
        x_coords = torch.round(filtered_points[0, :]).to(torch.int32)
        y_coords = torch.round(filtered_points[1, :]).to(torch.int32)

        offsets = torch.tensor([[-1, 0, 1]], device=self.device)
        X_offsets, Y_offsets = torch.meshgrid(offsets.squeeze(), offsets.squeeze())

        X_flat = (x_coords[:, None] + X_offsets.flatten()).flatten()
        Y_flat = (y_coords[:, None] + Y_offsets.flatten()).flatten()

        sv = torch.ones((filtered_intensity.shape[0],1), device=self.device)
        colors = self.hsv_to_rgb(filtered_intensity, sv, sv)
        colors_expanded = colors.unsqueeze(1).repeat(1, 9, 1).view(-1, 3) 

        self.image[Y_flat.cpu().numpy(), X_flat.cpu().numpy()] = colors_expanded.cpu().numpy()
        self.image = self.image[:,:,[2,1,0]]
        cv2.imshow("overlay", self.image)
        cv2.waitKey(1)

    def points_callback(self, msg):
        if self.lidar_to_camera_extrinsic_received and self.img_received:
            filtered_points, filtered_intensity = self.project_points(msg)
            # self.display_image(filtered_points, filtered_intensity)

    def image_callback(self, msg):
        self.image = ros2_numpy.numpify(msg)
        if self.img_received != True:
            self.img_received = True

def main(args=None):
    rclpy.init(args=args)
    map_gen_old = MapGenOld()
    rclpy.spin(map_gen_old)
    map_gen_old.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()