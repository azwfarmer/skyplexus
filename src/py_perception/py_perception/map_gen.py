import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import ros2_numpy
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import torch


class MapGen(Node):

    def __init__(self):
        super().__init__('map_gen')
        torch.set_printoptions(sci_mode=False)
        np.set_printoptions(suppress=True)
        torch.set_printoptions(threshold=10000)

        self.device = torch.device('cuda')

        # Grid bounds in cell indices, grown on the fly as points land outside
        self.min_x = -100
        self.min_y = -100
        self.max_x = 100
        self.max_y = 100

        self.cells_x = self.max_x - self.min_x + 1
        self.cells_y = self.max_y - self.min_y + 1

        # Per-cell running sums of (z, r, g, b) and point counts, mean = sum / count
        self.grid_sum = torch.zeros((4, self.cells_x, self.cells_y), dtype=torch.float32,
                                    device=self.device)
        self.grid_count = torch.zeros((self.cells_x, self.cells_y), dtype=torch.int,
                                      device=self.device)

        self.grid_res = 0.25

        qos_profile = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

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

        self.pcl_pub = self.create_publisher(PointCloud2, '/pcl', 10)

        self.image = torch.zeros((720, 1280, 3), dtype=torch.int, device=self.device)

        # Camera intrinsic matrix K (fx, fy, cx, cy)
        self.intrinsic = torch.tensor([[614.0482788085938, 0, 647.2186279296875],
                                      [0, 614.0172119140625, 350.9230041503906],
                                      [0, 0, 1]], device=self.device)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.extrinsics_received = False
        self.img_received = False

        self.DUMMY_FIELD_PREFIX = '__'
        self.type_mappings = [(PointField.INT8, np.dtype('int8')),
                              (PointField.UINT8, np.dtype('uint8')),
                              (PointField.INT16, np.dtype('int16')),
                              (PointField.UINT16, np.dtype('uint16')),
                              (PointField.INT32, np.dtype('int32')),
                              (PointField.UINT32, np.dtype('uint32')),
                              (PointField.FLOAT32, np.dtype('float32')),
                              (PointField.FLOAT64, np.dtype('float64'))]
        self.pftype_to_nptype = dict(self.type_mappings)

    def project_to_camera_frame(self, points_xyz, os_sensor_to_camera_frame_tf):
        # 3x4 [R|t] from lidar frame to camera optical frame
        homogeneous_matrix_cam = self.numpify(os_sensor_to_camera_frame_tf)[:3]
        os_sensor_to_camera_extrinsic = torch.tensor(homogeneous_matrix_cam, dtype=torch.float32,
                                                     device=self.device)

        # Homogeneous 4xN points, then project all at once: K @ [R|t] @ X
        to_proj_points_xyz = torch.vstack(
            (points_xyz.T, torch.ones((1, points_xyz.shape[0]), device=self.device)))
        proj_pts = self.intrinsic @ os_sensor_to_camera_extrinsic @ to_proj_points_xyz
        # Perspective divide by depth, keep (u, v) pixel coordinates
        proj_pts = (proj_pts / proj_pts[2, :].unsqueeze(0))[0:2]

        # Keep only points that land inside the image
        cam_mask = (proj_pts[0, :] > 1) & (proj_pts[0, :] < 1278) & \
            (proj_pts[1, :] > 1) & (proj_pts[1, :] < 718)
        proj_pts_in_frame = proj_pts[:, cam_mask]

        return torch.floor(proj_pts_in_frame.T).int(), cam_mask

    def publish_pointcloud_rgb(self, tensor_points_rgb, header):
        """
        Publish a PointCloud2 message from a combined tensor of XYZRGB values.

        Args:
        ----
            tensor_points_rgb (torch.Tensor): Tensor of shape (N, 4) containing XYZ
                coordinates and packed RGB values (uint32) in the last column
            header: ROS2 Header object

        """
        # Convert tensor to numpy and ensure correct shapes
        points_rgb = tensor_points_rgb.cpu().detach().numpy()

        # Create structured numpy array
        cloud_struct = np.zeros(len(points_rgb), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.uint32)
        ])

        # Fill in the data
        cloud_struct['x'] = points_rgb[:, 0]
        cloud_struct['y'] = points_rgb[:, 1]
        cloud_struct['z'] = points_rgb[:, 2]
        cloud_struct['rgb'] = points_rgb[:, 3].astype(np.uint32)  # Ensure RGB is uint32

        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]

        # Set the frame ID
        header.frame_id = 'home'

        # Create and return the PointCloud2 message
        cloud_msg = point_cloud2.create_cloud(header, fields, cloud_struct)
        self.pcl_pub.publish(cloud_msg)

    def numpify(self, tf):
        # TF to 3x4 matrix, with axes reordered to match the sensor convention
        homogeneous_matrix_map = ros2_numpy.numpify(tf.transform)[:3]
        temp2_2 = homogeneous_matrix_map[2, 2]
        homogeneous_matrix_map[2, 2] = -1 * homogeneous_matrix_map[0, 0]
        homogeneous_matrix_map[0, 0] = temp2_2

        temp2_1 = homogeneous_matrix_map[2, 1]
        homogeneous_matrix_map[2, 1] = homogeneous_matrix_map[1, 0]
        homogeneous_matrix_map[1, 0] = temp2_1

        temp0_1 = homogeneous_matrix_map[0, 1]
        homogeneous_matrix_map[0, 1] = -1 * homogeneous_matrix_map[1, 2]
        homogeneous_matrix_map[1, 2] = temp0_1

        homogeneous_matrix_map[1, 1] = -1 * homogeneous_matrix_map[1, 1]
        homogeneous_matrix_map[2, 0] = -1 * homogeneous_matrix_map[2, 0]
        return homogeneous_matrix_map

    def map_pcl(self, points_xyz, cam_mask, os_sensor_to_home_tf):
        # Transform the in-image points from lidar frame to the home (map) frame
        homogeneous_matrix_map = self.numpify(os_sensor_to_home_tf)
        os_sensor_to_home_extrinsic = torch.tensor(homogeneous_matrix_map, dtype=torch.float32,
                                                   device=self.device)
        filtered_points_xyz = points_xyz[cam_mask]
        mapped_points = (os_sensor_to_home_extrinsic @ torch.vstack((
            filtered_points_xyz.T,
            torch.ones((1, filtered_points_xyz.shape[0]), device=self.device)))).T
        return mapped_points

    def pad_grid(self, grid, d_min_x, d_max_x, d_min_y, d_max_y):
        # Grow the grid to the new bounds, keeping existing cells in place
        temp_grid = torch.zeros((*grid.shape[:-2], self.cells_x, self.cells_y), dtype=grid.dtype,
                                device=self.device)
        temp_grid[..., d_min_x: self.cells_x - d_max_x, d_min_y: self.cells_y - d_max_y] = grid
        return temp_grid

    def points_callback(self, msg):
        if not self.img_received:
            return

        try:
            os_sensor_to_camera_frame_tf = self.tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'os_sensor', rclpy.time.Time())
        except TransformException:
            return

        try:
            os_sensor_to_home_tf = self.tf_buffer.lookup_transform(
                'home', 'os_sensor', rclpy.time.Time())
        except TransformException:
            return

        raw_points_dict = ros2_numpy.numpify(msg)

        raw_points_xyz = torch.tensor(raw_points_dict['xyz'], device=self.device)

        filtered_points_cam_frame, cam_mask = self.project_to_camera_frame(
            raw_points_xyz, os_sensor_to_camera_frame_tf)

        mapped_points = self.map_pcl(raw_points_xyz, cam_mask, os_sensor_to_home_tf)
        if mapped_points[:, 2].numel() < 10:
            return

        # Look up the pixel color at each projected (v, u) location
        mapped_colors = self.image[filtered_points_cam_frame[:, 1],
                                   filtered_points_cam_frame[:, 0]]

        mapped_zhsv = torch.cat((mapped_points[:, 2:3], mapped_colors.to(torch.float32)), dim=1)

        if mapped_points.numel() == 0:
            return

        # Bin map-frame xy into grid cells
        mapped_points_idx = torch.floor(mapped_points[:, :2] / self.grid_res).int()

        max_x = torch.max(mapped_points_idx[:, 0]).item()
        max_y = torch.max(mapped_points_idx[:, 1]).item()
        min_x = torch.min(mapped_points_idx[:, 0]).item()
        min_y = torch.min(mapped_points_idx[:, 1]).item()

        # Expand grid bounds if this scan reaches outside them
        d_min_x = d_min_y = d_max_x = d_max_y = 0

        if min_x < self.min_x:
            d_min_x = self.min_x - min_x
            self.min_x = min_x

        if min_y < self.min_y:
            d_min_y = self.min_y - min_y
            self.min_y = min_y

        if max_x > self.max_x:
            d_max_x = max_x - self.max_x
            self.max_x = max_x

        if max_y > self.max_y:
            d_max_y = max_y - self.max_y
            self.max_y = max_y

        self.cells_x = self.max_x - self.min_x + 1
        self.cells_y = self.max_y - self.min_y + 1

        # Shift to zero-based indices and flatten to a linear cell index
        mapped_points_idx[:, 0] -= self.min_x
        mapped_points_idx[:, 1] -= self.min_y
        mapped_points_idx_flat = mapped_points_idx[:, 1] + mapped_points_idx[:, 0] * self.cells_y

        self.grid_sum = self.pad_grid(self.grid_sum, d_min_x, d_max_x, d_min_y, d_max_y)
        self.grid_count = self.pad_grid(self.grid_count, d_min_x, d_max_x, d_min_y, d_max_y)

        grid_count_flat = self.grid_count.view(self.cells_x * self.cells_y)
        grid_sum_flat = self.grid_sum.view(4, self.cells_x * self.cells_y)

        # Scatter-add every point into its cell, then average
        grid_count_flat.index_add_(0, mapped_points_idx_flat, torch.ones(
            mapped_points.size(0), dtype=torch.int, device=self.device))
        grid_sum_flat.index_add_(1, mapped_points_idx_flat, mapped_zhsv.T)

        grid_z_mean, grid_h_mean, grid_s_mean, grid_v_mean = \
            (grid_sum_flat / grid_count_flat).view(4, self.cells_x, self.cells_y)

        grid_r_flat, grid_g_flat, grid_b_flat = grid_h_mean.flatten(), grid_s_mean.flatten(), \
            grid_v_mean.flatten()

        # Pack mean color into a single uint32 for the PointCloud2 rgb field
        grid_rgb_flat = (grid_r_flat.to(torch.int32) << 16) | \
            (grid_g_flat.to(torch.int32) << 8) | grid_b_flat.to(torch.int32)
        grid_rgb_flat.to(torch.uint32)

        # Build cell center coordinates and publish the grid as a colored point cloud
        x_grid = torch.arange(self.min_x * self.grid_res,
                              self.min_x * self.grid_res + self.cells_x * self.grid_res,
                              self.grid_res, dtype=torch.float32, device=self.device)
        y_grid = torch.arange(self.min_y * self.grid_res,
                              self.min_y * self.grid_res + self.cells_y * self.grid_res,
                              self.grid_res, dtype=torch.float32, device=self.device)
        X, Y = torch.meshgrid(x_grid, y_grid, indexing='ij')
        Z_UC = torch.stack((X.flatten(), Y.flatten(), grid_z_mean.flatten(), grid_rgb_flat), dim=1)
        self.publish_pointcloud_rgb(Z_UC, msg.header)

    def image_callback(self, msg):
        np_img = ros2_numpy.numpify(msg)
        self.image = torch.tensor(np_img).cuda()
        if self.img_received is not True:
            self.img_received = True


def main(args=None):
    rclpy.init(args=args)
    map_gen = MapGen()
    rclpy.spin(map_gen)
    map_gen.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
