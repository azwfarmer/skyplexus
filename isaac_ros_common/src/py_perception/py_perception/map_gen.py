import time
import os
import sys
import math
import numpy as np
import cv2

import torch
from torchvision import transforms
import torch.nn.functional as F

import ros2_numpy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

efficientvit_path = os.path.abspath('/workspaces/isaac_ros-dev/efficientvit')
sys.path.append(efficientvit_path)
from efficientvit.models.utils import resize
from efficientvit.seg_model_zoo import create_seg_model
from eval_seg_model import ADE20KDataset, ToTensor, get_canvas

class MapGen(Node):

    def __init__(self):
        super().__init__('map_gen')
        torch.set_printoptions(sci_mode=False)
        np.set_printoptions(suppress=True)
        torch.set_printoptions(threshold=10000)

        self.device = torch.device("cuda")

        self.min_x = 0
        self.min_y = 0
        self.max_x = 0
        self.max_y = 0

        self.cells_x = self.max_x - self.min_x + 1
        self.cells_y = self.max_y - self.min_y + 1

        # self.grid_RGB = torch.zeros((self.cells_y, self.cells_x), dtype=torch.int, device=self.device) # pad all of these in the future
        # self.grid_intensity = torch.zeros((self.cells_y, self.cells_x), dtype=torch.float32, device=self.device)
        # self.grid_z_max = torch.zeros((self.cells_y, self.cells_x), dtype=torch.float32, device=self.device)
        self.grid_z_mean = torch.zeros((self.cells_x, self.cells_y), dtype=torch.float32, device=self.device)

        self.grid_z_sum = torch.zeros(self.cells_x * self.cells_y, dtype=torch.float32, device=self.device)
        self.grid_z_count = torch.zeros(self.cells_x * self.cells_y, dtype=torch.int, device=self.device)

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


        self.image = torch.zeros((1080, 1920, 3), dtype=torch.int, device=self.device)

        self.model = create_seg_model("l2", "ade20k", weight_url=None).cuda()
        self.model.eval()

        self.intrinsic = torch.tensor([[1252.348876953125, 0, 971.9642944335938], 
                                      [0, 1252.5963134765625, 542.4281616210938], 
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

    def fields_to_dtype(self, fields, point_step):
        offset = 0
        np_dtype_list = []
        for f in fields:
            while offset < f.offset:
                np_dtype_list.append(
                    ('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
                offset += 1

            dtype = self.pftype_to_nptype[f.datatype]
            if f.count != 1:
                dtype = np.dtype((dtype, f.count))

            np_dtype_list.append((f.name, dtype))
            offset += self.pftype_to_nptype[f.datatype].itemsize * f.count

        while offset < point_step:
            np_dtype_list.append(('%s%d' % (self.DUMMY_FIELD_PREFIX, offset), np.uint8))
            offset += 1

        return np_dtype_list

    def pcl_numpify(self, cloud_msg):
        dtype_list = self.fields_to_dtype(cloud_msg.fields, cloud_msg.point_step)
        cloud_arr = np.frombuffer(cloud_msg.data, dtype_list)

        cloud_arr = cloud_arr[
            [fname for fname, _type in dtype_list if not (
                fname[:len(self.DUMMY_FIELD_PREFIX)] == self.DUMMY_FIELD_PREFIX)]]

        if cloud_msg.height == 1:
            return np.reshape(cloud_arr, (cloud_msg.width,))
        else:
            return np.reshape(cloud_arr, (cloud_msg.height, cloud_msg.width))

    def project_to_camera_frame(self, points_xyz, os_sensor_to_camera_frame_tf):
        homogeneous_matrix_cam = ros2_numpy.numpify(os_sensor_to_camera_frame_tf.transform)[:3]
        os_sensor_to_camera_extrinsic = torch.tensor(homogeneous_matrix_cam, dtype=torch.float32, requires_grad=True, device=self.device)

        to_proj_points_xyz = torch.vstack((points_xyz.T, torch.ones((1, points_xyz.shape[0]), device=self.device)))
        proj_pts = self.intrinsic @ os_sensor_to_camera_extrinsic @ to_proj_points_xyz
        proj_pts = (proj_pts / proj_pts[2, :].unsqueeze(0))[0:2]

        cam_mask = (proj_pts[0, :] > 1) & (proj_pts[0, :] < 1918) & (proj_pts[1, :] > 1) & (proj_pts[1, :] < 1078)
        proj_pts_in_frame = proj_pts[:, cam_mask]

        return torch.floor(proj_pts_in_frame.T).int(), cam_mask

    def publish_pointcloud2(self, tensor_points, header):
        points = tensor_points.cpu().detach().numpy()
        
        header.frame_id = 'home'

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud_msg = point_cloud2.create_cloud(header, fields, points)

        self.pcl_pub.publish(cloud_msg)

    def display_image(self,filtered_points, filtered_intensity):
        filtered_points = filtered_points.T
        x_coords = torch.round(filtered_points[0, :]).int()
        y_coords = torch.round(filtered_points[1, :]).int()

        offsets = torch.tensor([[-1, 0, 1]], device=self.device)
        X_offsets, Y_offsets = torch.meshgrid(offsets.squeeze(), offsets.squeeze())

        X_flat = (x_coords[:, None] + X_offsets.flatten()).flatten()
        Y_flat = (y_coords[:, None] + Y_offsets.flatten()).flatten()

        sv = torch.ones((filtered_intensity.shape[0],1), device=self.device)
        colors = self.hsv_to_rgb(filtered_intensity, sv, sv)
        colors_expanded = colors.unsqueeze(1).repeat(1, 9, 1).view(-1, 3) 

        self.image[Y_flat, X_flat] = colors_expanded
        self.image = self.image[:,:,[2,1,0]]
        cv2.imshow("overlay", self.image.numpy())
        cv2.waitKey(1)

    def semseg(self):
        data = self.image
        h, w = self.image.shape[:2]
        if h < w:
            th = 512
            tw = math.ceil(w / h * th / 32) * 32
        else:
            tw = 512
            th = math.ceil(h / w * tw / 32) * 32
        if th != h or tw != w:
            data = cv2.resize(
                self.image.cpu().numpy(),
                dsize=(tw, th),
                interpolation=cv2.INTER_CUBIC,
            )

        transform = transforms.Compose(
            [
                ToTensor(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        class_colors = ADE20KDataset.class_colors
        data = transform({"data": data, "label": np.ones_like(data)})["data"]

        with torch.inference_mode():
            data = torch.unsqueeze(data, dim=0).cuda()
            output = self.model(data)
            if output.shape[-2:] != self.image.shape[:2]:
                output = resize(output, size=self.image.shape[:2])
            output = torch.argmax(output, dim=1)[0]
            return output
            # canvas = get_canvas(self.image, output, class_colors)
            # return canvas
            # cv2.imshow("semseg", canvas)
            # cv2.waitKey(1)

    def map_pcl(self, points_xyz, cam_mask, os_sensor_to_home_tf):     
        homogeneous_matrix_map = ros2_numpy.numpify(os_sensor_to_home_tf.transform)[:3]
        homogeneous_matrix_map[0,0] = -1 * homogeneous_matrix_map[0,0]
        homogeneous_matrix_map[0,1], homogeneous_matrix_map[1,2] = -1 * homogeneous_matrix_map[1,2], homogeneous_matrix_map[0,1]
        homogeneous_matrix_map[1,0], homogeneous_matrix_map[2,1] = homogeneous_matrix_map[2,1], homogeneous_matrix_map[1,0]
        homogeneous_matrix_map[1,1] = -1 * homogeneous_matrix_map[1,1]
        homogeneous_matrix_map[2,0] = -1 * homogeneous_matrix_map[2,0]

        os_sensor_to_home_extrinsic = torch.tensor(homogeneous_matrix_map, dtype=torch.float32, requires_grad=True, device=self.device)

        filtered_points_xyz = points_xyz[cam_mask]

        mapped_points = (os_sensor_to_home_extrinsic @ torch.vstack((filtered_points_xyz.T, torch.ones((1, filtered_points_xyz.shape[0]), device=self.device)))).T

        return mapped_points

    def pad_grid(self, grid, d_min_x, d_max_x, d_min_y, d_max_y):
        temp_grid = torch.zeros((self.cells_x, self.cells_y), dtype=grid.dtype, device=self.device)
        temp_grid[d_min_x : self.cells_x - d_max_x, d_min_y : self.cells_y - d_max_y] = grid
        return temp_grid

    def points_callback(self, msg):
        if not self.img_received:
            return
        
        try:
            os_sensor_to_camera_frame_tf = self.tf_buffer.lookup_transform('camera_color_optical_frame','os_sensor',rclpy.time.Time())
        except TransformException:
            return
        
        try:
            os_sensor_to_home_tf = self.tf_buffer.lookup_transform('home','os_sensor',rclpy.time.Time())
        except TransformException:
            return

        raw_points_dict = ros2_numpy.numpify(msg)
        raw_points_xyz = torch.tensor(raw_points_dict['xyz'], device=self.device, requires_grad=True)
        valid_mask = (raw_points_xyz >= 1).all(dim=1)
        points_xyz = raw_points_xyz[valid_mask,:]
        
        filtered_points_cam_frame, cam_mask = self.project_to_camera_frame(points_xyz, os_sensor_to_camera_frame_tf)

        mapped_points = self.map_pcl(points_xyz, cam_mask, os_sensor_to_home_tf)

        if mapped_points.numel() == 0:
            return

        mapped_points_idx = torch.floor(mapped_points[:,:2] / self.grid_res).int() 

        max_x = torch.max(mapped_points_idx[:,0]).item()
        max_y = torch.max(mapped_points_idx[:,1]).item()

        min_x = torch.min(mapped_points_idx[:,0]).item()
        min_y = torch.min(mapped_points_idx[:,1]).item()

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
            d_max_y = max_y = self.max_y
            self.max_y = max_y
        
        self.cells_x = self.max_x - self.min_x + 1
        self.cells_y = self.max_y - self.min_y + 1

        mapped_points_idx[:,0] -= self.min_x
        mapped_points_idx[:,1] -= self.min_y
        mapped_points_idx_flat = mapped_points_idx[:,1] + mapped_points_idx[:,0] * self.cells_y

        self.grid_z_mean = self.pad_grid(self.grid_z_mean, d_min_x, d_max_x, d_min_y,d_max_y)
        self.grid_z_count = self.pad_grid(self.grid_z_count, d_min_x, d_max_x, d_min_y, d_max_y)
        grid_z_count_flat = self.grid_z_count.view(self.cells_x * self.cells_y)
        grid_z_count_flat.index_add_(0, mapped_points_idx_flat, torch.ones(mapped_points.size(0), dtype=torch.int, device=self.device))
        self.get_logger().info(f'{torch.max(mapped_points_idx_flat)} > {grid_z_count_flat.size(0)} {torch.max(mapped_points_idx_flat) > grid_z_count_flat.size(0)}')
        
        self.grid_z_sum = self.pad_grid(self.grid_z_sum, d_min_x, d_max_x, d_min_y, d_max_y)
        grid_z_sum_flat = self.grid_z_sum.view(self.cells_x * self.cells_y)
        grid_z_sum_flat.index_add_(0, mapped_points_idx_flat, mapped_points[:,2].squeeze())
        
        self.grid_z_mean = (grid_z_sum_flat / grid_z_count_flat).view(self.cells_x, self.cells_y)

        x_grid = torch.arange(self.min_x * self.grid_res, self.min_x * self.grid_res + self.cells_x * self.grid_res, self.grid_res, dtype=torch.float32, device=self.device)
        y_grid = torch.arange(self.min_y * self.grid_res, self.min_y * self.grid_res + self.cells_y * self.grid_res, self.grid_res, dtype=torch.float32, device=self.device)
        X, Y = torch.meshgrid(x_grid, y_grid, indexing='ij')
        Z_U = torch.stack((X.flatten(), Y.flatten(), self.grid_z_mean.flatten()), dim=1)

        self.publish_pointcloud2(Z_U, msg.header)

        # raw_intensity = torch.tensor(raw_points_dict['intensity'] / 65535.0, dtype=torch.float32, device=self.device, requires_grad=True)
        # intensity = raw_intensity[valid_mask]

        # class_mask = self.semseg()

        # self.publish_pointcloud2(mapped_points, msg.header)

        # mapped_colors = self.image[filtered_points_cam_frame[:,1], filtered_points_cam_frame[:,0]]
        # mapped_semantics = class_mask[filtered_points_cam_frame[:,1], filtered_points_cam_frame[:,0]]
        # mapped_intensity = intensity[cam_mask]


    def image_callback(self, msg):
        np_img = ros2_numpy.numpify(msg)
        self.image = torch.tensor(np_img).cuda()
        if self.img_received != True:
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