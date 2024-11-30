import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
from sensor_msgs.msg import Image
import numpy as np
import os
import tvm
import matplotlib.pyplot as plt

cmap = plt.cm.viridis

class FastDepth(Node):

    def __init__(self):
        super().__init__('fast_depth')
        self.model_dir = '/workspaces/isaac_ros-dev/results/tvm_compiled/tx2_gpu_mobilenet_nnconv5dw_skipadd_pruned/'
        self.loaded_lib = tvm.module.load(os.path.join(self.model_dir, "deploy_lib.o"))
        self.dev_lib = tvm.module.load(os.path.join(self.model_dir, "deploy_cuda.ptx"))
        self.loaded_lib.import_module(self.dev_lib)
        self.loaded_graph = open(os.path.join(self.model_dir,"deploy_graph.json")).read()
        self.loaded_params = bytearray(open(os.path.join(self.model_dir, "deploy_param.params"), "rb").read())
        self.fcreate = tvm.get_global_func("tvm.graph_runtime.create")
        self.ctx = tvm.gpu(0)
        self.gmodule = self.fcreate(self.loaded_graph, self.loaded_lib, self.ctx.device_type, self.ctx.device_id)
        self.set_input, self.get_output, self.run = self.gmodule["set_input"], self.gmodule["get_output"], self.gmodule["run"]

        self.subscription = self.create_subscription(
            Image,
            '/resize/image',
            self.img_callback,
            10)
        self.subscription

        self.publisher = self.create_publisher(Image, 'depth_image', 10)

    def img_callback(self, msg):
        rgb_np = rnp.numpify(msg)
        x = np.zeros([1,3,224,224])
        x[0,:,:,:] = np.transpose(rgb_np, (2,0,1))
        self.set_input('0', tvm.nd.array(x.astype('float32')))
        self.gmodule["load_params"](self.loaded_params)
        self.run()
        out_shape = (1, 1, 224, 224)
        out = tvm.nd.empty(out_shape, "float32")
        self.get_output(0, out)
        pred_np_2d = out.asnumpy()[0,0,:,:]
        d_min = np.min(pred_np_2d)
        d_max = np.max(pred_np_2d)              
        depth_relative = (pred_np_2d - d_min) / (d_max - d_min)
        pred_np_color = 255 * cmap(depth_relative)[:,:,:3]
        new_msg = rnp.msgify(Image, pred_np_color.astype('uint8'), 'rgb8')
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)

    fast_depth = FastDepth()

    rclpy.spin(fast_depth)

    fast_depth.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()