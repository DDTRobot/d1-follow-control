import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3, TwistStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.srv import GetParameters
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import serial
from fastcrc import crc16
from struct import unpack_from, pack, unpack
import math

# 串口配置
serial_port = '/dev/ttyUSB0'

class DistC5:
    def __init__(self, uwb_processor):
        self.FORMAT = "<1B1B1B1H1B1B1I1I1I1H1f1f1f1B"
        self.uwb_processor = uwb_processor

    def parse(self, data):
        (h1, h2, s, len, cmd, len2, sync_cnt, index, fob_id, fob_type,
         distance, angle, pitch, rssi) = unpack_from(self.FORMAT, data)

        # 修正角度坐标系
        adjusted_angle = angle
        if adjusted_angle > 180:
            adjusted_angle -= 360
        elif adjusted_angle < -180:
            adjusted_angle += 360

        # 转换为弧度
        angle_rad = math.radians(adjusted_angle)

        self.uwb_processor.update_uwb_data(distance, angle_rad, adjusted_angle)

class Parser:
    def __init__(self, uwb_processor):
        self.arr = bytearray(b'')
        self.data_length = 0
        self.crc_ok = False
        self.uwb_processor = uwb_processor
        self.dist_parser = DistC5(uwb_processor)

    def append(self, byte):
        length = len(self.arr)
        if length == 0:
            if byte == 0x55:
                self.arr.append(byte)
            else:
                return False
        elif length == 1:
            if byte == 0xaa:
                self.arr.append(byte)
            else:
                self.arr = bytearray(b'')
        elif length <= 4:
            self.arr.append(byte)
            if length == 4:
                self.data_length = self.arr[3] + (self.arr[4] << 8)
        elif length <= self.data_length + 6:
            self.arr.append(byte)
            if length == self.data_length + 6:
                crc1 = self.arr[length - 1]
                crc2 = self.arr[length]
                data = self.arr[5: self.data_length + 5]
                calc_crc = self.calculate_crc(bytes(data))
                if calc_crc & 0xff == crc2 and (calc_crc >> 8) & 0xff == crc1:
                    self.crc_ok = True
                return True
        return False

    def reset(self):
        self.arr = bytearray(b'')
        self.crc_ok = False
        self.data_length = 0

    def parse(self):
        if not self.crc_ok:
            self.reset()
            return

        bytes_arr = bytes(self.arr)
        cmd = self.arr[5]

        if cmd == 0xc5:
            self.dist_parser.parse(bytes_arr)

        self.reset()

    def calculate_crc(self, data):
        return crc16.xmodem(data)

class UWBProcessor:
    def __init__(self, main_node):
        self.node = main_node
        self.current_distance = 4.0
        self.current_angle_rad = 0.0
        self.current_angle_deg = 0.0
        self.uwb_data_updated = False
        self.data_lock = threading.Lock()
        self.last_update_time = time.time()
        self.tx_enabled = False

        # 纯比例控制参数
        self.target_distance = 1.2
        self.kp_distance = 1.0
        self.kp_angle = 1.2

        # 平滑控制参数
        self.max_linear_speed = 1.5
        self.max_angular_speed = 1.5

        # 死区阈值
        self.distance_deadzone = 0.2
        self.angle_deadzone = math.radians(10.0)

        # 非线性增益参数
        self.distance_threshold = 0.8
        self.angle_threshold = math.radians(25)

        # 初始化串口线程
        self.serial_thread = None
        self.stop_event = threading.Event()
        self.start_serial_thread()

    def update_uwb_data(self, distance, angle_rad, angle_deg):
        """更新UWB数据"""
        with self.data_lock:
            self.current_distance = distance
            self.current_angle_rad = angle_rad
            self.current_angle_deg = angle_deg
            self.uwb_data_updated = True
            self.last_update_time = time.time()

    def get_uwb_data(self):
        """获取UWB数据"""
        with self.data_lock:
            distance = self.current_distance
            angle_rad = self.current_angle_rad
            angle_deg = self.current_angle_deg
            updated = self.uwb_data_updated

            # 检查数据是否过期
            data_age = time.time() - self.last_update_time
            if data_age > 0.5:
                updated = False

            return distance, angle_rad, angle_deg, updated

    def nonlinear_gain(self, error, base_gain, threshold):
        """非线性增益函数"""
        abs_error = abs(error)
        if abs_error < threshold:
            return base_gain * (abs_error / threshold) ** 2
        else:
            return base_gain

    def calculate_follow_command(self, current_distance, current_angle_rad):
        """简化的纯比例控制算法"""
        filtered_distance = current_distance
        filtered_angle_deg = math.degrees(current_angle_rad)

        # 计算角度误差绝对值
        angle_error_abs = abs(filtered_angle_deg)

        # 特殊处理：角度误差过大时只旋转
        if angle_error_abs > 90.0:
            return self.angle_only_control(current_angle_rad)

        # 计算误差
        error_distance = filtered_distance - self.target_distance
        error_angle_rad = current_angle_rad

        # 死区处理
        if abs(error_distance) < self.distance_deadzone:
            error_distance = 0
        if abs(filtered_angle_deg) < math.degrees(self.angle_deadzone):
            filtered_angle_deg = 0
            error_angle_rad = 0

        # 非线性增益
        kp_dist = self.nonlinear_gain(error_distance, self.kp_distance, self.distance_threshold)
        kp_angle = self.nonlinear_gain(error_angle_rad, self.kp_angle, self.angle_threshold)

        # 纯比例控制
        linear_x = kp_dist * error_distance
        angular_z = -kp_angle * error_angle_rad

        # 限制速度
        linear_x = max(min(linear_x, self.max_linear_speed), -self.max_linear_speed)
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        return linear_x, angular_z

    def angle_only_control(self, angle_rad):
        """纯角度控制模式（用于大角度调整）"""
        error_angle_rad = angle_rad

        # 非线性增益
        kp_angle = self.nonlinear_gain(error_angle_rad, self.kp_angle, self.angle_threshold)

        # 纯比例角度控制
        angular_z = -kp_angle * error_angle_rad
        angular_z = max(min(angular_z, self.max_angular_speed), -self.max_angular_speed)

        # 距离控制（停止）
        linear_x = 0.0

        return linear_x, angular_z

    def serial_reader(self, serial_port, stop_event):
        """串口读取线程函数"""
        try:
            ser = serial.Serial(serial_port, 115200, timeout=0.1)
            parser = Parser(self)

            TX_MSG = bytes([0x55, 0xAA, 0x00, 0x04, 0x00, 0x07, 0x02, 0x01, 0x15, 0x4E, 0xE8])

            #信令配对
            # INIT_MSG_1 = bytes([0x55, 0xAA, 0x00, 0x03, 0x00, 0x15, 0x01, 0x0A, 0x3A, 0xE8])
            # INIT_MSG_2 = bytes([0x55, 0xAA, 0x00, 0x03, 0x00, 0x05, 0x01, 0x01, 0xC8, 0xE0])
            # ser.write(INIT_MSG_1)
            # time.sleep(0.05)
            # ser.write(INIT_MSG_2)

            self.node.get_logger().info("串口读取线程启动")

            while not stop_event.is_set():
                if ser.in_waiting > 0:
                    bytes_data = ser.read(ser.in_waiting)
                    # self.node.get_logger().info(f"串口原始数据: {bytes_data.hex(' ')}")
                    if self.tx_enabled:
                        ser.write(TX_MSG)
                    for b in bytes_data:
                        if parser.append(b):
                            parser.parse()
                else:
                    time.sleep(0.001)

        except Exception as e:
            self.node.get_logger().error(f"串口线程错误: {e}")
        finally:
            if 'ser' in locals():
                ser.close()
            self.node.get_logger().info("串口读取线程结束")

    def start_serial_thread(self):
        """启动串口读取线程"""
        self.serial_thread = threading.Thread(
            target=self.serial_reader,
            args=(serial_port, self.stop_event)
        )
        self.serial_thread.daemon = True
        self.serial_thread.start()
        self.node.get_logger().info("UWB串口线程已启动")

    def stop(self):
        """停止串口线程"""
        self.stop_event.set()
        if self.serial_thread:
            self.serial_thread.join(timeout=1.0)

class D435iObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('d435i_obstacle_avoidance')
        self.robot_ns = os.environ.get('ROBOT_NS', 'd1')
        self.get_logger().info(f"ROBOT_NS = {self.robot_ns}")

        # 创建QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # 创建发布者
        self.publisher_ = self.create_publisher(
            TwistStamped,
            f'/{self.robot_ns}/command/cmd_twist',
            qos_profile
        )

        # 订阅深度图像话题
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10
        )

        # 订阅相机内参话题（用于地面过滤）
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10
        )

        # 相机内参和高度参数
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_height = 0.5  # 相机离地高度（米），可根据实际情况调整
        self.ground_threshold = 0.05  # 地面高度阈值（米）

        # 初始化use_sdk参数
        self.use_sdk = False
        
        # 初始化参数客户端
        self.init_parameter_client()

        # 初始化CV Bridge
        self.bridge = CvBridge()

        # 避障参数
        self.safe_distance = 0.8

        # 检测区域
        self.center_region = 0.4
        self.left_region = 0.3
        self.right_region = 0.3

        # 状态变量
        self.current_mode = "UWB_FOLLOWING"

        # 初始化UWB处理器
        self.uwb_processor = UWBProcessor(self)

        # 创建控制定时器
        self.control_timer = self.create_timer(0.05, self.control_timer_callback)  # 20Hz

        # 存储最新的深度数据
        self.latest_center_min = float('inf')
        self.latest_left_min = float('inf')
        self.latest_right_min = float('inf')
        self.depth_data_lock = threading.Lock()

        # 控制命令
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.current_action = "待机"
        self._last_avoid_was_turn = False  # 上一帧避障是否为纯旋转

        # 速度平滑（EMA），alpha 越大跟手越快、越小越平滑
        self.smooth_linear_x = 0.0
        self.smooth_angular_z = 0.0
        self.smooth_alpha = 0.6

        # 转向后前进状态控制
        self.turning_forward_active = False      # 是否处于转向后前进阶段
        self.turning_forward_start_time = 0.0    # 开始前进的时间戳（秒）
        self.turning_forward_duration = 1.2      # 前进持续时间（秒）
        self.turning_forward_speed = 1.0        # 前进速度（m/s）
        self._following_side_avoid_active = False  # 跟随模式中侧方避障是否激活

        # 日志控制
        self.log_counter = 0

        self.get_logger().info('D435i避障与UWB跟随节点已启动')
        self.get_logger().info(f'安全距离: {self.safe_distance}m')
        self.get_logger().info(f'目标跟随距离: {self.uwb_processor.target_distance}m')
        self.get_logger().info(f'比例参数: kp_distance={self.uwb_processor.kp_distance}, kp_angle={self.uwb_processor.kp_angle}')

    def init_parameter_client(self):
        """初始化参数客户端，用于获取use_sdk参数"""
        # 初始化参数客户端
        self.param_client = None
        
        # 尝试从参数服务器获取use_sdk参数的节点名称
        # 这里假设参数来自'teleop_command'节点，你可以根据实际情况修改
        param_service_name = f'/{self.robot_ns}/teleop_command/get_parameters'
        
        try:
            # 创建参数客户端
            self.param_client = self.create_client(
                GetParameters,
                param_service_name
            )
            
            # 创建定时器定期获取参数
            self.param_timer = self.create_timer(0.4, self.param_timer_callback)
            
            self.get_logger().info(f'参数客户端初始化成功，将定期获取use_sdk参数，目标服务: {param_service_name}')
            
        except Exception as e:
            self.get_logger().warning(f'初始化参数客户端失败: {e}')
            self.get_logger().info('将使用默认参数值: use_sdk=False')

    def param_timer_callback(self):
        if self.param_client is None or not self.param_client.service_is_ready():
            return

        try:
            request = GetParameters.Request()
            request.names = ['use_sdk']
            future = self.param_client.call_async(request)
            future.add_done_callback(self.param_response_callback)

        except Exception as e:
            self.get_logger().error(f'请求参数时发生错误: {e}')

    def param_response_callback(self, future):
        """参数服务响应回调函数"""
        try:
            response = future.result()
            if response is not None and response.values:
                # 获取参数值
                new_use_sdk = response.values[0].bool_value

                # 检查参数值是否发生变化
                if new_use_sdk != self.use_sdk:
                    self.use_sdk = new_use_sdk
                    self.get_logger().info(f'use_sdk参数已更新: {self.use_sdk}')

                    # 根据use_sdk参数值调整行为
                    self.on_use_sdk_changed()
            else:
                self.get_logger().warn('获取的参数响应为空')

        except Exception as e:
            self.get_logger().error(f'处理参数响应时发生错误: {e}')

    def on_use_sdk_changed(self):
        if self.use_sdk:
            self.get_logger().info('=== SDK 已开启，开始发布控制指令 ===')
        else:
            self.get_logger().info('=== SDK 已关闭，停止发布控制指令 ===')
            self._reset_state()

    def _reset_state(self):
        self.turning_forward_active = False
        self._last_avoid_was_turn = False
        self._following_side_avoid_active = False
        self.smooth_linear_x = 0.0
        self.smooth_angular_z = 0.0
        self.current_mode = "UWB_FOLLOWING"
        self.get_logger().info('状态已重置')

    def camera_info_callback(self, msg):
        """相机内参回调，用于获取深度相机内参"""
        # 提取内参矩阵中的参数
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        # self.get_logger().info(f"相机内参已更新: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def compute_min_distance_with_ground_filter(self, depth_region, rows, cols, fx, fy, cx, cy, camera_height, ground_threshold):
        """
        计算深度区域的最小距离，同时过滤掉地面点。
        depth_region: 深度值数组（单位：米）
        rows: 对应每个像素的行坐标（0-based）
        cols: 对应每个像素的列坐标（0-based）
        fx, fy, cx, cy: 相机内参
        camera_height: 相机离地高度（米）
        ground_threshold: 地面高度阈值（米），点的高度低于该值视为地面
        返回最小距离（米），若无有效点则返回 inf
        """
        # 有效深度点掩码
        valid_mask = depth_region > 0
        if not np.any(valid_mask):
            return float('inf')
        
        # 提取有效点的深度、行、列
        depths = depth_region[valid_mask]
        rows_valid = rows[valid_mask]
        cols_valid = cols[valid_mask]  # 实际上未使用，但为保持完整性保留
        
        # 计算相机坐标系下的 y 坐标（向下为正）
        # y = (v - cy) * d / fy
        y = (rows_valid - cy) * depths / fy
        
        # 计算点在地面坐标系中的高度（假设相机高度为 camera_height，y 向下）
        height = camera_height - y
        
        # 地面点掩码：高度低于阈值视为地面，过滤掉
        ground_mask = height < ground_threshold
        not_ground_mask = ~ground_mask
        
        # 如果没有非地面点，返回 inf
        if not np.any(not_ground_mask):
            return float('inf')
        
        # 非地面点的深度值
        obstacle_depths = depths[not_ground_mask]
        
        # 简化处理：如果近距离（<0.5m）的非地面点超过5个，返回最小距离，否则返回5%分位数
        close_mask = obstacle_depths < 0.5
        close_pixels = obstacle_depths[close_mask]
        
        return np.percentile(obstacle_depths, 5)

    def process_depth_image(self, depth_image, fx, fy, cx, cy, camera_height, ground_threshold):
        """
        处理深度图像，返回三个区域的最小距离（已过滤地面）。
        """
        try:
            height, width = depth_image.shape
            
            # 转换为米
            depth_image_meters = depth_image.astype(np.float32) / 1000.0
            
            # 定义区域边界
            center_start = int(width * (0.5 - self.center_region/2))
            center_end = int(width * (0.5 + self.center_region/2))
            left_start = 0
            left_end = int(width * self.left_region)
            right_start = int(width * (1 - self.right_region))
            right_end = width
            
            # 生成行索引矩阵（全图）
            # 注意：深度图像行索引为0..height-1
            row_indices = np.arange(height).reshape(-1, 1)  # 列向量
            col_indices = np.arange(width).reshape(1, -1)   # 行向量
            # 扩展为与深度图相同形状的矩阵
            rows_full = np.tile(row_indices, (1, width))
            cols_full = np.tile(col_indices, (height, 1))
            
            # 提取各区域
            center_region = depth_image_meters[:, center_start:center_end]
            left_region = depth_image_meters[:, left_start:left_end]
            right_region = depth_image_meters[:, right_start:right_end]
            
            # 提取对应的行、列索引子区域
            rows_center = rows_full[:, center_start:center_end]
            cols_center = cols_full[:, center_start:center_end]
            rows_left = rows_full[:, left_start:left_end]
            cols_left = cols_full[:, left_start:left_end]
            rows_right = rows_full[:, right_start:right_end]
            cols_right = cols_full[:, right_start:right_end]
            
            # 计算每个区域的最小距离（过滤地面）
            center_min = self.compute_min_distance_with_ground_filter(
                center_region, rows_center, cols_center, fx, fy, cx, cy, camera_height, ground_threshold)
            left_min = self.compute_min_distance_with_ground_filter(
                left_region, rows_left, cols_left, fx, fy, cx, cy, camera_height, ground_threshold)
            right_min = self.compute_min_distance_with_ground_filter(
                right_region, rows_right, cols_right, fx, fy, cx, cy, camera_height, ground_threshold)
            
            return center_min, left_min, right_min
            
        except Exception as e:
            self.get_logger().error(f'处理深度图像时出错: {str(e)}')
            return float('inf'), float('inf'), float('inf')

    def depth_callback(self, msg):
        """深度图像回调函数"""
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 如果相机内参尚未获取，则直接使用原始深度（不过滤地面）
            if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
                # 临时处理：使用简单方法
                center_min, left_min, right_min = self.process_depth_image_simple(depth_image)
                self.get_logger().warn("相机内参未就绪，使用简单深度处理（不过滤地面）")
            else:
                center_min, left_min, right_min = self.process_depth_image(
                    depth_image, self.fx, self.fy, self.cx, self.cy,
                    self.camera_height, self.ground_threshold)
            
            with self.depth_data_lock:
                self.latest_center_min = center_min
                self.latest_left_min = left_min
                self.latest_right_min = right_min
                
        except Exception as e:
            self.get_logger().error(f'深度图像回调处理出错: {str(e)}')
    
    def process_depth_image_simple(self, depth_image):
        """当相机内参未就绪时使用的简化处理（原逻辑）"""
        try:
            height, width = depth_image.shape
            depth_image_meters = depth_image.astype(np.float32) / 1000.0
            
            center_start = int(width * (0.5 - self.center_region/2))
            center_end = int(width * (0.5 + self.center_region/2))
            left_start = 0
            left_end = int(width * self.left_region)
            right_start = int(width * (1 - self.right_region))
            right_end = width
            
            center_region = depth_image_meters[:, center_start:center_end]
            left_region = depth_image_meters[:, left_start:left_end]
            right_region = depth_image_meters[:, right_start:right_end]
            
            center_min = self.get_simple_min_distance(center_region)
            left_min = self.get_simple_min_distance(left_region)
            right_min = self.get_simple_min_distance(right_region)
            
            return center_min, left_min, right_min
        except Exception as e:
            self.get_logger().error(f'简化深度处理出错: {str(e)}')
            return float('inf'), float('inf'), float('inf')
    
    def get_simple_min_distance(self, region):
        """简化距离计算"""
        valid_distances = region[region > 0]
        if len(valid_distances) == 0:
            return float('inf')
        # 快速检测近距离障碍物
        close_mask = valid_distances < 0.5
        close_pixels = valid_distances[close_mask]
        if len(close_pixels) > 5:
            return np.min(close_pixels)
        else:
            return np.percentile(valid_distances, 5)

    def is_safe_to_follow(self, center_min, left_min, right_min):
        """检查是否安全可以执行跟随"""
        return (center_min > self.safe_distance)

    def calculate_avoidance_speed(self, center_distance, left_distance, right_distance, prev_linear_x=0.0):
        """根据障碍物距离计算避障速度和角速度"""
        linear_x = 0.0
        angular_z = 0.0
        action = "待机"
        side_avoid_signal = False

        # 转速根据避障前线速度和障碍物距离动态计算：速度越快、障碍越近则转速越大
        speed_factor = max(abs(prev_linear_x), 0.5)
        avoid_turn_speed = min(8.0, max(1.0, speed_factor * self.safe_distance / max(center_distance, 0.1)))
        side_turn_cap = 4
        micro_turn = 0.2

        if center_distance < self.safe_distance:
            if left_distance > right_distance and left_distance > self.safe_distance:
                linear_x = 0.0
                angular_z = avoid_turn_speed
                action = "避障左转"
            elif right_distance > left_distance and right_distance > self.safe_distance:
                linear_x = 0.0
                angular_z = -avoid_turn_speed
                action = "避障右转"
            else:
                linear_x = -0.6
                angular_z = 0.0
                action = "左右无空间，避障后退"
        else:
            action = "安全"
            if left_distance < self.safe_distance and right_distance < self.safe_distance:
                if left_distance < right_distance:
                    angular_z = -micro_turn
                    action = "安全(右细调)"
                else:
                    angular_z = micro_turn
                    action = "安全(左细调)"
                side_avoid_signal = True
            elif left_distance < self.safe_distance:
                urgency = (self.safe_distance - left_distance) / self.safe_distance
                angular_z = -side_turn_cap * urgency
                action = "安全(右微调)"
                side_avoid_signal = True
            elif right_distance < self.safe_distance:
                urgency = (self.safe_distance - right_distance) / self.safe_distance
                angular_z = side_turn_cap * urgency
                action = "安全(左微调)"
                side_avoid_signal = True

        return linear_x, angular_z, action, side_avoid_signal

    def control_timer_callback(self):
        # 获取深度数据
        with self.depth_data_lock:
            center_min = self.latest_center_min
            left_min = self.latest_left_min
            right_min = self.latest_right_min

        # 获取UWB数据
        uwb_distance, uwb_angle_rad, uwb_angle_deg, uwb_updated = self.uwb_processor.get_uwb_data()

        # 计算角度误差的绝对值
        angle_error_abs = abs(uwb_angle_deg)

        # 创建TwistStamped消息
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header = Header()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = 'base_link'

        # --------------------------------------------------------------
        # 转向后前进状态处理（优先级最高）
        if self.turning_forward_active:
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed = now - self.turning_forward_start_time

            if elapsed < self.turning_forward_duration:
                if center_min < self.safe_distance:
                    self.turning_forward_active = False
                    self.get_logger().info("转向后前进被障碍物打断，重新进入避障模式")
                else:
                    linear_x = self.turning_forward_speed
                    angular_z = 0.0
                    self.current_action = f"转向后前进 ({elapsed:.1f}s)"
                    self.current_mode = "TURNING_FORWARD"

                    self.smooth_linear_x = self.smooth_alpha * linear_x + (1 - self.smooth_alpha) * self.smooth_linear_x
                    self.smooth_angular_z = self.smooth_alpha * angular_z + (1 - self.smooth_alpha) * self.smooth_angular_z

                    twist_stamped_msg.twist.linear = Vector3(x=float(self.smooth_linear_x), y=0.0, z=0.0)
                    twist_stamped_msg.twist.angular = Vector3(x=0.0, y=0.0, z=float(self.smooth_angular_z))
                    if self.use_sdk:
                        self.publisher_.publish(twist_stamped_msg)

                    self.log_counter += 1
                    if self.log_counter % 5 == 0:
                        self.get_logger().info(
                            f'模式: {self.current_mode}, 动作: {self.current_action}, '
                            f'距离: 前{center_min:.2f}m 左{left_min:.2f}m 右{right_min:.2f}m, '
                            f'速度: 线{self.smooth_linear_x:.3f}m/s 角{self.smooth_angular_z:.3f}rad/s'
                        )
                    return
            else:
                self.turning_forward_active = False
                self.get_logger().info("转向后前进阶段结束，恢复UWB跟随模式")

        # --------------------------------------------------------------
        # 决策逻辑
        is_safe = self.is_safe_to_follow(center_min, left_min, right_min)
        is_large_angle_adjustment = angle_error_abs > 90.0 and uwb_updated

        if is_large_angle_adjustment:
            self.current_mode = "LARGE_ANGLE_ADJUSTMENT"
            linear_x, angular_z = self.uwb_processor.calculate_follow_command(uwb_distance, uwb_angle_rad)
            self.current_action = f"大角度调整: 角度{uwb_angle_deg:.1f}°"

        elif is_safe and uwb_updated:
            # 从避障旋转切换为安全，触发短暂前进
            if self.current_mode == "OBSTACLE_AVOIDANCE" and self._last_avoid_was_turn:
                self.turning_forward_active = True
                self.turning_forward_start_time = self.get_clock().now().nanoseconds / 1e9
                self._last_avoid_was_turn = False
                self._following_side_avoid_active = False
                self.get_logger().info(f"旋转完成前方已安全，触发前进 {self.turning_forward_duration}s")
                return
            self.current_mode = "UWB_FOLLOWING"
            linear_x, angular_z = self.uwb_processor.calculate_follow_command(uwb_distance, uwb_angle_rad)

            avoid_linear_x, avoid_angular_z, action, side_avoid_signal = self.calculate_avoidance_speed(
                center_min, left_min, right_min, linear_x
            )

            if side_avoid_signal and uwb_distance > self.uwb_processor.target_distance :
                angular_z = avoid_angular_z
                angular_z = max(min(angular_z, self.uwb_processor.max_angular_speed), -self.uwb_processor.max_angular_speed)
                self._following_side_avoid_active = True
                self.current_action = f"跟随(侧方避障): 距离{uwb_distance:.2f}m, 角度{uwb_angle_deg:.1f}°, {action}"
            else:
                if self._following_side_avoid_active and uwb_distance > self.uwb_processor.target_distance:
                    self._following_side_avoid_active = False
                    self.turning_forward_active = True
                    self.turning_forward_start_time = self.get_clock().now().nanoseconds / 1e9
                    self.get_logger().info(f"侧方避障解除，触发前进 {self.turning_forward_duration}s")
                    return
                self._following_side_avoid_active = False
                self.current_action = f"跟随: 距离{uwb_distance:.2f}m, 角度{uwb_angle_deg:.1f}°"
        else:
            if uwb_distance > self.uwb_processor.target_distance :
                self.current_mode = "OBSTACLE_AVOIDANCE"
                linear_x, angular_z, action, _ = self.calculate_avoidance_speed(center_min, left_min, right_min, self.linear_x)
                self.current_action = action
                self._last_avoid_was_turn = action in ("避障左转", "避障右转", "安全(左微调)", "安全(右微调)")

                if not uwb_updated:
                    self.current_action = f"避障(无UWB)"
            else:
                self.current_mode = "UWB_FOLLOWING"
                linear_x, angular_z = self.uwb_processor.calculate_follow_command(uwb_distance, uwb_angle_rad)
                self.current_action = f"跟随: 距离{uwb_distance:.2f}m, 角度{uwb_angle_deg:.1f}°"

        self.linear_x = linear_x
        self.angular_z = angular_z

        # EMA 平滑，消除模式切换时的速度突变
        self.smooth_linear_x = self.smooth_alpha * linear_x + (1 - self.smooth_alpha) * self.smooth_linear_x
        self.smooth_angular_z = self.smooth_alpha * angular_z + (1 - self.smooth_alpha) * self.smooth_angular_z

        twist_stamped_msg.twist.linear = Vector3(x=float(self.smooth_linear_x), y=0.0, z=0.0)
        twist_stamped_msg.twist.angular = Vector3(x=0.0, y=0.0, z=float(self.smooth_angular_z))

        if not self.use_sdk:
            if self.log_counter % 1000 == 0:
                self.get_logger().info('模拟模式: 控制命令计算中但不发布，use_sdk=False')
        else:
            self.publisher_.publish(twist_stamped_msg)

        self.log_counter += 1
        if self.log_counter % 5 == 0:
            if is_large_angle_adjustment:
                self.get_logger().info(
                    f'模式: {self.current_mode}, 动作: {self.current_action}, '
                    f'速度: 线{self.smooth_linear_x:.3f}m/s 角{self.smooth_angular_z:.3f}rad/s'
                )
            else:
                self.get_logger().info(
                    f'模式: {self.current_mode}, 动作: {self.current_action}, '
                    f'距离: 前{center_min:.2f}m 左{left_min:.2f}m 右{right_min:.2f}m, '
                    f'UWB: 距离{uwb_distance:.2f}m 角度{uwb_angle_deg:.1f}°, '
                    f'速度: 线{self.smooth_linear_x:.3f}m/s 角{self.smooth_angular_z:.3f}rad/s'
                )

    def destroy_node(self):
        """销毁节点前的清理工作"""
        self.get_logger().info("正在停止节点...")
        self.uwb_processor.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = D435iObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到中断信号，正在停止...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()