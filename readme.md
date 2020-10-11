# Flowfield-Turbulence Three-robot-formation Control System

本repo是东南大学自动化学院2020-2021学年课程《控制系统建模与分析综合设计I》的课题《流场扰动下的3个机器人闭曲线上的编队控制ROS仿真》的成果。

## 一阶模型下同心圆轨迹编队控制启动方法：
1. 在工作目录下执行 catkin_make
2. 运行 launch_first_order.sh
3. 等待 gazebo 加载完毕
4. 运行 launch_first_order_controller.sh

## 一阶模型下椭圆轨迹编队控制启动方法：
1. 在工作目录下执行 catkin_make
2. 运行 launch_first_order.sh
3. 等待 gazebo 加载完毕
4. 运行 launch_first_order_controller_advanced.sh
5. 在编队参数窗口中输入编队参数，调整参数后点击Publish Topic按钮。
6. 注意当椭圆轴长为0时，控制器停止控制。
7. 注意当速度ω=0时，小车会保持原地不动。
