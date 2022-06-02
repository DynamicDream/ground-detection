# 基于3d点云的目标检测算法
'''
算法思路：同现有算法相似，首先将点云进行扇形栅格化，再进一步采用距离、坡度和高度连续性进行地面检测。
鉴于步骤简单，单帧处理时间仅20ms左右
'''
# 依赖库安装：PCL和Opencv
# 参考文献：
'''
  1.Fast Ground Segmentation for 3D LiDAR Point Cloud Based on  Jump-Convolution-Process
  2.Patchwork
'''
