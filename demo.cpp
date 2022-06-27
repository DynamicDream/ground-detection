#include"sidewalk.h"
//#include"FastCluster.h"

void main() {

	std::vector<cv::String> filename{"2022-01-25-09-36-30_intensity","2022-01-26-12-25-15_intensity","2022-01-26-11-25-46_intensity","2022-01-25-10-13-08_intensity"};
	GroundDetector detector;
	cv::String lidar_dir = "E:/DL/M1/"+filename[1];
	std::vector<cv::String> lidar_files;	//用于存放搜索到的点云文件路径
	cv::glob(lidar_dir, lidar_files);	//获取点云文件路径
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	detector.Init_Viewer(viewer);
	//连续播放
	while (true)
	{
		//从第1帧开始，遍历整个文件夹内的点云数据（一般情况下第0帧数据缺失较大）
		for (int id = 1; id < lidar_files.size(); id++) {
			DATA input;	//初始化数据容器
			std::cout << lidar_files[id] << std::endl;	//打印当前处理的文件路径，有助于调试时针对特定帧进行参数优化
			ParseFile(lidar_files[id], input);	//解析点云数据到数据容器
			std::vector<cv::Point3d> boundary_pts;	//初始化边界点向量
			int64_t start_time = cv::getTickCount();//计时开始
			Sector Result = detector.Computer(input, boundary_pts);//可行驶区域检测
			//检测时间花销（依赖硬件）
			std::cout << "time: " << (cv::getTickCount() - start_time) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
			detector.Show_Ground_Object(viewer, Result, boundary_pts);//检测结果显示
		}
	}

}
