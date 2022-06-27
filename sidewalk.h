#pragma once
#ifndef SIDEWALK_H_
#define SIDEWALK_H_


#include<opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>//标准C++库中的输入输出类相关头文件。
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //PCL中支持的点类型头文件。
#include<Eigen/Dense>

#define arc_to_ang 180/CV_PI
#define ang_to_arg CV_PI/180


typedef struct {
	
	std::vector<cv::Point3d>pts;	//点云向量
	std::vector<bool>is_ground;		//地面标志位向量
	std::vector<double>yaw;			//水平扫描角向量
	std::vector<double>pitch;		//垂直扫描角向量
	std::vector<double>radius;		//径向半径向量
	std::vector<uint8_t>intensity;	//反射强度向量
	std::vector<int>input_idx;		//原始索引向量
}Sector;	//存放扇形栅格数据

typedef struct {
	std::vector<cv::Point3d>pts;
	std::vector<double>yaws;
	std::vector<double>pitches;
	std::vector<double> radius;
	std::vector<int>laser_id;
	std::vector<uint8_t>intensity;
	double yaw_min = 360, yaw_max = -360;	//存放该帧点云的水平和俯仰角范围
	double pitch_min = 360, pitch_max = -360;
}DATA;

//数据文件解析函数
void ParseFile(cv::String filename, DATA &input);


class GroundDetector{
public:
	double install_height = -1.72;
	double horizontal_resolution = 1;	//方位角间隔
	double radius_resolution = 0.2;	//半径/环间隔
	double distance_max = 120, distance_min = 1;	//感兴趣区域设定
	int grid_cols = 0 ;//栅格列数
	int grid_rows = 0;//栅格行数
	
	//初始化容器：扇形栅格向量、扇形栅格均值、扇形栅格地面标志位
	std::vector<cv::Point3d>ring_init,ring_temp;
	std::vector<std::vector<cv::Point3d>> gridcells;	//栅格向量
	
	std::vector<cv::Point3d> gridcells_median;	//栅格地面代表点
	std::vector<int> is_valid;	//栅格向量
	Sector Computer(DATA input, std::vector<cv::Point3d>& boundary_pts);
	void Init_Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer);
	void Show_Ground_Object(boost::shared_ptr<pcl::visualization::PCLVisualizer> &view, Sector src, std::vector<cv::Point3d> boundary_pts);
private:
	void Data_Process(DATA input, std::vector<std::vector<cv::Point3d>>& gridcells);
	void Get_Init_SeedPonit(std::vector<std::vector<cv::Point3d>>& gridcells, std::vector<cv::Point3d>& gridcells_median, std::vector<cv::Point3d>&ring_init);
	void Smooth_Init_Ring_SeedPoint(std::vector<cv::Point3d>& ring_init);
	void Croase_Ground_Detection(std::vector<std::vector<cv::Point3d>> gridcells, std::vector<cv::Point3d>& gridcells_median, std::vector<cv::Point3d>&ring_init);
	void Finetune_Ground_Detection();
	void GetParkingPoint(std::vector<cv::Point3d>& boundary_pts);
	void GetResult(DATA input, Sector& maps,std::vector<cv::Point3d>boundary_pts);
};

#endif