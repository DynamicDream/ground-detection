#include"sidewalk.h"

//解析文本文件功能函数,单个点云以（x,y,z）和反射强度、yaw、pitch角以及通道索引组成,实验数据是以五路通道拼接而成的点云为基础
void ParseFile(cv::String filename, DATA &input) {

	FILE *fp = fopen(filename.c_str(), "r");
	if (fp == NULL) {
		std::cout << "文件路径错误，打开失败！" << std::endl;
	}
	else {
		input.pts.clear();
		input.yaws.clear();
		input.pitches.clear();
		input.laser_id.clear();
		input.radius.clear();

		double x = 0, y = 0, z = 0, yaw = 0, pitch = 0;
		double laser_id = 0, intensity = 0;//雷达通道数

		while (!feof(fp)) {
			fscanf(fp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &intensity, &laser_id, &pitch, &yaw);

			if (abs(yaw) < 100) {
				input.yaw_max = MAX(yaw, input.yaw_max);
				input.yaw_min = MIN(yaw, input.yaw_min);
				input.pitch_min = MIN(pitch, input.pitch_min);
				input.pitch_max = MAX(pitch, input.pitch_max);

				input.radius.emplace_back(sqrt(x*x + y * y));
				input.pts.emplace_back(cv::Point3d(x, y, z));
				input.yaws.emplace_back(yaw);
				input.pitches.emplace_back(pitch);
				input.laser_id.emplace_back(int(laser_id));
			}
		}
		//std::cout << "yaw_min: " << input.yaw_min <<" yaw_max: "<< input.yaw_max 
		//	<< "pitch_min: " << input.pitch_min << " pitch_max: " << input.pitch_max << std::endl;
		fclose(fp);
	}
}

//初始化点云显示
void GroundDetector::Init_Viewer(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer) {
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("test"));

	viewer->setBackgroundColor(0.0, 0, 0); //设置背景色为黑色
	viewer->addCoordinateSystem(1); //建立空间直角坐标系
	//viewer->setCameraPosition(0,0,200); //设置坐标原点
	viewer->initCameraParameters();   //初始化相机参数
}

//显示地面及障碍物
void GroundDetector::Show_Ground_Object(boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer, Sector src,std::vector<cv::Point3d> boundary_pts) {
	viewer->removeAllShapes();
	viewer->removeAllPointClouds();
	pcl::PointCloud<pcl::PointXYZ>::Ptr Ground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Object(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr BAN(new pcl::PointCloud<pcl::PointXYZ>);
	//FILE *fp = fopen("wwww.txt","w+");
	for (int idx = 0; idx < src.pts.size(); idx++) {
		if (src.is_ground[idx]) {
			Ground->push_back(pcl::PointXYZ(src.pts[idx].x, src.pts[idx].y, src.pts[idx].z));
			//fprintf(fp,"%f,%f,%f,%d,%d,%d\n", src.pts[idx].x, src.pts[idx].y, src.pts[idx].z,0,255,0);
		}
		else {
			Object->push_back(pcl::PointXYZ(src.pts[idx].x, src.pts[idx].y, src.pts[idx].z));
			//fprintf(fp, "%f,%f,%f,%d,%d,%d\n", src.pts[idx].x, src.pts[idx].y, src.pts[idx].z, 255, 0, 0);
		}
	}
	//fclose(fp);
	//exit(0);
	for (int k = 0; k < boundary_pts.size(); k++) {
		BAN->points.push_back(pcl::PointXYZ(boundary_pts[k].x, boundary_pts[k].y, boundary_pts[k].z));
	}
	//std::cout << "size: " << cloud->size() << std::endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_ground(Ground, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_object(Object, 255, 0, 0);
	viewer->addPolygon<pcl::PointXYZ>(BAN, "boundary_pts");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "boundary_pts");
	viewer->addPointCloud(Ground, color_ground, "ground");
	viewer->addPointCloud(Object, color_object, "object");
	viewer->spinOnce(1);
}

//根据Z轴排序
bool Cmp_Z(cv::Point3d p1, cv::Point3d p2) {
	if (p1.z < p2.z) {
		return true;
	}
	return false;
}

//根据径向半径排序
bool Cmp_Radius(cv::Point3d p1, cv::Point3d p2) {
	double r1 = p1.x*p1.x + p1.y + p1.y;
	double r2 = p2.x*p2.x + p2.y + p2.y;
	if (r1 < r2) {
		return true;
	}
	return false;
}

//坡度判断：在距离地面一定高度内的点云的坡度均值是否大于阈值
bool Calculate_Slope(std::vector<cv::Point3d>grid, cv::Point3d mean_p) {
	int count = 0;
	double mslop = 0;
	//double zmin = 0, zmax = -1e3;
	
	for (int id = 0; id < grid.size(); id++) {
		cv::Point3d offset = grid[id] - mean_p;
		if (offset.z < 0.5) {
			double slop = atan(MIN(abs(offset.z /0.1),1.0)) * 180 / CV_PI;
			mslop += slop;
			count++;
		}
	}
	
	mslop /= (count+1e-3);
	
	if (mslop < 25) {
		return true;
	}
	return false;
}

//根据地面平整性判断是否为障碍物
bool Is_Object(std::vector<cv::Point3d>grid,cv::Point3d gridmean) {
	double zmin = gridmean.z;
	double max_z = zmin;
	for (int id = 0; id < grid.size(); id++) {
		if (grid[id].z - zmin < 1.7) {
			max_z = MAX(max_z, grid[id].z);
		}
	}
	
	if (max_z - zmin < 0.2) {
		return true;
	}
	return false;
}

//根据地面高度差判断是否为障碍物
int Is_Boundary(std::vector<cv::Point3d>grid, cv::Point3d seedpoint) {
	int count = 0;
	double all_delta_z = 0;
	for (int id = 0; id < grid.size(); id++) {
		cv::Point3d offset = grid[id] - seedpoint;
		if (grid[id].z<= 0) {
			all_delta_z += abs(offset.z);
			count++;
		}
	}

	if (count) {
		if (all_delta_z / (count) > 0.15)
			return 0;
		else
			return 1;
	}

	return -1;
}

//保存栅格化结果
void Save_PatchWork(std::vector<std::vector<cv::Point3d>> gridcells) {
	srand(time(NULL));
	FILE *fp = fopen("PatchWork.txt","w+");
	cv::Vec3b color = cv::Vec3b(rand()%255, rand() % 255, rand() % 255);
	for (int id = 0; id < gridcells.size(); id++) {
		color = cv::Vec3b(rand() % 255, rand() % 255, rand() % 255);
		for (int k = 0; k < gridcells[id].size(); k++) {
			fprintf(fp,"%f,%f,%f,%d,%d,%d\n", gridcells[id][k].x, gridcells[id][k].y, gridcells[id][k].z, color[0],color[1],color[2]);
		}
	}
	fclose(fp);
	exit(0);
}

//依照水平角排序
bool Sort_Angle(cv::Point3d p1,cv::Point3d p2) {
	double r1 = sqrt(p1.x*p1.x+p1.y*p1.y);
	double r2 = sqrt(p2.x*p2.x + p2.y*p2.y);
	double a1 = asin(p1.y / r1)*180;
	double a2 = asin(p2.y / r2)*180;
	if (a1 > a2|| (a1 == a2 && r1 < r2)) {
		return true;
	}
	else {
		return false;
	}
}

//将边界点根据水平角重新排序
void Sort_Boundary(std::vector<cv::Point3d>& cnt) {
	std::sort(cnt.begin(),cnt.end(),Sort_Angle);
	cnt.emplace_back(cv::Point3d(0,0,-1.72));
}

/*
	功能:M1数据处理
	注意:为了避免多路拼接不佳，导致重叠区域同一目标高程分层的问题,每个水平角度仅保留单一通道的数据
*/
void GroundDetector::Data_Process(DATA input, std::vector<std::vector<cv::Point3d>>& gridcells) {
	this->grid_cols = int((input.yaw_max - input.yaw_min) / horizontal_resolution);//栅格列数
	this->grid_rows = int((distance_max - distance_min) / radius_resolution);//栅格行数
	gridcells = std::vector<std::vector<cv::Point3d>>((grid_rows + 1)*(grid_cols + 1));	
	is_valid = std::vector<int>((grid_rows + 1)*(grid_cols + 1),0);
	std::vector<int> gridcells_channel_idx = std::vector<int>((grid_cols + 1), -1);	//栅格向量
	//将点云栅格化
	for (int id = 0; id < input.pts.size(); id++) {
		int x = int((input.radius[id] - distance_min) / radius_resolution);
		int y = int((input.yaws[id] - input.yaw_min) / horizontal_resolution);
		if (x<0 || y<0 || x>grid_rows || y>grid_cols) {
			continue;
		}
		if (gridcells_channel_idx[y] == -1) {
			gridcells_channel_idx[y] = input.laser_id[id];
			gridcells[x*(grid_cols + 1) + y].emplace_back(input.pts[id]);
			
		}
		else if (gridcells_channel_idx[y] == input.laser_id[id]) {
			gridcells[x*(grid_cols + 1) + y].emplace_back(input.pts[id]);
			
		}
	}
}
//检索中值点
int Search_Median(std::vector<cv::Point3d>cell,cv::Point3d& gridmean) {
	std::sort(cell.begin(), cell.end(), Cmp_Z);
	std::vector<cv::Point3d> temp;
	for (int id = 0; id < cell.size(); id++) {
		if(cell[id].z + cell[0].z <=0.3)
			temp.emplace_back(cell[id]);
	}
	if (!temp.empty()) {
		gridmean = temp[temp.size() / 2];
		return temp.size();
	}
	gridmean = cell[0];
	gridmean.z = 0;
	return 1;

}
/*
	功能：获取扇形栅格化后，每列的初始种子点
	输入：待写入的栅格点云向量、栅格中值点向量、栅格种子点
	输出：已填充的栅格点云向量、栅格中值点向量、栅格种子点
*/
void GroundDetector::Get_Init_SeedPonit(std::vector<std::vector<cv::Point3d>>& gridcells,std::vector<cv::Point3d>& gridcells_median, std::vector<cv::Point3d>&ring_init) {
	gridcells_median = std::vector<cv::Point3d>((grid_rows + 1)*(grid_cols + 1), cv::Point3d(0, 0, 0));	//存放中值点
	//std::vector<int> gridcells_nums = std::vector<int>((grid_rows + 1)*(grid_cols + 1), 0);
	ring_init = std::vector<cv::Point3d>(grid_cols + 1, cv::Point3d(0, 0, 0));//存放初始种子点
	//计算出每个栅格的代表点
	for (int id = 0; id < gridcells.size(); id++) {
		if (gridcells[id].size()) {
			Search_Median(gridcells[id],gridcells_median[id]);
		}
	}
	//确定障碍物区域,并初始化每列的起始疑似地面区域
	for (int c = 0; c <= grid_cols; c++) {
		for (int r = 0; r <= grid_rows; r++) {
			if (gridcells[r*(grid_cols+1) + c].size()) {
				if (ring_init[c] == cv::Point3d(0, 0, 0) ) {
					ring_init[c] = gridcells_median[r*(grid_cols+1) + c];
					break;
				}
			}
		}
	}
}

//平滑地面点
void GroundDetector::Smooth_Init_Ring_SeedPoint(std::vector<cv::Point3d>& ring_init) {
	int seedpoint_index = -1;
	for (int c = 0; c < ring_init.size(); c++) {
		//最低地面点仅在车辆横±5m范围内选取（默认此范围内才会出现地面）
		if (ring_init[c].y >= -5 && ring_init[c].y <= 5) {
			if (seedpoint_index == -1) {
				seedpoint_index = c;
			}
			else if (ring_init[c].z < ring_init[seedpoint_index].z) {
				seedpoint_index = c;
			}
		}
	}

	std::cout << "最低种子点: " << ring_init[seedpoint_index] << std::endl;
	//由地面种子点向两侧延伸平滑
	cv::Point3d ground_seedponit = ring_init[seedpoint_index];
	for (int c = seedpoint_index - 1; c >= 0; c--) {
		if (ring_init[c] != cv::Point3d(0, 0, 0)) {	//是否包含有效数据
			//判断当前区域是否比地面种子区域距离雷达更近
			if (ring_init[c].z - ring_init[seedpoint_index].z > 0.2) {
				ring_init[c].z = ring_init[seedpoint_index].z + 0.2;
			}
			else {
				ground_seedponit = ring_init[c];
			}
		}
	}
	ground_seedponit = ring_init[seedpoint_index];
	for (int c = seedpoint_index + 1; c < ring_init.size(); c++) {
		if (ring_init[c] != cv::Point3d(0, 0, 0)) {	//是否包含有效数据
			//判断当前区域是否比地面种子区域距离雷达更近
			if (ring_init[c].z - ring_init[seedpoint_index].z > 0.2) {
				ring_init[c].z = ring_init[seedpoint_index].z + 0.2;
			}
			else {
				ground_seedponit = ring_init[c];
			}

		}
	}
	ring_temp = ring_init;
}

//根据坡度和高度差粗略判断非地面点
void GroundDetector::Croase_Ground_Detection(std::vector<std::vector<cv::Point3d>> gridcells, std::vector<cv::Point3d>& gridcells_median, std::vector<cv::Point3d>&ring_init) {
	for (int c = 0; c <= grid_cols; c++) {
		bool object_ = true;
		for (int r = 0; r <= grid_rows && object_; r++) {
			if (gridcells_median[r*(grid_cols + 1) + c] != cv::Point3d(0, 0, 0)) {
				cv::Point3d offset = gridcells_median[r*(grid_cols + 1) + c] - ring_init[c];
				if (offset.z <= 0.15) {
					if (Calculate_Slope(gridcells[r*(grid_cols + 1) + c], gridcells_median[r*(grid_cols + 1) + c])) {
						is_valid[r*(grid_cols + 1) + c] = 1;
						ring_init[c] = gridcells_median[r*(grid_cols + 1) + c];
					}
					else {
						gridcells_median[r*(grid_cols + 1) + c] = ring_init[c];
						is_valid[r*(grid_cols + 1) + c] = -1;
						object_ = false;
					}
				}
				else {
					std::vector<cv::Point3d> seeds;
					gridcells_median[r*(grid_cols + 1) + c].z = ring_init[c].z;
					if (Calculate_Slope(gridcells[r*(grid_cols + 1) + c], ring_init[c])) {
						for (int id = 0; id < gridcells[r*(grid_cols + 1) + c].size(); id++) {
							offset = gridcells[r*(grid_cols + 1) + c][id] - gridcells_median[r*(grid_cols + 1) + c];
							if (offset.z <= 0.15) {
								seeds.emplace_back(gridcells[r*(grid_cols + 1) + c][id]);
							}
						}
						if (!seeds.empty()) {
							gridcells_median[r*(grid_cols + 1) + c] = ring_init[c] = seeds[seeds.size() / 2];
						}
						is_valid[r*(grid_cols + 1) + c] = 1;
					}
					else {
						is_valid[r*(grid_cols + 1) + c] = -1;
					}
				}
			}
		}
	}
}

//将可能被漏检的区域依照平面性重新筛选出来
void GroundDetector::Finetune_Ground_Detection() {
	for (int c = 0; c <= grid_cols; c++) {
		for (int r = 0; r <= grid_rows; r++) {
			if (is_valid[r*(grid_cols + 1) + c] == -1 && Is_Object(gridcells[r*(grid_cols + 1) + c],gridcells_median[r*(grid_cols + 1) + c])) {
				for (int id = 0; id < gridcells[r*(grid_cols + 1) + c].size(); id++) {
					cv::Point3d offset = gridcells[r*(grid_cols + 1) + c][id] - ring_init[c];
					if (offset.z <= 0.15) {
						is_valid[r*(grid_cols + 1) + c] = 1;
						break;
					}
				}
			}
		}
	}
}

//将地面检测结果与边界点结合，获得最终的可行驶区域
void GroundDetector::GetResult(DATA input,Sector& maps,std::vector<cv::Point3d>boundary_pts) {
	std::vector<int>b2d_x(grid_cols+1,0);
	std::vector<cv::Point>b2d;
	//3d边界点转栅格2d坐标点
	for (int id = 0; id < boundary_pts.size()-1; id++) {
		double r = sqrt(boundary_pts[id].x*boundary_pts[id].x + boundary_pts[id].y*boundary_pts[id].y)+1e-3;
		int x = MAX(int((r - distance_min) / radius_resolution),0);//行数
		int y = int((asin(boundary_pts[id].y/r)*180/CV_PI - input.yaw_min) / horizontal_resolution);//行数
		b2d.insert(b2d.begin(),cv::Point(x,y));
	}
	//根据相邻边界点计算贝塞尔曲线（两个控制点就相当于直线）
	for (int id = 0; id < b2d.size() - 1; id++) {
		for (int k = b2d[id].y; k <= b2d[id + 1].y; k++) {
			double t = 1.0*(k - b2d[id].y) / (b2d[id + 1].y - b2d[id].y + 1e-3);
			b2d_x[k] = int((1 - t)*b2d[id].x + b2d[id + 1].x*t);
		}
	}

	//根据边界点筛选出可行驶区域
	for (int id = 0; id < b2d_x.size(); id++) {
		if (b2d_x[id]) {
			for (int r = 0; r <= grid_rows; r++) {
				if (r <= b2d_x[id]) {
					if (is_valid[r *(grid_cols + 1) + id] == 1) {
						for (int k = 0; k < gridcells[r * (grid_cols + 1) + id].size(); k++) {
							cv::Point3d offset = gridcells[r * (grid_cols + 1) + id][k] - gridcells_median[r * (grid_cols + 1) + id];
							if (offset.z < 0.15) {
								maps.pts.emplace_back(gridcells[r * (grid_cols + 1) + id][k]);
								maps.is_ground.emplace_back(true);
							}
							else {
								maps.pts.emplace_back(gridcells[r * (grid_cols + 1) + id][k]);
								maps.is_ground.emplace_back(false);
							}
						}

					}
					else {
						for (int k = 0; k < gridcells[r * (grid_cols + 1) + id].size(); k++) {
							maps.pts.emplace_back(gridcells[r * (grid_cols + 1) + id][k]);
							maps.is_ground.emplace_back(false);
						}
					}
				}
				else {
					for (int k = 0; k < gridcells[r * (grid_cols + 1) + id].size(); k++) {
						maps.pts.emplace_back(gridcells[r * (grid_cols + 1) + id][k]);
						maps.is_ground.emplace_back(false);
					}
				}
			}
		}
		else {
			for (int r = 0; r <= grid_rows; r++) {
				for (int k = 0; k < gridcells[r*(grid_cols + 1) + id].size(); k++) {
					maps.pts.emplace_back(gridcells[r*(grid_cols + 1) + id][k]);
					maps.is_ground.emplace_back(false);
				}
			}
		}
	}
	////FILE *fp = fopen("woc.txt","w+");
	////根据边界点计算出每列的终止行索引
	//for (int c = 0; c <= grid_cols; c++) {
	//	for (int r = 0; r <= grid_rows; r++) {
	//		if (is_valid[r*(grid_cols + 1) + c] == 1 ) {
	//			for (int id = 0; id < gridcells[r*(grid_cols + 1) + c].size(); id++) {
	//				cv::Point3d offset = gridcells[r*(grid_cols + 1) + c][id] - gridcells_median[r*(grid_cols + 1) + c];
	//				if (offset.z < 0.15) {
	//					maps.pts.emplace_back(gridcells[r*(grid_cols + 1) + c][id]);
	//					maps.is_ground.emplace_back(true);
	//				}
	//				else {
	//					maps.pts.emplace_back(gridcells[r*(grid_cols + 1) + c][id]);
	//					maps.is_ground.emplace_back(false);
	//				}
	//			}
	//			//fprintf(fp, "%f,%f,%f\n", gridcells_median[r*(grid_cols + 1) + c].x, gridcells_median[r*(grid_cols + 1) + c].y, gridcells_median[r*(grid_cols + 1) + c].z);
	//		}
	//		else {
	//			for (int id = 0; id < gridcells[r*(grid_cols + 1) + c].size(); id++) {
	//				maps.pts.emplace_back(gridcells[r*(grid_cols + 1) + c][id]);
	//				maps.is_ground.emplace_back(false);
	//			}
	//		}
	//		
	//	}
	//}
	////fclose(fp);
	////exit(0);
}

/*	边界点主要有以下情况：
	情况1：遇到障碍物时，此时终止，并将该栅格点云保存为边界点;
	情况2：遇到非障碍物的区域（悬挂物）时，此时跳过,并将其点云保留为边界点;
	情况3：遇到地面区域,直接跳过.*/
void GroundDetector::GetParkingPoint(std::vector<cv::Point3d>& boundary_pts) {
	int bins = this->ring_init.size();
	boundary_pts = std::vector<cv::Point3d>(bins,cv::Point3d(0,0,0));
	for (int c = 0; c <= grid_cols; c++) {
		int statue = -1;//状态：0表示此处为障碍物、1表示此处为地面、-1表示此处为悬挂物
		bool is_init = false;
		int pre_statue = 1;
		cv::Point3d pre_seedpoint(0,0,0);
		for (int r = 0; r <= grid_rows; r++) {
			if (is_valid[r*(grid_cols + 1) + c] == 1) {
				//刚遇到第一个有效的区域
				if (!is_init) {
					statue = Is_Boundary(gridcells[r*(grid_cols + 1) + c], ring_temp[c]);
					pre_seedpoint = ring_temp[c];
					is_init = true;
				}
				else {
					statue = Is_Boundary(gridcells[r*(grid_cols + 1) + c], pre_seedpoint);
				}
				if (statue == 1) {
					pre_seedpoint = gridcells_median[r*(grid_cols + 1) + c];
				}
				else if (statue == 0) {
					boundary_pts[c] = pre_seedpoint;
					break;
				}
				else {
					boundary_pts[c] = pre_seedpoint;
				}
			}
			else if(is_valid[r*(grid_cols + 1) + c] ==  0 ){
				boundary_pts[c] = pre_seedpoint;
			}
			else if(is_valid[r*(grid_cols + 1) + c] == -1){
				if (is_init) {
					boundary_pts[c] = pre_seedpoint;
				}
				else {
					boundary_pts[c] = ring_temp[c];
				}
				break;
			}
		}
	}
	for (int id = 0; id < boundary_pts.size(); id++) {
		if (boundary_pts[id] == cv::Point3d(0, 0, 0)) {
			boundary_pts.erase(boundary_pts.begin() + id);
			id--;
		}
	}
	Sort_Boundary(boundary_pts);
}
/*	功能：可行驶区域检测
	输入：点云相关数据、边界点向量容器
	输出：可行驶区域检测结果、检测到的边界点向量*/
Sector GroundDetector::Computer(DATA input, std::vector<cv::Point3d>& boundary_pts) {
	Sector Result;	//可行驶区域检测结果
	Data_Process( input, gridcells);
	Get_Init_SeedPonit(gridcells, gridcells_median,ring_init);
	Smooth_Init_Ring_SeedPoint(ring_init);
	Croase_Ground_Detection(gridcells, gridcells_median, ring_init);
	Finetune_Ground_Detection();
	GetParkingPoint(boundary_pts);
	GetResult(input, Result, boundary_pts);
	return Result;
}