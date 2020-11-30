#include<pcl/visualization/pcl_visualizer.h>
#include<boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include<boost/thread/thread.hpp>
#include "Kalman.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

int t0=0,t1=0,t2=0;
double duration;
clock_t start,over;

boost::shared_ptr<pcl::visualization::PCLVisualizer>viewer (new pcl::visualization::PCLVisualizer ("line Viewer"));

void Tracker::on_HoughLines(const Mat &a,vector<cv::Vec4i> &lines){	
	//Mat mid;
	//Canny(a,mid,100, 200, 3);
	HoughLinesP(a, lines, 1, CV_PI/180, 40, 30, 5);//第六个最短线长，第七个连接点需要的距离
	//cout<<lines.size()<<endl;
	// for(size_t i = 0; i < lines.size(); i++){
	// 	Vec4i l = lines[i];
	// 	line( a, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(186,88,255), 1, CV_8U);
	// }
	 //cout<<lines[0][0]-<<endl;
	//  namedWindow("ab", WINDOW_NORMAL);
	//  imshow("ab", a);
	//  waitKey(100);
	 //imshow("mid", mid);
}

void Tracker::TransformCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,pcl::PointCloud<pcl::PointXYZI>::Ptr hough,pcl::PointCloud<pcl::PointXYZI>::Ptr new1)
{	
	pcl::PointXYZI min;
	pcl::PointXYZI max;
	pcl::getMinMax3D(*cloud,min,max);
	int xlen=(max.x-min.x)+1;
	int ylen=(max.y-min.y)+1;
	//cout<<xlen<<endl;
	Mat one = Mat::zeros(2*xlen,2*ylen,CV_8UC1);
	for (size_t i = 0; i < cloud->points.size(); ++i) {
        pcl::PointXYZI points = cloud->points[i];
		int x=(points.x-min.x)*2;
		int y=(points.y-min.y)*2;
		//a[x][y] = 255;
		one.at<unsigned char>(x,y) = 255;
	}
	vector<cv::Vec4i> lines;
	on_HoughLines(one,lines);

    //pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
	
	stringstream ss;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> single_color(cloud, 0, 255, 0);
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZI>(cloud, single_color, "sample cloud");
	viewer->updatePointCloud<pcl::PointXYZI>(cloud, single_color, "sample cloud");
	pcl::PointCloud<pcl::PointXYZI>::Ptr new2 (new pcl::PointCloud<pcl::PointXYZI>);
	pcl::KdTreeFLANN<pcl::PointXYZI> KDtree;
	KDtree.setInputCloud(hough);
	vector<bool> flag(hough->points.size(),false);
	for (size_t i = 0; i < lines.size(); ++i){
		Vec4i m = lines[i];
		pcl::PointXYZI p1,p2;
		p1.x = (m[1]/2 + min.x);
		p1.y = (m[0]/2 + min.y);
		p1.z = 0;
		p2.x = (m[3]/2 + min.x);
		p2.y = (m[2]/2 + min.y);
		p2.z = 0;
		double x1=p1.x,x2=p2.x,y1=p1.x,y2=p2.y;
		x1=cv::max(x1,x2);
		y1=cv::max(y1,y2);
		double k=(y1-y2)/(x1-x2);
		if(x1-x2<0.01) k=100;
		double b=y1-k*x1;
		double step=0.3;
		vector<int> indexp1,indexp2;
		vector<float> distance;
		while(x2<x1){
			x2+=step;
			y2=k*x2+b;
			pcl::PointXYZI p0;
			p0.x=x2;
			p0.y=y2;
			KDtree.radiusSearch(p0,2,indexp1,distance);
			for(size_t i = 0; i < indexp1.size(); ++i){
			if(flag[indexp1[i]]==true) continue;
			new2->points.push_back(hough->points[indexp1[i]]);
			flag[indexp1[i]]=true;
			}
		}
		ss<<"line"<<i;
		viewer->addLine(p1,p2,0,0,1,ss.str(),0);
		// temp_cloud->points.push_back(p2);
		//*cloud+=*temp_cloud;
	}
	*new1=*new2;
	viewer->spinOnce(100);
	viewer->removeAllShapes();
}

void Tracker::processPointcloud(const sensor_msgs::PointCloud2 &scan/*pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud*/)
{	
	//读取rosbag的消息，转化成pcl格式
	start = clock();
	pcl::PCLPointCloud2 pcl_pc;
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_raw(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_filtered(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr points_ransac(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr hough(new pcl::PointCloud<pcl::PointXYZI>);
	std::vector<pcl::PointIndices> cluster_indices;
    pcl_conversions::toPCL(scan,pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc,*points_raw);
	//roi
    for (size_t i = 0; i < points_raw->points.size(); ++i) {
        pcl::PointXYZI points=points_raw->points[i];
        if((points.x*points.x+points.y*points.y)>50.0&&abs(points.z)<0.5) {
            points.z=0;
            points_filtered->points.push_back(points);
        }
    }
    //TransformCloud(cloud,cloud,hough);
	//TransformCloud(cloud); //霍夫变换
	//欧式距离聚类
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
	tree->setInputCloud (points_filtered);
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
	ec.setClusterTolerance (2); //设置近邻搜索的搜索半径1m
	ec.setMinClusterSize (100);    //设置一个聚类需要的最少点数目
	ec.setMaxClusterSize (25000);  //设置一个聚类需要的最大数目
	ec.setSearchMethod (tree);     //设置点云的搜索方法
	ec.setInputCloud (points_filtered);
	ec.extract (cluster_indices);   //从点云中提取聚类并保存到cluster_indices中

	//根据index提取点云，并分别提取直线
	pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZI> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMethodType(pcl::SAC_LMEDS);//最小二乘
	seg.setDistanceThreshold(1);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>);//用来汇合
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
   		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
   		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     		cloud_cluster->points.push_back (points_filtered->points[*pit]); 
		seg.setInputCloud(cloud_cluster);
		seg.segment(*inliers, *coefficients2);
		for (int i = 0; i < inliers->indices.size(); ++i) {
			pcl::PointXYZI points=cloud_cluster->points.at(inliers->indices[i]);
			points.z=1;
        	points_ransac->points.push_back(points);
    	}
	}
	// for (size_t i = 0; i < hough->points.size(); ++i) {
    //     	pcl::PointXYZI points=hough->points[i];
	// 		points.z=1;
	// 		temp_cloud->points.push_back(points);
    // }
	// hough->clear();
	// *hough=*temp_cloud;
	// *cloud1=*cloud1+*cloud;
	// *hough=*hough+*cloud;
//	cloud1->width = cloud1->points.size();
//	cloud1->height = 1;
//	hough->width = hough->points.size();
//	hough->height = 1;
	//cloud1是输出点云,write pcd.
//	stringstream ss1,ss2;
//	ss1<<"/home/lhq/RANSAC/直线/"<<t1++<<".pcd";
//	pcl::io::savePCDFileASCII (ss1.str(), *cloud1);
//	cout<<t1<<endl;
//	ss2<<"/home/lhq/hough/直线/"<<t2++<<".pcd";
//	pcl::io::savePCDFileASCII (ss2.str(), *hough);
//	cout<<t2<<endl;
	//合并两个点云
	// *cloud=*cloud1+*cloud;

	over = clock();
	duration = (double)(over-start) / CLOCKS_PER_SEC;
	cout<<duration<<endl;
	sensor_msgs::PointCloud2 cloud_raw;
	sensor_msgs::PointCloud2 cloud_out;
    pcl::toROSMsg(*points_raw, cloud_raw);
    cloud_raw.header.frame_id = "velodyne";
    raw_pub.publish(cloud_raw);
    pcl::toROSMsg(*points_ransac, cloud_out);
    cloud_out.header.frame_id = "velodyne";
    out_pub.publish(cloud_out);
}

//void Tracker::readPCDfile(pcl::PointCloud<pcl::PointXYZI>::Ptr cd){
//	std::vector<std::string> res;
//	string path = "/media/lhq/SANDISK/label/merge_straight/";
//	for (const auto &entry : std::filesystem::directory_iterator(path)){
//		res.push_back(entry.path().string());
//	  }
//	sort(res.begin(),res.end(),cmp1);
//	// for(size_t i = 0;i<res.size();++i){
//	// std::cout<<res[i]<<std::endl;
//	// }
//	cout<<res.size()<<endl;
//	for(int i = 0; i < res.size(); ++i){
//		if(pcl::io::loadPCDFile<pcl::PointXYZI>(res[i], *cd)==-1){
//			cout << "error " << endl;
//			return;
//		}
//		processPointcloud(cd);
//		cd->clear();
//		//cout<<t0++<<endl;
//	}
//}

int main(int argc, char **argv)
{	
	std::cout << "kalman" << std::endl;
    ros::init(argc, argv, "kalman_node");
	viewer->setBackgroundColor(0.0, 0, 0);
	viewer->addCoordinateSystem(1.0);
    Tracker tracker;
//	pcl::PointCloud<pcl::PointXYZI>::Ptr cld(new pcl::PointCloud<pcl::PointXYZI>);
//	tracker.readPCDfile(cld);
    ros::spin();
    return 0;
}
