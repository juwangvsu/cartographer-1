#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <map>
#include <boost/algorithm/string/trim.hpp>
#include <string.h>
#include <vector>
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
template<class T> struct Point{
    template <typename NewType> Point<NewType> cast() const{
        return Point<NewType>();
    }
};
struct tt{
	int x;
	int y;
};
int main()
{
	std::string message("/home/root/xyz.csv");
	vector<string> tokens;
boost::split(tokens, message, boost::is_any_of("/"));
	std::cout<<tokens[0] << " " << tokens[tokens.size()-1];
	for(auto beg=tokens.begin(); beg!=tokens.end();++beg){
		std::cout << *beg <<" " ;
}

	Eigen::Quaterniond quat0(1, 2,0,0);
	std::cout<<"quat0: " <<quat0.w()<<" "<<quat0.x()<<" "<<quat0.y()<<" "<<quat0.z()<<"\n";
	Eigen::Matrix3d mat0 = quat0.toRotationMatrix();
	std::cout<<"mat0 (from quat0): \n" <<mat0 <<"\n";
	Eigen::Quaterniond quat1=quat0.conjugate();
	std::cout<<"quat0 conjugate: " <<quat1.w()<<" "<<quat1.x()<<" "<<quat1.y()<<" "<<quat1.z()<<"\n";
	std::cout<<"quat0: " <<quat0.w()<<" "<<quat0.x()<<" "<<quat0.y()<<" "<<quat0.z()<<"\n";
	quat0.normalize();
	std::cout<<"quat0: " <<quat0.w()<<" "<<quat0.x()<<" "<<quat0.y()<<" "<<quat0.z()<<"\n";
	Eigen::Matrix3d mat1 = quat0.toRotationMatrix();
	std::cout<<"mat1 (quat0 normalized): \n" <<mat1 <<"\n";
	Eigen::Quaterniond quat(1, 0,0,0);
	std::string ss0;
	std::ofstream outFile("/home/student//Documents/cartographer/test_ceres_pcd/sweep4.txt");
	std::ostringstream ssout("~/Documents/cartographer/test_ceres_pcd/sweep4.txt");
	ssout<<"hello"<<2;
	ssout.str("");//reset
	ssout<<"newhello"<<2;
	outFile<<"hello"<<3<<"hello";
	outFile<<ssout.str();
	outFile<<1<<","<<2;
	std::cout<<ssout.str();

    std::cout<<"Eigen::Quaterniond: "<< quat.w() <<"\n";
	Eigen::Matrix3d mat3 = Eigen::Quaterniond(1, 0,0,0).toRotationMatrix();
    Eigen::Vector3d v3d(-1., 0., 0.);
    Eigen::Transform<double, 3, Eigen::Affine > pose_transform;

    pose_transform.translation()=v3d;
    pose_transform.rotate(Eigen::AngleAxis<double>(0.5, Eigen::Vector3d::UnitX()));
    std::cout<<"Eigen::Transform: rotation"<< pose_transform.rotation() <<"\n";
    pose_transform.rotate(quat);
    pose_transform.linear()=mat3; //Eigen::AngleAxis<double>(0.5, Eigen::Vector3d::UnitX());
    //pose_transform.rotate() =mat3;
    //pose_transform.block<3,3>(0,0) =mat3;
    std::cout<<"Eigen::Matrix3f: "<< mat3 <<"\n";
    std::cout<<"Eigen::Vector3d: "<< v3d <<v3d[0]<<"\n";
    std::cout<<"Eigen::Transform: "<< pose_transform.translation() <<"\n";
    std::cout<<"Eigen::Transform: rotation"<< pose_transform.rotation() <<"\n";
    Eigen::Matrix<float, 3, 1> translation0(2,2,2);
    Eigen::Matrix<double, 3, 1> translation1(translation0.cast<double>());


    Point<float> p1;
     Point<double> p2;
    tt t1{1,2};
    //tt t2(1,2); bad structure initialization
    std::cout<<"struct tt: " <<t1.x<<"\n";
    std::vector<int> v = { 7, 5, 16, 8 };
    std::vector<int> intvec1;
    std::cout <<"intvec1.size(): "<< intvec1.size();
    intvec1.push_back(3);
    std::cout <<"\nintvec1.front(): "<< intvec1.front()<<"\n";

    //std::cout<<p1 <<" "<<p2 <<"\n";
    p2 = p1.cast<double>();
	std::array<int,3> a1{{1,2,3}};
	std::cout<<a1.data()[2];
	std::cout<<a1.at(2);

    std::cout<<"hello";
    std::string filename="testopt2.txt";

    std::cout<<NULL<<"----\n";
std::ifstream stream(filename.c_str());
//    std::ifstream stream("testopt2.txt");
    std::string myoptstr= std::string((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
    std::cout<< myoptstr;
map<string, string> mymap;
char *token;
token = strtok(const_cast<char*>(myoptstr.c_str()), "\n");
    while (token != NULL) {
        string s(token);
        size_t pos = s.find(":");
        //mymap[s.substr(0, pos)] = boost::algorithm::trim(s);
	string svalue = s.substr(pos + 1, string::npos);
	boost::algorithm::trim(svalue);
        mymap[s.substr(0, pos)] = svalue;
	//boost::algorithm::trim(s.substr(pos + 1, string::npos));
        token = strtok(NULL, "\n");

	if (s.substr(0, pos)=="tst"){
	std::cout<<"test tst"<<std::endl;
		char * token2;
		//std::cout<<svalue<<std::endl;
		token2 = strtok(const_cast<char*>(svalue.c_str()), ",");
		while (token2 != NULL) {
			string ss(token2);
			std::cout<<token2 <<" ";
        		token2 = strtok(NULL, ",");
		}

	}
    }

    std::cout<<"\n*****************\ncout mymap:\n";
    for (auto keyval : mymap)
        cout << keyval.first << ":" << keyval.second << endl;

      pcl::PointCloud<pcl::PointXYZ> cloud;

  // Fill in the cloud data
  cloud.width    = 5;
  cloud.height   = 1;
  cloud.is_dense = false;
  cloud.points.resize (cloud.width * cloud.height);

  for (auto& point: cloud)
  {
    point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    point.z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

    return 0;
}
