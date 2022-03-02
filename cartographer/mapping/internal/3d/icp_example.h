namespace cartographer {
namespace mapping {

	void wait_pcl();
	 Eigen::Matrix4f loadyaml3(std::string filename, std::string & pcdfn1, std::string & pcdfn2, std::string & method, double & resolution, Eigen::Quaternion<double> & quat0);

	Eigen::Matrix4f loadyaml_gt(std::string filename, std::string & pcdfn1, std::string & pcdfn2);
	void show_pcl_2cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2, std::string title);
	void icp_main2 (transform::Rigid3d * pose_observation_in_submap, transform::Rigid3d initial_ceres_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr scan,pcl::PointCloud<pcl::PointXYZ>::Ptr map, std::string icp_method, int pcl_viewerflag);
	Eigen::Matrix4f load2yaml();
	int icp_main3(int argc, char** argv); //from hdl_graph_ icp_example main 3/1/22
}  // namespace io
}  // namespace cartographer

