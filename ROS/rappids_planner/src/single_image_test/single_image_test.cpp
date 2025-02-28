#include "../../include/rappids/depth_image_planner.hpp"
#include "ros/ros.h"

using namespace CommonMath;
using namespace RectangularPyramidPlanner;

std::string depth_image_path;
static void display_image(const cv::Mat& depth_image, 
                          double pixelMin, double pixelMax);
void SaveTrajectories(std::vector<TrajectoryTest>& trajectories);
void SavePyramids(const std::vector<Pyramid>& pyramids, const double f,
                  const double cx, const double cy);
std::string ToCSV(const Vec3 v);
void GetFeaturePoints(std::vector<Vec3>& features, const std::string& filename);

struct planner_specs {
  double computation_time;
  double physical_vehicle_radius;
  double vehicle_radius_for_planning;
  double minimum_collision_distance;
  double min_sample_depth;
  double max_sample_depth;
  double min_sample_time;
  double max_sample_time;
};

struct depth_image_specs {
  double depth_scale;
  double f;
  double cx;
  double cy;
};

struct trajectory_specs {
  double desired_yaw;
  double min_allowed_thrust;
  double max_allowed_thrust;
  double max_allowed_ang_vel;
  double max_allowed_vel;
  double min_allowed_height;
};

struct goal_specs {
  double posx_picture;
  double posy_picture;
  double posz_picture;
};

static void display_image(const cv::Mat& depth_image, double pixelMin,
  double pixelMax);

int main(int argc, char** argv) {
  ros::init(argc, argv, "rappids_node");
  ROS_INFO("initializing the rappids node.");
  ros::NodeHandle nh;
  bool get_param_success = true;

  double k_perc;
  planner_specs test_planner_spec;
  get_param_success &= nh.getParam("single_image_test/planner_specs/computation_time",
    test_planner_spec.computation_time);
  get_param_success = get_param_success && nh.getParam("single_image_test/planner_specs/k_perc", k_perc);
  get_param_success &= nh.getParam("single_image_test/planner_specs/physical_vehicle_radius",
    test_planner_spec.physical_vehicle_radius);
  get_param_success &= nh.getParam("single_image_test/planner_specs/vehicle_radius_for_planning",
    test_planner_spec.vehicle_radius_for_planning);
  get_param_success &= nh.getParam("single_image_test/planner_specs/minimum_collision_distance",
    test_planner_spec.minimum_collision_distance);
  get_param_success &= nh.getParam("single_image_test/planner_specs/min_sample_depth",
    test_planner_spec.min_sample_depth);
  get_param_success &= nh.getParam("single_image_test/planner_specs/max_sample_depth",
    test_planner_spec.max_sample_depth);
  get_param_success &= nh.getParam("single_image_test/planner_specs/min_sample_time",
    test_planner_spec.min_sample_time);
  get_param_success &= nh.getParam("single_image_test/planner_specs/max_sample_time",
    test_planner_spec.max_sample_time);

  depth_image_specs test_depth_image_specs;
  get_param_success &= nh.getParam("single_image_test/test/image_path", depth_image_path);
  get_param_success &= nh.getParam("single_image_test/depth_camera/depth_scale", test_depth_image_specs.depth_scale);
  get_param_success &= nh.getParam("single_image_test/test/image_f", test_depth_image_specs.f);
  get_param_success &= nh.getParam("single_image_test/test/image_cx", test_depth_image_specs.cx);
  get_param_success &= nh.getParam("single_image_test/test/image_cy", test_depth_image_specs.cy);

  trajectory_specs test_trajectory_specs;
  get_param_success &= nh.getParam("single_image_test/trajectory/desired_yaw", test_trajectory_specs.desired_yaw);
  get_param_success &= nh.getParam("single_image_test/trajectory/min_allowed_thrust",
    test_trajectory_specs.min_allowed_thrust);
  get_param_success &= nh.getParam("single_image_test/trajectory/max_allowed_thrust",
    test_trajectory_specs.max_allowed_thrust);
  get_param_success &= nh.getParam("single_image_test/trajectory/max_allowed_ang_vel",
    test_trajectory_specs.max_allowed_ang_vel);
  get_param_success &= nh.getParam("single_image_test/trajectory/max_allowed_vel",
    test_trajectory_specs.max_allowed_vel);
  get_param_success &= nh.getParam("single_image_test/trajectory/min_allowed_height",
    test_trajectory_specs.min_allowed_height);

  goal_specs test_goal_specs;
  get_param_success &= nh.getParam("single_image_test/test/goal_posx_picutre", test_goal_specs.posx_picture);
  get_param_success &= nh.getParam("single_image_test/test/goal_posy_picutre", test_goal_specs.posy_picture);
  get_param_success &= nh.getParam("single_image_test/test/goal_posz_picutre", test_goal_specs.posz_picture);

  bool with_perception;
  with_perception &= nh.getParam("single_image_test/test/with_perception", with_perception);

  if (!get_param_success){
    ROS_ERROR("Some ros parameters are not loaded successfully!");
    return -1;
  }

  ros::Time planningStartTime = ros::Time::now();
  // cv::Mat depthImg = cv_bridge::toCvShare(msg,
  // sensor_msgs::image_encodings::TYPE_16UC1)->image;
  cv::Mat depthImg = cv::imread(depth_image_path + "example.png", cv::IMREAD_ANYDEPTH);
  ROS_INFO("Loaded image");

  //cv::Mat depthImg = 10000 * cv::Mat::ones(480, 640, CV_16UC1);

  double min, max;
  cv::minMaxLoc(depthImg, &min, &max);
  ROS_INFO("Min value: %f", min);
  ROS_INFO("Max value: %f", max);

  // Initialize the planner
  DepthImagePlanner planner(
    depthImg, test_depth_image_specs.depth_scale, test_depth_image_specs.f,
    test_depth_image_specs.cx, test_depth_image_specs.cy,
    test_planner_spec.physical_vehicle_radius,
    test_planner_spec.vehicle_radius_for_planning,
    test_planner_spec.minimum_collision_distance,
    test_trajectory_specs.min_allowed_thrust,
    test_trajectory_specs.max_allowed_thrust,
    test_trajectory_specs.max_allowed_ang_vel,
    test_trajectory_specs.max_allowed_vel,
    test_trajectory_specs.min_allowed_height);

  std::random_device rd;
  planner.SetRandomSeed(rd());

  // Find trajectories at the stereo camera's picture frame:
  // camera's pointed direction is Z direction.
  Vec3 vel_picture =  Vec3(0.0, 0.0, 0.0);
  Vec3 acc_picture =  Vec3(0.0, 0.0, 0.0);
  Vec3 g_picture = Vec3(0.0, 9.81, 0.0);

  RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator traj(
    Vec3(0.0, 0.0, 0.0), vel_picture, acc_picture, g_picture);
  std::vector<TrajectoryTest> trajectories;

  // current goal in the picture frame
  Vec3 current_goal_picture = Vec3(test_goal_specs.posx_picture, test_goal_specs.posy_picture, test_goal_specs.posz_picture);  

  // Find trajectories
  SamplingParameters samplingParameters;
  samplingParameters.minDepth = test_planner_spec.min_sample_depth;
  samplingParameters.maxDepth = test_planner_spec.max_sample_depth;
  samplingParameters.minTime = test_planner_spec.min_sample_time;
  samplingParameters.maxTime = test_planner_spec.max_sample_time;

  //rotation from body frame to camera frame (i.e. transformation matrix from camera frame to body frame)
  Rotation depthCamAtt = Rotation::FromEulerYPR(-90.0 * M_PI / 180.0, 0.0 * M_PI / 180.0, -90.0 * M_PI / 180.0);
  Rotation estVehicleAtt = Rotation::Identity();
  Vec3 estVehiclePos(0.0, 0.0, 1.0);
  if (with_perception){
    
    int imageFPS = 15; 
    double exposureTime = 0.008; //in second
    std::string feature_filename = depth_image_path + "single_image_test_feature.csv";
    std::vector<Vec3> features;
    GetFeaturePoints(features, feature_filename);
    //convert from the camera frame into odometry frame
    for (int i = 0; i < features.size(); ++i){
      features[i] = estVehicleAtt * depthCamAtt * features[i];
    }
    // for (const Vec3& feature : features){
    //   std::cout << feature.x << " " << feature.y << " " << feature.z << std::endl;
    // }
    double speed_cost = 0;
    double perception_cost = 0;
    bool trajectory_found = planner.FindLowestCostTrajectory(traj, trajectories, speed_cost, perception_cost, k_perc, test_planner_spec.computation_time,
        samplingParameters, current_goal_picture, features, depthCamAtt, estVehicleAtt, estVehiclePos, imageFPS, exposureTime);
  }else{
    double speed_cost = 0;
    bool trajectory_found = planner.FindLowestCostTrajectory(
      traj, trajectories, speed_cost, test_planner_spec.computation_time,
      samplingParameters, current_goal_picture,
      estVehicleAtt * depthCamAtt, estVehiclePos);
  }

  std::vector<Pyramid> pyramids = planner.GetPyramids();

  SaveTrajectories(trajectories);
  SavePyramids(pyramids, test_depth_image_specs.f, test_depth_image_specs.cx, test_depth_image_specs.cy);

  // planner internal logging
  // should not use _traj* variables, which is only for found collision-free
  // trajectories
  if(!with_perception){
    ROS_INFO(
    "%6d collision-free / %6d pyramids / %6d velocity / %6d "
    "feasible / %6d low cost / %6d generated",
    planner.GetNumCollisionFree(), planner.GetNumPyramids(),
    planner.GetNumVelocityChecks(), planner.GetNumCollisionChecks(),
    planner.GetNumCostChecks(), planner.GetNumTrajectoriesGenerated());
  }else{
    ROS_INFO(
    " %6d low cost /  %6d collision-free / %6d pyramids / "
    "%6d velocity / %6d feasible / %6d generated / ",
    planner.GetNumCostChecks(), planner.GetNumCollisionFree(), planner.GetNumPyramids(),
    planner.GetNumVelocityChecks(), planner.GetNumCollisionChecks(), planner.GetNumTrajectoriesGenerated());   
  }

}

static void display_image(const cv::Mat& depth_image, double pixelMin,
  double pixelMax) {
  cv::Mat displayable_image;
  double alpha = 255.0 / (pixelMax - pixelMin);
  double beta = -alpha * pixelMin;
  depth_image.convertTo(displayable_image, CV_8UC1, alpha, beta);
  // BLUE = FAR
  // RED = CLOSE
  cv::applyColorMap(displayable_image, displayable_image, cv::COLORMAP_HOT);
  cv::imshow("display", displayable_image);
  cv::waitKey(1);
}

void SaveTrajectories(std::vector<TrajectoryTest>& trajectories){
  //file for logging:
  std::ofstream logfile;
  logfile.open(depth_image_path + "single_image_test_trajectories.csv");  
  
  for (auto & trajectory_test : trajectories) {
    enum TrajectoryTestResult result = trajectory_test.result;    
    CommonMath::Trajectory trajectory_params = trajectory_test.traj.GetTrajectory();

    double final_time = trajectory_test.traj.GetFinalTime();
    Vec3 coeff0 = trajectory_params[0]; //coeffs for t^5, (x,y,z) axes
    Vec3 coeff1 = trajectory_params[1]; //coeffs for t^4, (x,y,z) axes
    Vec3 coeff2 = trajectory_params[2]; //coeffs for t^3, (x,y,z) axes
    Vec3 coeff3 = trajectory_params[3]; //coeffs for t^2, (x,y,z) axes
    Vec3 coeff4 = trajectory_params[4]; //coeffs for t^1, (x,y,z) axes
    Vec3 coeff5 = trajectory_params[5]; //constant terms, (x,y,z) axes

    logfile << result << " ";
    logfile << final_time << " ";
    logfile << ToCSV(coeff0);
    logfile << ToCSV(coeff1);
    logfile << ToCSV(coeff2);
    logfile << ToCSV(coeff3);
    logfile << ToCSV(coeff4);
    logfile << ToCSV(coeff5);

    logfile << "\n";
  }
  logfile.close();
}

void SavePyramids(const std::vector<Pyramid>& pyramids, const double f, const double cx, const double cy){
  //file for logging:
  std::ofstream logfile;
  logfile.open(depth_image_path + "single_image_test_pyramids.csv");  
  
  for (auto & pyramid : pyramids) {
    double depth = pyramid.depth; //in [m]
    double right_pixel_bound = pyramid.rightPixBound; //in pixel
    double top_pixel_bound = pyramid.topPixBound; //in pixel
    double left_pixel_bound = pyramid.leftPixBound; //in pixel
    double bottom_pixel_bound = pyramid.bottomPixBound; //in pixel

    Vec3 top_left_corner = Vec3(depth * (left_pixel_bound - cx) / f, depth * (top_pixel_bound - cy) / f, depth);
    Vec3 top_right_corner = Vec3(depth * (right_pixel_bound - cx) / f, depth * (top_pixel_bound - cy)/ f, depth);
    Vec3 bottom_left_corner = Vec3(depth * (left_pixel_bound - cx) / f, depth * (bottom_pixel_bound - cy)/f, depth);
    Vec3 bottom_right_corner = Vec3(depth * (right_pixel_bound - cx) / f, depth * (bottom_pixel_bound - cy)/f, depth);

    logfile << ToCSV(top_left_corner); 
    logfile << ToCSV(top_right_corner); 
    logfile << ToCSV(bottom_left_corner); 
    logfile << ToCSV(bottom_right_corner); 

    logfile << "\n";
  }
  logfile.close();
}

std::string ToCSV(const Vec3 v) {
  std::stringstream ss;
  ss << v.x << " " << v.y << " " << v.z << " ";
  return ss.str();
}

void GetFeaturePoints(std::vector<Vec3>& features, const std::string& filename){
  //File in
  std::ifstream myFile(filename);
  //Open an existing file
  if(!myFile.is_open()) throw std::runtime_error("Could not open file");
  double val;
  std::string line;

  // Read data, line by line
  while(std::getline(myFile, line))
  {
    // Create a stringstream of the current line
    std::stringstream ss(line);
    // Keep track of the current column index
    Vec3 feature_point;
    // Extract each feature
    ss >> feature_point.x >> feature_point.y >> feature_point.z;
    features.push_back(feature_point);
  }
  myFile.close();
}