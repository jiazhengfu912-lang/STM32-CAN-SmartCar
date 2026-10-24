// 只读离线重放已保存的扫描和轮式里程计；无订阅、TF、控制或导航目标发布。
#include <fstream>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <pluginlib/class_loader.hpp>
#include <slam_toolbox/slam_mapper.hpp>

class Listener : public karto::MapperLoopClosureListener {
 public:
  std::ofstream log;
  double stamp=0.;
  explicit Listener(const std::string &path):log(path) {log<<"sim_s,event,message\n";}
  // 保留粗/细匹配的原始响应与方差，CSV引号包含内部换行。
  void LoopClosureCheck(const std::string &s) override {
    log<<stamp<<",CHECK,\"";
    for(char c:s) {if(c=='\"')log<<'\"';log<<c;}
    log<<"\""<<std::endl;
  }
  // 记录实际闭环触发点，区分局部关联与全局优化。
  void BeginLoopClosure(const std::string &s) override {log<<stamp<<",BEGIN,"<<s<<std::endl;}
  void EndLoopClosure(const std::string &s) override {log<<stamp<<",END,"<<s<<std::endl;}
};

int main(int argc,char**argv) {
  if(argc<3)return 2;
  const std::filesystem::path out(argv[2]);
  if(std::filesystem::exists(out))return 3;
  std::filesystem::create_directories(out);
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("slam_toolbox");
  node->declare_parameter("debug_logging",false);
  karto::Dataset source;
  source.LoadFromFile(argv[1]);
  for(auto object:source.GetLasers()) {
    auto sensor=dynamic_cast<karto::Sensor*>(object);
    if(sensor)karto::SensorManager::GetInstance()->RegisterSensor(sensor,true);
  }
  pluginlib::ClassLoader<karto::ScanSolver> loader("slam_toolbox","karto::ScanSolver");
  auto solver=loader.createSharedInstance("solver_plugins::CeresSolver");
  solver->Configure(node);
  mapper_utils::SMapper smapper;
  smapper.configure(node);
  auto mapper=smapper.getMapper();
  mapper->SetScanSolver(solver.get());
  Listener listener((out/"loops.csv").string());mapper->AddListener(&listener);
  std::ofstream stream(out/"online_pose.csv");stream<<std::setprecision(15);
  stream<<"scan_id,sim_s,odom_x,odom_y,odom_yaw,slam_x,slam_y,slam_yaw,processed\n";
  // 每个原始扫描都重放一次，不使用保存时已经优化的CorrectedPose作输入。
  for(const auto &entry:source.GetData()) {
    auto scan=dynamic_cast<karto::LocalizedRangeScan*>(entry.second);
    if(!scan)continue;
    auto current=new karto::LocalizedRangeScan(scan->GetSensorName(),scan->GetRangeReadingsVector());
    current->SetTime(scan->GetTime());current->SetOdometricPose(scan->GetOdometricPose());
    current->SetCorrectedPose(scan->GetOdometricPose());listener.stamp=scan->GetTime();
    const bool accepted=mapper->Process(current);
    const auto odom=current->GetOdometricPose(),pose=current->GetCorrectedPose();
    stream<<entry.first<<','<<scan->GetTime()<<','<<odom.GetX()<<','<<odom.GetY()<<','<<odom.GetHeading()<<','
          <<pose.GetX()<<','<<pose.GetY()<<','<<pose.GetHeading()<<','<<accepted<<'\n';
    if(entry.first%100==0) {stream.flush();std::cout<<"scan "<<entry.first<<" time "<<scan->GetTime()<<std::endl;}
    if(!accepted)delete current;
  }
  // 原样导出最终占据栅格，供既有独立几何评价器读取。
  auto grid=smapper.getOccupancyGrid(.05);
  auto origin=grid->GetCoordinateConverter()->GetOffset();
  std::ofstream cells(out/"grid.csv");cells<<std::setprecision(15);
  cells<<grid->GetWidth()<<','<<grid->GetHeight()<<','<<origin.GetX()<<','<<origin.GetY()<<",0.05\n";
  for(int y=0;y<grid->GetHeight();++y) {
    for(int x=0;x<grid->GetWidth();++x) {
      if(x)cells<<',';
      cells<<int(*grid->GetDataPointer(karto::Vector2<kt_int32s>(x,y)));
    }
    cells<<'\n';
  }
  delete grid;mapper->RemoveListener(&listener);
  rclcpp::shutdown();return 0;
}
