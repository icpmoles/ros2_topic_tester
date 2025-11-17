#include <memory>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <queue>
#include <cstdlib>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"



using std::placeholders::_1;

struct rolling_statistics{
  float max;
  float min;
  float average;
  float sample;
  int n_samples;
}; 

// uint GROUPS_NUMBER = 1;
// uint GROUPS_PERIOD = 600;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      get_params();


      switch (msg_type_)
      {
      case 0:
        subscription_ls_ =  this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "topic",
                    rclcpp::QoS(rclcpp::SensorDataQoS()),
                    [this](const sensor_msgs::msg::LaserScan& msg)
                    {
                        MinimalSubscriber::laserscan_callback(msg);
                    }); 
        break;
      case 1:
        subscription_pc_ =  this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    "topic",
                    rclcpp::QoS(rclcpp::SensorDataQoS()),
                    [this](const sensor_msgs::msg::PointCloud2& msg)
                    {
                        MinimalSubscriber::pointcloud_callback(msg);
                    }); 
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "msg_type incorrect (how?): use only 0 or 1");
        rclcpp::shutdown();
        break;
      }

    }

	~MinimalSubscriber(){
        log_file_.close();
        RCLCPP_INFO(this->get_logger(), "Destructor Called");
    }


  private:

    void get_params(){
        this->declare_parameter("file_output_prefix", "output/latency");
        log_prefix_ = this->get_parameter("file_output_prefix").as_string();

        this->declare_parameter("rolling_window_size", 10);
        max_queue_size_ = (size_t) this->get_parameter("rolling_window_size").as_int();;


		this->declare_parameter("long_mode", false);
        long_mode_ = (size_t) this->get_parameter("long_mode").as_bool();;

		this->declare_parameter("long_mode_total_groups", 1);
        long_mode_total_groups_ = this->get_parameter("long_mode_total_groups").as_int();;
        
        if(!long_mode_){
            // overwrite: the single record is just a long mode with 1 group
            long_mode_total_groups_ = 1;
        }

		this->declare_parameter("long_mode_period", 10);
        long_mode_total_groups_ =  this->get_parameter("long_mode_period").as_int();;
        

        this->declare_parameter("msg_type", 0);
        msg_type_ = this->get_parameter("msg_type").as_int();
        switch (msg_type_)
        {
        case 0:
          RCLCPP_INFO(this->get_logger(), "Using laserscan sink");
          break;
        case 1:
          RCLCPP_INFO(this->get_logger(), "Using pointcloud sink");
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "msg_type incorrect: use only 0 or 1");
          rclcpp::shutdown();
          break;
        }
        parameters_initialized_=true;
    
    }

    void write_metadata(std::string csv_path, double timestamp){
        std::string metadata_path = csv_path + ".metadata.txt";
        std::string container_name; //= std::string(std::getenv("CONTAINER_NAME"));
        char const* tmp = std::getenv("CONTAINER_NAME");
        if ( tmp == NULL ) {
            container_name = "generic";
        } else {
            std::string container_name(tmp);
        }


        std::ofstream  metadata_file;
        metadata_file.open(metadata_path,std::ios::trunc);
        metadata_file << "Environment: " << container_name << std::endl;
        metadata_file << "Start timestamp: "  <<  std::fixed << std::setprecision(0) << timestamp << std::endl;
        std::time_t tt = std::chrono::system_clock::system_clock::to_time_t (std::chrono::system_clock::system_clock::now());

        struct std::tm * ptm = std::localtime(&tt);
        // std::cout << "Now (local time): " << std::put_time(ptm,"%c") << '\n';
        metadata_file << "Start time: "  << std::put_time(ptm,"%c") << std::endl;


        metadata_file << "Rolling window size: " << max_queue_size_ << std::endl;

        metadata_file.close();
    }

    void write_statistics(double timestamp, rolling_statistics stats, size_t payload_size ){
        if(log_file_to_initialize){
            if(group_counter_>1){
                csv_filename = log_prefix_ + "_" + std::to_string(group_counter_) + ".csv";
            } else {
                csv_filename = log_prefix_  + ".csv";
            }
            write_metadata(csv_filename, timestamp);
            log_file_.open(csv_filename,std::ios::trunc);
            log_file_ << "t, delay [ns],average delay [ns], max delay [ns] ,min delay [ns], msg size [byte]" << std::endl;
            log_file_to_initialize = false;
        }
      

        // log_file_ << "t, average delay [ns], max delay [ns] ,min delay [ns]";
      log_file_ <<  std::fixed << std::setprecision(3) << timestamp << "," << std::setprecision(0)  << stats.sample << "," << stats.average << "," << stats.max << "," << stats.min << "," << payload_size << std::endl  ;
    }

    void pointcloud_callback (const sensor_msgs::msg::PointCloud2 &msg)
    {
      // if (is_first_msg_){
      //   is_first_msg_ = false;
      //   max_queue_size_ = std::min((int)(rolling_window_/ std::stof(msg.header.frame_id))+1 , 50);
      // }
      
      rclcpp::Time reception_time = this->now();
      rclcpp::Duration diff = reception_time - rclcpp::Time(msg.header.stamp) ;
      float delta_ns = diff.nanoseconds();
      float delta_ms = delta_ns*1e-6; 
      size_t msg_size = 4*msg.data.size() + sizeof(msg);
      // ranges and intensities are float32 vectors + struct containing all the msg metadata 
      // nb. i forgot to include the size of the frame_id. it's just a rough estimate


      rolling_statistics stats = get_new_statistics(delta_ns);
      // log_file_ << "t, average delay [ns], max delay [ns] ,min delay [ns]";
      double epoch_with_ms = reception_time.nanoseconds()*1e-9;
      write_statistics(epoch_with_ms, stats, msg_size);
      // msg->header.frame_id.c_str()
      RCLCPP_INFO(this->get_logger(), "Delta: %f ms", delta_ms);
    }
       
    void laserscan_callback (const sensor_msgs::msg::LaserScan &msg)
    {
      // if (is_first_msg_){
      //   is_first_msg_ = false;
      //   max_queue_size_ = std::min((int)(rolling_window_/ msg.scan_time)+1 , 50);
      // }
      
      rclcpp::Time reception_time = this->now();
      rclcpp::Duration diff = reception_time - rclcpp::Time(msg.header.stamp) ;
      float delta_ns = diff.nanoseconds();
      float delta_ms = delta_ns*1e-6; 
      size_t msg_size = 4*msg.intensities.size() + 4*msg.ranges.size() + sizeof(msg);
      // ranges and intensities are float32 vectors + struct containing all the msg metadata 
      // nb. i forgot to include the size of the frame_id. it's just a rough estimate

      rolling_statistics stats = get_new_statistics(delta_ns);
      // log_file_ << "t, average delay [ns], max delay [ns] ,min delay [ns]";
      double epoch_with_ms = reception_time.nanoseconds()*1e-9;
      write_statistics(epoch_with_ms, stats, msg_size);
      // msg->header.frame_id.c_str()
      RCLCPP_INFO(this->get_logger(), "Delta: %f ms", delta_ms);
    }
 
    rolling_statistics get_new_statistics(float new_delay){
        
        float sum = 0.0;
        size_t n = delays_.size();
        if ( n == max_queue_size_){
          delays_.pop_front();
        } 
        delays_.push_back(new_delay);
        assert(delays_.size() <= max_queue_size_);
        n = delays_.size();

        float max = delays_.at(0);
        float min = delays_.at(0);
        
        // to avoid floating point errors
        for (auto it = delays_.begin(); it != delays_.end(); ++it) {
          if (*it > max){
            max = *it;
          }
          if (*it < min){
            min = *it;
          }

          sum += *it;
          

        }

        rolling_statistics stats {max, min, sum/n , new_delay, (int) n} ;

        return stats;
      } 

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_ls_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_pc_;
    std::ofstream log_file_;
    std::deque<float> delays_;
    size_t max_queue_size_;
    bool is_first_msg_ = true;
    std::string log_prefix_;
    int msg_type_;
    bool log_file_to_initialize = true;
    uint msg_counter_=1;  
    bool parameters_initialized_=false;

    bool long_mode_;
    uint long_mode_total_groups_=1;
   // uint long_mode_groups_period_;
    uint group_counter_=1;
    std::string csv_filename = "";

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}