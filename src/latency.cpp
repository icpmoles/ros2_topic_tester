#include <memory>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <queue>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"


using std::placeholders::_1;

struct rolling_statistics{
  float max;
  float min;
  float average;
  int n_samples;
}; 

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      MinimalSubscriber::is_first_msg_ = true;
      myfile_.open ("latency.csv",std::ios::trunc);
      myfile_ << "t, average delay [ns], max delay [ns] ,min delay [ns]" << std::endl;

       subscription_ =  this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "laserscan",
                    rclcpp::QoS(rclcpp::SensorDataQoS()),
                    [this](const sensor_msgs::msg::LaserScan& msg)
                    {
                        MinimalSubscriber::laserscan_callback(msg);
                    }); 

      /* subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>("laserscan", 
        rclcpp::QoS(rclcpp::SensorDataQoS()), 
        std::bind(&MinimalSubscriber::topic_callback, this, _1)
      ); */
    }

  private:
   /*  void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
         
         rclcpp::Time reception_time = this->now();
         rclcpp::Duration diff = reception_time - rclcpp::Time(msg->header.stamp) ;
         float delta_ms = diff.nanoseconds()*1e-6;      
         // msg->header.frame_id.c_str()
         RCLCPP_INFO(this->get_logger(), "Delta: %f ms", delta_ms );
      }   */
       
       
    void laserscan_callback (const sensor_msgs::msg::LaserScan &msg)
    {
      if (is_first_msg_){
        is_first_msg_ = false;
        max_queue_size_ = std::min((int)(rolling_window_/ msg.scan_time)+1 , 50);
      }
      
      rclcpp::Time reception_time = this->now();
      rclcpp::Duration diff = reception_time - rclcpp::Time(msg.header.stamp) ;
      float delta_ns = diff.nanoseconds();
      float delta_ms = delta_ns*1e-6; 

      rolling_statistics stats = get_new_statistics(delta_ns);
      // myfile_ << "t, average delay [ns], max delay [ns] ,min delay [ns]";
      double epoch_with_ms = reception_time.nanoseconds()*1e-9;
      myfile_ <<  std::fixed << std::setprecision(3) << epoch_with_ms << "," << std::setprecision(0)  << stats.average << "," << stats.max << "," << stats.min << std::endl ;

      // msg->header.frame_id.c_str()
      RCLCPP_INFO(this->get_logger(), "Delta: %f ms, scan time %f", delta_ms , msg.scan_time);


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

        rolling_statistics stats {max, min, sum/n , (int) n} ;

        return stats;
      } 

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::ofstream myfile_;
    std::deque<float> delays_;
    size_t max_queue_size_;
    float rolling_window_ = 1.0;
    bool is_first_msg_;
    // float rolling_average_sum_ = 0.0;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}