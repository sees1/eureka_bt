#include "eureka_bt/cv.hpp"

CV_detection::CV_detection(const std::string& name,
                           const BT::NodeConfiguration& config,
                           rclcpp::Node::SharedPtr node)
: BT::StatefulActionNode(name, config),
  node_(node),
  narrow("No_detection"),
  length(0.0),
  angle(0.0),
  coef(0.0)
{
  if (node_->has_parameter("full_info"))
    full_info_ = node_->get_parameter("full_info").as_bool();
  else
    full_info_ = node_->declare_parameter("full_info", true);

  if (node_->has_parameter("dry_run"))
    dry_run_ = node_->get_parameter("dry_run").as_bool();
  else
    dry_run_ = node_->declare_parameter("dry_run", false);

  subscription = node_->create_subscription<sensor_msgs::msg::JointState>("/arrow_detection", 10,
    [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lc(mut_);

      // if (full_info_)
      //   RCLCPP_INFO(node_->get_logger(), "(cv) detection main compute loop called!");

      if (!msg->name.empty() && !dry_run_ && names_.size() != 5)
      {
        double max = 0;
        size_t max_idx = 0;
        for(size_t idx = 0; idx < msg->effort.size(); ++idx)
        {
          if (msg->effort[idx] > max)
          {
            max = msg->effort[idx];
            max_idx = idx;
          }
        }

        names_.emplace_back(msg->name[max_idx]);
        positions_.emplace_back(msg->position[max_idx]);
        velocities_.emplace_back(msg->velocity[max_idx]);
        efforts_.emplace_back(msg->effort[max_idx]);
      }
    }
  );
}

BT::PortsList CV_detection::providedPorts() {
  return {
    BT::OutputPort<std::string>("narrow_arrow"),
    BT::OutputPort<double>("length"),
    BT::OutputPort<double>("angle"),
    BT::OutputPort<double>("coef")
  };
}

BT::NodeStatus CV_detection::onStart()
{
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CV_detection::onRunning()
{
  std::lock_guard<std::mutex> lc(mut_);
  if (names_.size() != 5)
  {
    RCLCPP_INFO(node_->get_logger(), "(cv) Continue collect data!");
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    processValues();
    setOutput("narrow_arrow", narrow);
    setOutput("length", length);
    setOutput("angle", angle);
    setOutput("coef", coef);
    clearData();


    return BT::NodeStatus::SUCCESS;
  }
}

void CV_detection::onHalted()
{
  RCLCPP_ERROR(node_->get_logger(), "(cv) current node is halted!");
}

void CV_detection::processValues()
{
  bool has_no_detection = std::any_of(names_.begin(), names_.end(),
    [](const std::string& name)
    {
      return name == "No_detection";
    }
  );

  narrow = has_no_detection ? "No_detection" : names_.back();

  length = calculateAverage(positions_);
  angle = calculateAverage(velocities_);
  coef = calculateAverage(efforts_);

  if (full_info_)
    RCLCPP_INFO(node_->get_logger(), "(cv) length = %f, angle = %f, coef = %f, after process values!", length, angle, coef);
}

double CV_detection::calculateAverage(const std::vector<double>& values)
{
  double sum = 0.0;

  for (const auto& value : values)
    sum += value;
  
  return sum / values.size();  
}

void CV_detection::clearData() {
  narrow = "No_detection";
  length = 0.0;
  angle = 0.0;
  coef = 0.0;
  names_.clear();
  positions_.clear();
  velocities_.clear();
  efforts_.clear();
}