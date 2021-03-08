#include "TrainerNode.h"


void TrainerNode::isGBARunningCallback(const std_msgs::Bool::ConstPtr& isGBARunningMsg)
{
  is_GBA_running_previous = is_GBA_running;
  is_GBA_running = isGBARunningMsg->data;
}

void TrainerNode::update()
{
  if (!training_active)
  {
    ROS_WARN_THROTTLE(5, "Training finalized!");
    return;
  }

  if (!is_GBA_running)
  {
    if (!playing_reverse_video) // iterate forwards
    {
      color_image_fwd_iterator++;
      depth_image_fwd_iterator++;
      if (color_image_fwd_iterator == color_image_iters.end() ||
          depth_image_fwd_iterator == depth_image_iters.end())
      {
        printf("Iteration forwards %d done\n", loop_number);
        // loop back to beginning
        color_image_fwd_iterator = color_image_iters.begin();
        depth_image_fwd_iterator = depth_image_iters.begin();
        loop_number++;
        if (loop_number > training_loops)
        {
          // Finish or go reverse
          if (do_reverse_video)
          {
            loop_number = 1;
            playing_reverse_video =  true;
          }
          else
          {
            training_active = false;
            orb_slam2_ros::SaveMap orb2_save_map_srv;
            orb2_save_map_srv.request.name = map_file_name;
            orb2_save_map_service.call(orb2_save_map_srv);
          }

          return;
        }
      }
      color_image_msg = (*(*color_image_fwd_iterator)).instantiate<sensor_msgs::Image>();
      depth_image_msg = (*(*depth_image_fwd_iterator)).instantiate<sensor_msgs::Image>();
    }
    else // iterate backwards
    {
      color_image_bwd_iterator++;
      depth_image_bwd_iterator++;
      if (color_image_bwd_iterator == color_image_iters.rend() ||
          depth_image_bwd_iterator == depth_image_iters.rend())
      {
        printf("Iteration backwards %d done\n", loop_number);
        // loop back to beginning (reversed)
        color_image_bwd_iterator = color_image_iters.rbegin();
        depth_image_bwd_iterator = depth_image_iters.rbegin();
        loop_number++;
        if (loop_number > training_loops)
        {
          // Finish
          training_active = false;
          orb_slam2_ros::SaveMap orb2_save_map_srv;
          orb2_save_map_srv.request.name = map_file_name;
          orb2_save_map_service.call(orb2_save_map_srv);
          return;
        }
      }
      color_image_msg = (*(*color_image_bwd_iterator)).instantiate<sensor_msgs::Image>();
      depth_image_msg = (*(*depth_image_bwd_iterator)).instantiate<sensor_msgs::Image>();
    }
  }
  color_image_msg->header.stamp =  ros::Time::now();
  depth_image_msg->header.stamp =  ros::Time::now();
  color_image_pub.publish(color_image_msg);
  depth_image_pub.publish(depth_image_msg);
}

TrainerNode::TrainerNode()
{

  ros::NodeHandle node_private_handle("~");

  // Get params if specified in launch file or as params on command-line, set defaults
  node_private_handle.param<std::string>("video_rosbag_name", video_rosbag_name, "video_loop.bag");
  node_private_handle.param<std::string>("color_image_topic_name", color_image_topic_name, "camera/color/image_raw");
  node_private_handle.param<std::string>("depth_image_topic_name", depth_image_topic_name, "camera/depth/image_rect_raw");
  node_private_handle.param<std::string>("map_file_name", map_file_name, "orb_map_training.bin");

  node_private_handle.param<int>("training_loops", training_loops, 2);
  node_private_handle.param<bool>("do_reverse_video", do_reverse_video, false);

  // publishers & subscribers
  status_GBA_sub = node_private_handle.subscribe("status_gba", 1, &TrainerNode::isGBARunningCallback, this);
  color_image_pub = node_private_handle.advertise<sensor_msgs::Image>(color_image_topic_name, 1);
  depth_image_pub = node_private_handle.advertise<sensor_msgs::Image>(depth_image_topic_name, 1);

  // Service clients
  orb2_save_map_service = node_private_handle.serviceClient<orb_slam2_ros::SaveMap>("/orb_slam2_rgbd/save_map");

  // Open rosbag file
  printf("Reading bag file\n");
  input_bag.open(video_rosbag_name.c_str(), rosbag::bagmode::Read);
  printf("bag file read\n");

  // Setup View color image
  std::vector<std::string> topics_image;
  topics_image.push_back(color_image_topic_name);
  rosbag::View view_color_image(input_bag, rosbag::TopicQuery(topics_image));
  // Make a vector with instances of the View::iterator class pointing to each message
  // This is needed to be able to do reverse iteration as well
  for (rosbag::View::iterator iter = view_color_image.begin(); iter != view_color_image.end(); ++iter)
  {
      rosbag::View::iterator new_iter(iter);
      color_image_iters.push_back( new_iter );
  }
  color_image_fwd_iterator = color_image_iters.begin();
  color_image_bwd_iterator = color_image_iters.rbegin();

  // Setup View depth image
  std::vector<std::string> topics_depth;
  topics_depth.push_back(depth_image_topic_name);
  rosbag::View view_depth_image(input_bag, rosbag::TopicQuery(topics_depth));
  // Make a vector with instances of the View::iterator class pointing to each message
  // This is needed to be able to do reverse iteration as well
  for (rosbag::View::iterator iter = view_depth_image.begin(); iter != view_depth_image.end(); ++iter)
  {
      rosbag::View::iterator new_iter(iter);
      depth_image_iters.push_back( new_iter );
  }
  depth_image_fwd_iterator = depth_image_iters.begin();
  depth_image_bwd_iterator = depth_image_iters.rbegin();

  printf("Size color image sequence: %d images\n", color_image_iters.size());
  printf("Size depth image sequence: %d images\n", depth_image_iters.size());

  // iterate through the message pointers
  for (std::vector<rosbag::View::iterator>::iterator iter = color_image_iters.begin(); iter != color_image_iters.end(); ++iter)
  {
    sensor_msgs::Image::ConstPtr image_msg = (*(*iter)).instantiate<sensor_msgs::Image>();
  }
  for (std::vector<rosbag::View::iterator>::iterator iter = depth_image_iters.begin(); iter != depth_image_iters.end(); ++iter)
  {
    sensor_msgs::Image::ConstPtr image_msg = (*(*iter)).instantiate<sensor_msgs::Image>();
  }

}

TrainerNode::~TrainerNode(void)
{
  input_bag.close();
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "Trainer");
  ros::NodeHandle node_private_handle("~");

  double loop_rate;
  node_private_handle.param("loop_rate", loop_rate, 30.0);
  ros::Rate rate(loop_rate);

  TrainerNode trainer_node;
  ros::Time prev_time = ros::Time::now();
  while (ros::ok())
  {
    rate.sleep();  // Sleep at start so dt makes sense
    ros::spinOnce();  // Receive topics
    ros::Time now = ros::Time::now();
    ros::Duration dt = prev_time - now;
    if (dt.isZero())
    {
      ROS_WARN_THROTTLE(5, "dt=0.0, skipping loop(s)");
      continue;
    }

    // Update node status and publish active image frame
    trainer_node.update();

    // Check sample time
    ROS_WARN_COND(dt.toSec() > 1 / loop_rate,
                  "Node takes %fms., desired frame sample time is %fms.", dt.toSec()*1000, (1.0 / loop_rate)*1000);

    ros::spinOnce();  // Send topics
    prev_time = now;
  }

  return 0;
}
