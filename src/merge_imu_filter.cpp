#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Imu.h>
#include <imu_3dm_gx4/FilterOutput.h>

using namespace sensor_msgs;
using namespace imu_3dm_gx4;
using namespace message_filters;

ros::Publisher pubMerge;

void callback(const ImuConstPtr& imu, const FilterOutputConstPtr& filter)
{
  Imu new_imu = *imu;
  new_imu.orientation = filter->orientation;
  pubMerge.publish(new_imu);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "merge_imu_filter");

  ros::NodeHandle nh;

  pubMerge = nh.advertise<sensor_msgs::Imu>("data", 1);

  message_filters::Subscriber<Imu> imu_sub(nh, "/imu/imu", 5);
  message_filters::Subscriber<FilterOutput> filter_sub(nh, "/imu/filter", 5);

	typedef sync_policies::ApproximateTime<Imu, FilterOutput> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), imu_sub, filter_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
