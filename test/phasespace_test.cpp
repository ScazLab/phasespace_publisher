#include <gtest/gtest.h>
#include "phasespace_publisher/phasespace_publisher.h"

using namespace std;

TEST(PhasespaceTest, testPassPoints)
{
    PhasespacePublisher ps("ps_markers");

    ros::NodeHandle nh("phasespace_test");
    ros::Publisher pub = nh.advertise<phasespace_publisher::PhasespacePtArray>
                                    ("/ps_markers/phasespace_points", SUBSCRIBER_BUFFER);

    phasespace_publisher::PhasespacePtArray msg;
    phasespace_publisher::PhasespacePt point_;
    phasespace_publisher::PhasespacePt point_2;
    phasespace_publisher::PhasespacePt point_3;


    point_.id = 0;
    point_.pt.x = 1;
    point_.pt.y = 2;
    point_.pt.z = 3;

    point_2.id = 1;
    point_2.pt.x = 0.5;
    point_2.pt.y = 1.5;
    point_2.pt.z = 2.5;

    point_3.id = 2;
    point_3.pt.x = -0.5;
    point_3.pt.y = -1.5;
    point_3.pt.z = -2.5;


    msg.points.push_back(point_);
    msg.points.push_back(point_2);
    msg.points.push_back(point_3);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    std::vector<RVIZMarker> markers = ps.getMarkers();

    EXPECT_EQ(markers[0].points[0].x, 1.0);
    EXPECT_EQ(markers[0].points[0].y, 2.0);
    EXPECT_EQ(markers[0].points[0].z, 3.0);

    EXPECT_EQ(markers[0].points[1].x, 0.5);
    EXPECT_EQ(markers[0].points[1].y, 1.5);
    EXPECT_EQ(markers[0].points[1].z, 2.5);

    EXPECT_EQ(markers[0].points[2].x, -0.5);
    EXPECT_EQ(markers[0].points[2].y, -1.5);
    EXPECT_EQ(markers[0].points[2].z, -2.5);

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
    ros::init(argc, argv, "phasespace_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
