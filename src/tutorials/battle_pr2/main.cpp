#include <ros/ros.h>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <string>

using namespace std;

// Publisher for marker information
ros::Publisher visPub;

// Game map
int mapWidth = 21;
int mapHeight = 20;
int gameMap[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,            
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,            
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  
				 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
				 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,            
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,            
				 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  
				 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};      
             
void addCube(double x, double y, double z, double r, double g, double b, 
	double sx, double sy, double sz, double length, string ns, int id)
{
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time();
	marker.ns = ns;
	marker.id = id;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;
	
	// Position
	marker.pose.position.x = x;
	marker.pose.position.y = y;
	marker.pose.position.z = z;
	
	// Orientation
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	// Scale
	marker.scale.x = sx;
	marker.scale.y = sy;
	marker.scale.z = sz;
	
	// Cube colour
	marker.color.a = 1.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	
	visPub.publish( marker );
}

void drawMap(int * map_, int mapWidth_, int mapHeight_)
{
	int i, j, position, white;
	
	for (i = 0; i < mapHeight_; i++)
	{
		for (j = 0; j < mapWidth_; j++)
		{
			position = i * mapWidth_ + j;
			
			if (map_[position])
			{
				addCube(j - mapWidth_/2.0, i - mapWidth_/2.0, 0, 
					0, 0, 1,
					1, 1, 2,
					1, 
					"Battle_PR2_map",
					position);
			}
			else
			{
				white = position % 2;
				addCube(j - mapWidth_/2.0, i - mapWidth_/2.0, 2, 
					white, white, white,
					1, 1, 2,
					1, 
					"Battle_PR2_map",
					position);
				addCube(j - mapWidth_/2.0, i - mapWidth_/2.0, -2, 
					white, white, white,
					1, 1, 2,
					1, 
					"Battle_PR2_map",
					position + mapWidth_ * mapHeight_ + 1);
			}
		}
	}
}

int main(int argc, char ** argv)
{
	int refreshRoom = 50;
	double playerX, playerY, playerZ;
	
	ros::init(argc, argv, "Battle_Pr2");
	
	// Main node handle
	ros::NodeHandle node;

	// Transform broadcaster for player frame
	tf::TransformBroadcaster tfBroadcaster;

	ros::Rate rate(25.0);
	
	visPub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
	tf::Transform playerTransform;
	
	while (node.ok())
	{
		if (!refreshRoom--)
		{
			drawMap((int *)gameMap, mapWidth, mapHeight);
			refreshRoom = 50;
		}
		
		
		
		playerTransform.setOrigin(tf::Vector3(playerX, playerY, playerZ));
		playerTransform.setRotation(tf::Quaternion(0, -M_PI/2, 0));
		tfBroadcaster.sendTransform(tf::StampedTransform(playerTransform, 
			ros::Time::now(), "/player", "/world"));
		
		ros::spinOnce();
	}
}
