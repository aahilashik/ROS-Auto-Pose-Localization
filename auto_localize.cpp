#include "ros/ros.h"
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <sys/stat.h>


class AmrAutoLocalization{
    public :
        double* subPose = new double[4];
        double* pubPose = new double[4];
        std::string folderPath, filePath;
        
        ros::Publisher  nodePublisher;
        ros::Subscriber nodeSubscriber;

    AmrAutoLocalization(std::string nodeName, std::string moduleName, int argc, char **argv){
        
        ros::init(argc, argv, nodeName);
        ros::NodeHandle nh;
        nodePublisher   = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
        nodeSubscriber  = nh.subscribe("/robot_pose", 1000, &AmrAutoLocalization::poseCallback, this);
        
        folderPath  = ros::package::getPath(moduleName);
        filePath    = folderPath + "/cache/pose.txt";
        createCacheDir(folderPath + "/cache");

        if (isPathExists(filePath)){
            ROS_INFO("Saved robot position file exists!!");
            initialPosePublish();
        } else 
            ROS_INFO("Saved robot position file not exists!!");
    }

    void poseCallback(const geometry_msgs::Pose::ConstPtr& data){
        subPose[0] = data->position.x; 
        subPose[1] = data->position.y; 
        subPose[2] = data->orientation.z; 
        subPose[3] = data->orientation.w; 
        savePoseToFile(subPose);
        ROS_INFO("Robot position saved to local file successfully!!");
    }

    void initialPosePublish(){
        pubPose = loadPoseFromFile();
        geometry_msgs::PoseWithCovarianceStamped pose;
        ros::Rate loopRate(1);

        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.pose.position.x = pubPose[0];
        pose.pose.pose.position.y = pubPose[1];
        pose.pose.pose.orientation.z = pubPose[2];
        pose.pose.pose.orientation.w = pubPose[3];

        do {
            nodePublisher.publish(pose);
            ros::spinOnce();
            loopRate.sleep();
        } while(!((abs(subPose[0]-pubPose[0])<=0.002 && abs(subPose[1]-pubPose[1])<=0.002)) && ros::ok());
        ROS_INFO("Initial pose published successfully!!");
    }

    bool isPathExists(std::string path)
    {
        struct stat buffer;
        return (stat (path.c_str(), &buffer) == 0);
    }

    bool createCacheDir(std::string dirPath)
    {
        if (isPathExists(dirPath))
            ROS_INFO("Cache Directory already exists!!");
        else {
            if (mkdir(dirPath.c_str(), 0777) == -1)
                ROS_ERROR("Cache Directory not created!!");
            else
                ROS_INFO("Cache Directory created!!");
        }
    }

    void savePoseToFile(double pose[4]){
        std::ostringstream dataStream;
        dataStream << pose[0] << " " << pose[1] << " " << pose[2] << " " << pose[3];
        std::string data = dataStream.str();
        std::ofstream outFile(filePath);
        outFile << data;
        outFile.close();
    }

    double* loadPoseFromFile(){
        std::ifstream inFile; 
        std::stringstream data;
        double* values = new double[4];
        inFile.open(filePath);
        data << inFile.rdbuf();
        std::istringstream dataStream(data.str());
        for(int i=0; i<4; i++)
            dataStream >> values[i];
        return values;
    }

    void spin(){
        ros::spin();
    }
};

int main(int argc, char **argv) {
    AmrAutoLocalization amr("amr_node", "amr_robot_pose", argc, argv);
    amr.spin();
    return 0;
}
