/*
based off of code samples from:
https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/publisher.md
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes;
using twistMsg = RosMessageTypes.Geometry.TwistMsg;
using vector = RosMessageTypes.Geometry.Vector3Msg;

namespace MITHack.Robot.Utils
{
    public static class one_dof_twist_publisher
    {
        private static ROSConnection ros;
        public static string topicName = "/move";

        [RuntimeInitializeOnLoadMethod]
        private static void Initialize()
        {
            // start the ROS connection
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<twistMsg>(topicName);
        }

        public static void publish_control(float linear_x, float yaw)
        {
            vector linear = new vector(
                linear_x,
                0.0,
                0.0
            );

            vector angular = new vector(
                0.0,
                0.0,
                yaw
            );

            twistMsg twist = new twistMsg(
                linear,
                angular
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, twist);
        }
    }

}