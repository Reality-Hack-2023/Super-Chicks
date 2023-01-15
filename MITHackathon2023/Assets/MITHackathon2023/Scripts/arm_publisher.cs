/*
based off of code samples from:
https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/publisher.md
*/

using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes;
using vector = RosMessageTypes.Geometry.Vector3Msg;

namespace MITHack.Robot.Utils
{
    public static class arm_publisher
    {
        private static ROSConnection ros;
        public static string topicName = "/arm";

        [RuntimeInitializeOnLoadMethod]
        private static void Initialize()
        {
            // start the ROS connection
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<vector>(topicName);
        }

        public static void arm_publish(float x, float z)
        {
            vector linear = new vector(
                x,
                z,
                0.0
            );


            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, linear);
        }
    }

}