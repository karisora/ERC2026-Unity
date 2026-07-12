using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;

namespace ROS2
{
    [RequireComponent(typeof(ROS2UnityComponent))]
    public class GPS : MonoBehaviour
    {
        private ROS2UnityComponent ros2Unity;
        private ROS2Node ros2Node;
        private IPublisher<sensor_msgs.msg.NavSatFix> nav_sat_fix_pub;
        public string frameID = "gps_link";
        public string topicName = "/gps/fix";

        [Header("GPS Publish Settings")]
        // Publish interval: 20Hz
        public float TopicPublishHz = 20f;
        private float publishTimer = 0f;

        [Header("Initial GPS Coordinates")]
        public double baseLatitude = 35.0;   // Initial latitude coordinate
        public double baseLongitude = 135.0; // Initial longitude coordinate

        [Header("Base Position in Unity Coordinates")]
        public Vector3 basePosition = Vector3.zero;

        [Header("Current GPS State")]
        public double latitude;
        public double longitude;
        public double altitude;

        // Earth radius (m)
        const double EarthRadius = 6378137.0;

        // Current GPS coordinates
        public (double latitude, double longitude) GetCurrentGPS()
        {
            // Offset in Unity coordinates (meter conversion)
            Vector3 offset = transform.position - basePosition;
            double dNorth = -offset.x; // Unity x-axis is North, inverted
            double dEast = offset.z;  // Unity z-axis is East

            // Calculate the change in latitude and longitude
            double dLat = dNorth / EarthRadius * (180.0 / Mathf.PI);
            double dLon = dEast / (EarthRadius * Mathf.Cos((float)(baseLatitude * Mathf.Deg2Rad))) * (180.0 / Mathf.PI);

            latitude = baseLatitude + dLat;
            longitude = baseLongitude + dLon;

            return (latitude, longitude);
        }

        // Start is called before the first frame update
        void Start()
        {
            ros2Unity = GetComponent<ROS2UnityComponent>();
            if (ros2Unity == null)
            {
                Debug.LogError("ROS2UnityComponent not found");
                return;
            }
            ros2Node = ros2Unity.CreateNode("gps_node");
            nav_sat_fix_pub = ros2Node.CreatePublisher<sensor_msgs.msg.NavSatFix>(topicName);
        }

        // Update is called once per frame
        void Update()
        {
            if (nav_sat_fix_pub == null)
            {
                return;
            }

            publishTimer += Time.deltaTime;
            if (publishTimer >= 1 / TopicPublishHz)
            {
                var (latitude, longitude) = GetCurrentGPS();
                var msg = new sensor_msgs.msg.NavSatFix();

                // Header
                msg.Header = new std_msgs.msg.Header();
                msg.Header.Frame_id = frameID;
                ros2Node.clock.UpdateROSClockTime(msg.Header.Stamp);

                // GPS values
                msg.Latitude = latitude;
                msg.Longitude = longitude;
                msg.Altitude = transform.position.y; // Unity Y-axis is altitude

                // Covariance and Status can be set as needed
                msg.Status.Status = 0; // STATUS_FIX
                msg.Status.Service = 1; // SERVICE_GPS

                nav_sat_fix_pub.Publish(msg);
                publishTimer = 0f;
            }
        }
    }
}
