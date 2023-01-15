using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MITHack.Robot.Utils;

namespace MITHack.Robot
{
    public class joystick_arm : MonoBehaviour
    {
        public float HorizontalAxis;
        public float VerticalAxis;
        public float HorizontalAxisRaw;
        public float VerticalAxisRaw;

        public float HorizontalAxisThreshold;
        public float VerticalAxisThreshold;
        
        public float DeadZone = 0.1f;
        public float RatePerFixedUpdate;
        public float OGRatePerFixedUpdate = 25f;

        // Start is called before the first frame update
        void Start()
        {
            HorizontalAxisRaw = transform.rotation.eulerAngles.x;
            VerticalAxisRaw = transform.rotation.eulerAngles.z;
            RatePerFixedUpdate = OGRatePerFixedUpdate;
        }

        // Update is called once per frame
        void Update()
        {
            HorizontalAxisRaw = transform.localRotation.eulerAngles.z;
            VerticalAxisRaw = transform.localRotation.eulerAngles.x;
            
            if(HorizontalAxisRaw > 180)
            {
                HorizontalAxisRaw -= 360;
            }
            if (VerticalAxisRaw > 180)
            {
                VerticalAxisRaw -= 360;
            }
            HorizontalAxis = HorizontalAxisRaw / 45;
            VerticalAxis = VerticalAxisRaw / 45;
            Debug.Log("X:" + HorizontalAxisRaw + ", Z:" + VerticalAxisRaw);
        }
        private void FixedUpdate()
        {
            if (RatePerFixedUpdate > 0)
            {
                arm_publisher.arm_publish(VerticalAxisRaw, HorizontalAxisRaw);
                RatePerFixedUpdate = OGRatePerFixedUpdate;
            }
            RatePerFixedUpdate -= 1;
        }
        

    }
}
