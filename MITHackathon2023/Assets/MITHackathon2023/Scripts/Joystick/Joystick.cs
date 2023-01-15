using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MITHack.Robot.Utils;
using MITHack.Robot.Entities;

namespace MITHack.Robot
{
    public class Joystick : MonoBehaviour
    {
        private float HorizontalAxis;
        private float VerticalAxis;
        private float HorizontalAxisRaw;
        private float VerticalAxisRaw;

        public float HorizontalAxisThreshold;
        public float VerticalAxisThreshold;
        
        public float DeadZone = 0.1f;
        public float RatePerFixedUpdate;
        public float OGRatePerFixedUpdate = 25f;

        private RobotMovement _robotMovement;

        private RobotMovement RobotMovement
        {
            get
            {
                if (_robotMovement)
                {
                    return _robotMovement;
                }
                var robot = RobotEntity.Get();
                if (robot)
                {
                    _robotMovement = robot.RobotMovement;
                }
                return _robotMovement;
            }
        }
        

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
            Debug.Log("X:" + HorizontalAxis + ", Z:" + VerticalAxis);

            if (HorizontalAxis < DeadZone && HorizontalAxis > -DeadZone)
            {
                HorizontalAxis = 0;
            }
            if (VerticalAxis < DeadZone && VerticalAxis > -DeadZone)
            {
                VerticalAxis = 0;
            }
            
            if (RobotMovement)
            {
                RobotMovement.SetDirection(new Vector2(-HorizontalAxis, VerticalAxis));
            }
        }
        
        private void FixedUpdate()
        {
            if (RatePerFixedUpdate > 0)
            {
                one_dof_twist_publisher.publish_control(VerticalAxisRaw, HorizontalAxisRaw);
                RatePerFixedUpdate = OGRatePerFixedUpdate;
            }
            RatePerFixedUpdate -= 1;
        }
    }
}
