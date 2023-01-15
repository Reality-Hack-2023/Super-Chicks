using System;
using UnityEngine;

namespace MITHack.Robot.Utils.Tests
{
    public class VectorTestComponent : MonoBehaviour
    {
        [Header("References")]
        [SerializeField]
        private Transform referenceTarget;


        private void OnDrawGizmos()
        {
            if (referenceTarget)
            {
                var position = transform.position;
                var targetDirection = referenceTarget.position - position;
                targetDirection.Normalize();
                
                Gizmos.color = Color.red;
                Gizmos.DrawRay(position, targetDirection);

                Gizmos.color = Color.blue;
                var calculatedUp = VectorUtility.GetUpDirection(targetDirection);
                Gizmos.DrawRay(position, calculatedUp);

                var calculatedRight = VectorUtility.GetRightDirection(targetDirection);
                Gizmos.color = Color.yellow;
                Gizmos.DrawRay(position, calculatedRight);
            }
        }
    }
}