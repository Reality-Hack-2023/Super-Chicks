using Vector3 = UnityEngine.Vector3;

namespace MITHack.Robot.Utils
{
    public static class VectorUtility
    {

        /// <summary>
        /// Gets the up direction.
        /// </summary>
        /// <param name="inForwardDirection">The forward direction.</param>
        /// <returns>The up direction.</returns>
        public static Vector3 GetUpDirection(Vector3 inForwardDirection)
        {
            inForwardDirection.Normalize();
            var right = Vector3.Cross(inForwardDirection, Vector3.up);
            if (right == Vector3.zero)
            {
                right = Vector3.Cross(inForwardDirection, Vector3.forward);
            }
            right.Normalize();
            var upCrossed = Vector3.Cross(right, inForwardDirection);
            upCrossed.Normalize();
            return upCrossed;
        }

        /// <summary>
        /// Gets the right direction from the forward.
        /// </summary>
        /// <param name="inForwardDirection">The forward direction.</param>
        /// <returns>The right direction.</returns>
        public static Vector3 GetRightDirection(Vector3 inForwardDirection)
        {
            var upDirection = GetUpDirection(inForwardDirection);
            var crossed = Vector3.Cross(inForwardDirection.normalized, upDirection);
            crossed.Normalize();
            return -crossed;
        }

        /// <summary>
        /// Calculate the up and right directions.
        /// </summary>
        /// <param name="forward">The forward direction.</param>
        /// <param name="up">The up direction.</param>
        /// <param name="right">The right direction.</param>
        public static void CalculateDirections(in Vector3 forward, out Vector3 up, out Vector3 right)
        {
            up = GetUpDirection(forward);
            var crossed = Vector3.Cross(forward, up);
            crossed.Normalize();
            right = -crossed;
        }
    }
}