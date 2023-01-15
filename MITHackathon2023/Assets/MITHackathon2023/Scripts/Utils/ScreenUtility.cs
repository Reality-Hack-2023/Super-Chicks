using UnityEngine;

namespace MITHack.Robot.Utils
{
    public static class ScreenUtility
    {
        public static Vector2 ScreenSize => new Vector2(Screen.width, Screen.height);

        /// <summary>
        /// Gets the world screen bounds 
        /// </summary>
        /// <param name="topLeft">The bounds that represent the top left bounds.</param>
        /// <param name="bottomRight">The bounds that represent the bottom right bounds.</param>
        /// <returns>True if the world screen bounds were calculated.</returns>
        public static bool GetWorldScreenBounds(ref Vector3 topLeft, ref Vector3 bottomRight)
        {
            var mainCamera = CameraUtility.MainCamera;
            if (!mainCamera)
            {
                return false;
            }
            topLeft = mainCamera.ScreenToWorldPoint(
                new Vector3(0.0f, 0.0f));
            bottomRight = mainCamera.ScreenToWorldPoint(
                new Vector3(ScreenSize.x, ScreenSize.y));
            return true;
        }
    }
}