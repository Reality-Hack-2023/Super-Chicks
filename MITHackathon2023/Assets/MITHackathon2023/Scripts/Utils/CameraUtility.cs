using System.Collections.Generic;
using UnityEngine;
using UnityEngine.SceneManagement;

namespace MITHack.Robot.Utils
{
    public static class CameraUtility
    {
        private static readonly List<Camera> InternalCameras = new List<Camera>();

        public static List<Camera> Cameras => InternalCameras;

        public static Camera MainCamera => Camera.main;

        public static Camera FirstCamera
        {
            get
            {
                if (InternalCameras.Count <= 0)
                {
                    return null;
                }
                return InternalCameras[0];
            }
        }

        
        [RuntimeInitializeOnLoadMethod]
        private static void Initialize()
        {
            SceneManager.sceneLoaded += OnSceneLoaded;
        }

        private static void OnSceneLoaded(Scene scene, LoadSceneMode args)
        {
            UpdateCameras();
        }

        private static void UpdateCameras()
        {
            var cameraUpdate = Object.FindObjectsOfType<Camera>();
            InternalCameras.Clear();
            InternalCameras.AddRange(cameraUpdate);
        }
    }
}