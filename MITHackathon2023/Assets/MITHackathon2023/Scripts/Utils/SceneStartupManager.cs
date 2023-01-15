using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.SceneManagement;

#if UNITY_EDITOR
using UnityEditor.SceneManagement;
#endif

namespace TestJarLabs.SceneReference.Runtime
{
    /// <summary>
    /// Scene Startup manager that can load different scenes in a particular level.
    /// </summary>
    [ExecuteAlways]
    public class SceneStartupManager : MonoBehaviour
    {
#if EFLATUN_SCENE_REFERENCE
        [SerializeField]
        [InspectorName("Scenes Loaded On Startup")]
        [Tooltip("Scenes to have loaded when this scene is first opened.")]
        private List<Eflatun.SceneReference.SceneReference> eflatunScenesToLoad = new List<Eflatun.SceneReference.SceneReference>();
#endif
        
        private bool _hasStartupHappened = false;

#if UNITY_EDITOR
        public void Editor_LoadSceneOnStartup()
        {
#if EFLATUN_SCENE_REFERENCE
            foreach (var scene in eflatunScenesToLoad)
            {
                if (!SceneManager.GetSceneByPath(scene.Path).isLoaded)
                {
                    EditorSceneManager.OpenScene(scene.Path, OpenSceneMode.Additive);
                }
            }
#endif
        }
#endif

        private void LoadScenesOnStartup()
        {
#if EFLATUN_SCENE_REFERENCE
            foreach (var scene in eflatunScenesToLoad)
            {
                if (!SceneManager.GetSceneByPath(scene.Path).isLoaded)
                {
                    SceneManager.LoadScene(scene.Path, LoadSceneMode.Additive);
                }
            }
#endif
        }

        private void CleanAllScenesToLoad()
        {
            CleanScenesToLoad();
        }

        private void CleanScenesToLoad(Scene scene = default, string path = "")
        {
#if EFLATUN_SCENE_REFERENCE
            eflatunScenesToLoad.RemoveAll((inScene) =>
            {
                if (inScene == null) return true;
                try
                {
                    if (!inScene.HasValue) return true;
                    return inScene.Path.Length == 0;
                }
                catch (Exception)
                {
                    // ignored
                }
                return true;
            });
            
            eflatunScenesToLoad = eflatunScenesToLoad.Distinct().ToList();
            eflatunScenesToLoad.Sort((a, b) 
                => string.Compare(a.Path, b.Path, StringComparison.Ordinal));
#endif
        }

        private void Start()
        {
            CleanScenesToLoad();

            if (!Application.isPlaying)
            {
#if UNITY_EDITOR
                EditorSceneManager.sceneSaving += CleanScenesToLoad;

                if (!_hasStartupHappened)
                {
                    Editor_LoadSceneOnStartup();
                }
                _hasStartupHappened = true;
#endif
            }
            else
            {
                if (!_hasStartupHappened)
                {
                    LoadScenesOnStartup();
                }
                _hasStartupHappened = true;
            }
        }
    }
}