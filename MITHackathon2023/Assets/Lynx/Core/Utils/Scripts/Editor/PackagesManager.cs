/**
 * @file PackagesManager.cs
 * 
 * @author GC && Geoffrey Marhuenda
 * 
 * @brief Helper for packages management.
 */

using System.Collections.Generic;
using UnityEngine;

using System.IO;
//using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEditor.PackageManager;

namespace lynx
{
    public class PackagesManager : MonoBehaviour
    {

        public const string XR_OPENXR_PCKG_STR = "com.unity.xr.openxr@1.6.0";
        public const string XR_INTERACTION_PCKG_STR = "com.unity.xr.interaction.toolkit";


        public const string ULTRALEAP_TRACKING_STR = "com.ultraleap.tracking";
        public const string ULTRALEAP_TRACKING_PREVIEW_STR = "com.ultraleap.tracking.preview";
        public static string PackagesManifestPath
        {
            get => Path.Combine(Application.dataPath, "..", "Packages", "manifest.json");
        }

        #region EDITOR MENU
        /// <summary>
        /// Automatically install OpenXR packages.
        /// </summary>
        [MenuItem("Lynx/Packages/Install OpenXR", false, 95)]
        public static void InstallOpenXR()
        {
            Client.Add(XR_OPENXR_PCKG_STR);
        }

        /// <summary>
        /// Automatically install OpenXR packages.
        /// </summary>
        [MenuItem("Lynx/Packages/Install XR Interaction Toolkit", false, 98)]
        public static void InstallOpenXRI()
        {
            Client.Add(XR_INTERACTION_PCKG_STR);
        }


        /// <summary>
        /// Automatically add and install Ultraleap packages.
        /// </summary>
        //[MenuItem("Lynx/Packages/Install Ultraleap packages", false, 102)]
        //public static void InstallHandtrackingPackages()
        //{
        //    ScopedRegistry scopedRegistry = new ScopedRegistry("Ultraleap", "https://package.openupm.com", new List<string>() { "com.ultraleap" });

        //    PackagesManager.AddScopedRegistry(scopedRegistry);
        //    Client.Add(ULTRALEAP_TRACKING_PREVIEW_STR);
        //}

        #endregion

        #region SCOPED REGISTRIES
        //public static void AddScopedRegistry(ScopedRegistry scopedRegistry)
        //{
        //    // Read JSON File;
        //    JObject manifestJSON = JObject.Parse(File.ReadAllText(PackagesManifestPath));
        //    JArray Jregistries = (JArray)manifestJSON["scopedRegistries"];

        //    // Add scopedRegistries if missing
        //    if (Jregistries == null)
        //    {
        //        manifestJSON.Add("scopedRegistries", new JArray());
        //        Jregistries = (JArray)manifestJSON["scopedRegistries"];
        //    }
        //    else
        //    {
        //        bool packageAlreadyExists = false;
        //        foreach (JObject j in Jregistries)
        //        {
        //            ScopedRegistry current = (ScopedRegistry)scopedRegistry;
        //            if (current.url == scopedRegistry.url)
        //                packageAlreadyExists = true;

        //        }

        //        if (packageAlreadyExists)
        //        {
        //            Debug.LogWarning("Package not added (already exists).");
        //            return;
        //        }
        //    }


        //    // Create new Scoped Registry JObject
        //    JObject JRegistry = new JObject();

        //    // Set Scoped Registry name
        //    JRegistry["name"] = scopedRegistry.name;

        //    // Set Scoped Registry url
        //    JRegistry["url"] = scopedRegistry.url;

        //    // Set Scoped Registry scopes
        //    JArray scopes = new JArray();
        //    foreach (var scope in scopedRegistry.scopes)
        //    {
        //        scopes.Add(scope);
        //    }
        //    JRegistry["scopes"] = scopes;

        //    // Write JSON File
        //    Jregistries.Add(JRegistry);
        //    File.WriteAllText(PackagesManifestPath, manifestJSON.ToString());
        //    AssetDatabase.Refresh();
        //}
        #endregion
    }
}