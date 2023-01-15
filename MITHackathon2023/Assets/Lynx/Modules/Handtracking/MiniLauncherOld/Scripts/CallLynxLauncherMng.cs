using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using lynx;

public class CallLynxLauncherMng : MonoBehaviour
{
    public void BackToLauncher()
    {

        lynx.LynxAPI.SetVR();

        bool fail = false;

        AndroidJavaClass up = new AndroidJavaClass("com.unity3d.player.UnityPlayer");
        AndroidJavaObject ca = up.GetStatic<AndroidJavaObject>("currentActivity");
        AndroidJavaObject packageManager = ca.Call<AndroidJavaObject>("getPackageManager");

        AndroidJavaObject launchIntent = null;

        string lynxLauncherPackageName = "com.lynx.scenehome";

        try
        {
            launchIntent = packageManager.Call<AndroidJavaObject>("getLaunchIntentForPackage", lynxLauncherPackageName);
        }
        catch (System.Exception)
        {
            fail = true;
        }

        if (fail)
        {  //open app in store
            Application.OpenURL("https://lynx-r.com/");
        }
        else //open the app
            ca.Call("startActivity", launchIntent);


        Debug.Log("@@@@@@@@@@@@@@ Before finishAndRemoveTask");
        ca.Call("finishAndRemoveTask");

        up.Dispose();
        ca.Dispose();
        packageManager.Dispose();
        launchIntent.Dispose();

        Debug.Log("@@@@@@@@@@@@@@ Before Application quit called");
        Application.Quit();

    }
}
