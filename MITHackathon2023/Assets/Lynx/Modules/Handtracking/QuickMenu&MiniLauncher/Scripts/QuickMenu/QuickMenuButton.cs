using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Events;

public class QuickMenuButton : MonoBehaviour
{

    public Image circle;
    public Image circularFill;
    public float radius;
    [Space]
    public UnityEvent triggerEvent;



    private void OnValidate()
    {
        radius = transform.localScale.x * 0.04f;
    }

    private void Awake()
    {
        radius = transform.localScale.x * 0.04f;
    }

}
