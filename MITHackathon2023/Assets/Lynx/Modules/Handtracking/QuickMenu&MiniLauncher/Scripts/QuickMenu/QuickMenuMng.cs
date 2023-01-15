using Leap.Unity;
using System;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

public class QuickMenuMng : MonoBehaviour
{

    #region INSPECTOR
    public Transform cameraEye;
    [Header("Activation Parameters")]
    public float activationDelay = 1f;
    [Space]
    public float palmFacingDotProdThreshold = 0f;
    public float pinchActivateDist = 20f;
    public float pinchDeactivateDist = 20f;
    public float buttonClosestMaxDist = 0.4f;
    [Space]
    public bool rightHandUseEnabled = true;
    public bool leftHandUseEnabled = true;

    [Header("QuickMenu Elements")]
    public GameObject quickMenuCanvas;
    public GameObject quickMenuFillIndicator;
    public Color baseColor;
    public Color highlightColor;
    public List<QuickMenuButton> quickMenuButtons;

    [Header("QuickMenu Targets")]
    public MiniLauncherMng miniLauncher;
    //public DebugHandsText debugHandsText;
    #endregion

    #region PUBLIC
    //flags pinch
    [HideInInspector] public float pinchRightDist = 10;
    [HideInInspector] public float pinchLeftDist = 10;

    [HideInInspector] public bool pinchLeft = false;
    [HideInInspector] public bool pinchLeftPrev = false;
    
    [HideInInspector] public bool pinchRight = false;
    [HideInInspector] public bool pinchRightPrev = false;
    //flags palm states
    [HideInInspector] public bool palmFacingEyeLeft = false;
    [HideInInspector] public bool palmFacingEyeRight = false;

    [HideInInspector] public bool palmOpenLeft = false;
    [HideInInspector] public bool palmOpenRight = false;


    //flags quickmenu states
    [HideInInspector] public bool quickMenuButtonsActive = false;
    [HideInInspector] public bool quickMenuFreeHandHover = false;
    [HideInInspector] public bool quickMenuUsedByHandRight = false;
    [HideInInspector] public bool quickMenuUsedByHandLeft = false;
    //timer stuff
    [HideInInspector] public bool quickMenuTimerActive = false;
    private float timerStartedTime = 0;
    //quickmenu hovered button stuff
    [HideInInspector] public QuickMenuButton quickMenuButtonHovered;
    private float quickMenuButtonHoveredStartedTime = 0;
    [HideInInspector] public QuickMenuButton quickMenuButtonPressed;
    private float quickMenuButtonPressedStartedTime = 0;
    //[HideInInspector] public bool toggleTest = false;
    #endregion

    #region PRIVATE
    //positions
    private Vector3 quickMenuSpawnPos;
    private Vector3 quickMenuSpawnForward;
    #endregion


    private void OnValidate()
    {
        if (!rightHandUseEnabled && !leftHandUseEnabled) rightHandUseEnabled = true;
        if (pinchDeactivateDist < pinchActivateDist) pinchDeactivateDist = pinchActivateDist;
    }

    private void Awake()
    {
        if (cameraEye == null) cameraEye = Camera.main.gameObject.transform;
    }

    private void Start()
    {
        DeactivateQuickMenu();
    }

    private void Update()
    {
        UpdatePalmFacingFlags(palmFacingDotProdThreshold);
        UpdatePalmOpenFlags();
        UpdatePinchFlags();
        UpdatePinchDistanceFlags();

        //detect pinch start to isolate hand to track for quickmenuupdate
        if (!pinchRightPrev && pinchRight && !quickMenuUsedByHandLeft && rightHandUseEnabled) quickMenuUsedByHandRight = true;
        if (!pinchLeftPrev && pinchLeft && !quickMenuUsedByHandRight && leftHandUseEnabled) quickMenuUsedByHandLeft = true;
        //UpdateQuickMenu will release usedByHandFlags back to false on quickmenurelease conditions
        if (quickMenuUsedByHandRight) UpdateQuickMenu2(Hands.Right, pinchRight, pinchRightPrev ,palmFacingEyeRight, palmOpenRight);
        if (quickMenuUsedByHandLeft) UpdateQuickMenu2(Hands.Left, pinchLeft, pinchLeftPrev, palmFacingEyeLeft, palmOpenLeft);

        

    }


    //Update flags methods
    private void UpdatePalmFacingFlags(float threshold)
    {
        Leap.Hand handRight = Hands.Right;
        if (handRight != null)
        {
            Vector3 palmRightForward = handRight.PalmNormal;
            Vector3 palmRightToEye = cameraEye.transform.position - handRight.PalmPosition;
            palmFacingEyeRight = Vector3.Dot(palmRightForward, Vector3.Normalize(palmRightToEye)) > threshold;
        }
        else palmFacingEyeRight = false;

        Leap.Hand handLeft = Hands.Left;
        if (handLeft != null)
        {
            Vector3 palmLeftForward = handLeft.PalmNormal;
            Vector3 palmLeftToEye = cameraEye.transform.position - handLeft.PalmPosition;
            palmFacingEyeLeft = Vector3.Dot(palmLeftForward, Vector3.Normalize(palmLeftToEye)) > threshold;
        }
        else palmFacingEyeLeft = false;

    }
    private void UpdatePalmOpenFlags()
    {
        Leap.Hand handRight = Hands.Right;
        if (handRight != null)
        {
            bool indexRightIsExtendedLeap = handRight.GetIndex().IsExtended;
            bool middleRightIsExtendedLeap = handRight.GetMiddle().IsExtended;
            bool ringRightIsExtendedLeap = handRight.GetRing().IsExtended;
            bool pinkyRightIsExtendedLeap = handRight.GetPinky().IsExtended;
            //palmOpenRight = pinkyRightIsExtendedLeap && ringRightIsExtendedLeap;
            palmOpenRight = pinkyRightIsExtendedLeap;
        }
        else palmOpenRight = false;

        Leap.Hand handLeft = Hands.Left;
        if (handLeft != null)
        {
            bool indexLeftIsExtendedLeap = handLeft.GetIndex().IsExtended;
            bool middleLeftIsExtendedLeap = handLeft.GetMiddle().IsExtended;
            bool ringLeftIsExtendedLeap = handLeft.GetRing().IsExtended;
            bool pinkyLeftIsExtendedLeap = handLeft.GetPinky().IsExtended;
            //palmOpenLeft = pinkyLeftIsExtendedLeap && ringLeftIsExtendedLeap;
            palmOpenLeft = pinkyLeftIsExtendedLeap;
        }
        else palmOpenLeft = false;
    }
    private void UpdatePinchFlags()
    {
        Leap.Hand handRight = Hands.Right;

        if (!pinchRightPrev && pinchRight) pinchRightPrev = true;
        else if (pinchRightPrev && !pinchRight) pinchRightPrev = false;

        if (handRight != null)
        {
            if(!pinchRight) pinchRight = handRight.PinchDistance < pinchActivateDist;
            else            pinchRight = handRight.PinchDistance < pinchDeactivateDist;
        }
        else pinchRight = false;


        Leap.Hand handLeft = Hands.Left;

        if (!pinchLeftPrev && pinchLeft) pinchLeftPrev = true;
        else if (pinchLeftPrev && !pinchLeft) pinchLeftPrev = false;

        if (handLeft != null)
        {
            if (!pinchLeft) pinchLeft = handLeft.PinchDistance < pinchActivateDist;
            else            pinchLeft = handLeft.PinchDistance < pinchDeactivateDist;
        }
        else pinchLeft = false;
    }
    private void UpdatePinchDistanceFlags()
    {
        Leap.Hand handRight = Hands.Right;
        if (handRight != null)
        {
            pinchRightDist = handRight.PinchDistance;
        }
        else pinchRightDist = Mathf.Infinity;

        Leap.Hand handLeft = Hands.Left;
        if (handLeft != null)
        {
            pinchLeftDist = handLeft.PinchDistance;
        }
        else pinchLeftDist = Mathf.Infinity;

    }


    //QuickMenu methods
    private void UpdateQuickMenu(Leap.Hand hand, bool pinch, bool pinchPrev, bool palmFacingEye, bool palmOpen)
    {
        //pinch start
        if (!pinchPrev && pinch)
        {
            if (palmFacingEye && palmOpen && !quickMenuButtonsActive && !quickMenuTimerActive)
            {
                quickMenuTimerActive = true;
                timerStartedTime = Time.time;
                ActivateQuickMenu(hand);
                ActivateQuickMenuTimer();
            }
        }
        //timer progress interrupt condition
        if (quickMenuTimerActive && (!pinch || !palmFacingEye || !palmOpen) && !quickMenuButtonsActive)
        {
            quickMenuTimerActive = false;
            DeactivateQuickMenu();
            quickMenuUsedByHandRight = false;
            quickMenuUsedByHandLeft = false;
        }
        //timer progress update & quickmenu buttons activate condition
        if (quickMenuTimerActive && !quickMenuButtonsActive && pinch && palmOpen)
        {
            UpdateQuickMenuPinchPos(hand);
            UpdateQuickMenuTimerIndicator();
            float timeElapsed = Time.time - timerStartedTime;
            if (timeElapsed > activationDelay)
            {
                ActivateQuickMenuButtons();
                quickMenuButtonsActive = true;
                quickMenuTimerActive = false;
            }
        }
        //quickmenu buttons update
        if (quickMenuButtonsActive)
        {
            //menu hololense like interaction
            //UpdateQuickMenuTouchPos(hand);
            //UpdateQuickMenuTouchButtons(hand);

            //menu quest like interaction
            UpdateQuickMenuPinchPos(hand);
            UpdateQuickMenuPinchButtons(hand);
        }
        //pinch end
        if (pinchPrev && !pinch)
        {
            if (quickMenuButtonsActive || quickMenuTimerActive)
            {
                if(quickMenuButtonsActive && quickMenuButtonHovered != null)
                {
                    CallHoveredButtonEvent();
                    quickMenuButtonHovered = null;
                }
                DeactivateQuickMenu();
            }
            quickMenuUsedByHandRight = false;
            quickMenuUsedByHandLeft = false;
        }
    }
    private void UpdateQuickMenu2(Leap.Hand hand, bool pinch, bool pinchPrev, bool palmFacingEye, bool palmOpen)
    {
        //test driven by quickmenu states

        //quickmenu off
        if(!quickMenuButtonsActive && !quickMenuTimerActive)
        {
            if (!pinchPrev && pinch && palmFacingEye && palmOpen)
            {
                quickMenuTimerActive = true;
                timerStartedTime = Time.time;
                ActivateQuickMenu(hand);
                ActivateQuickMenuTimer();
            }

        }
        //quickmenu timer on
        if (quickMenuTimerActive && !quickMenuButtonsActive)
        {
            if (!pinch || !palmFacingEye || !palmOpen)
            {
                quickMenuTimerActive = false;
                DeactivateQuickMenu();
            }
            else if (pinch && palmOpen)
            {
                UpdateQuickMenuPinchPos(hand);
                UpdateQuickMenuTimerIndicator();
                float timeElapsed = Time.time - timerStartedTime;
                if (timeElapsed > activationDelay)
                {
                    ActivateQuickMenuButtons();
                    quickMenuButtonsActive = true;
                    quickMenuTimerActive = false;
                }
            }

        }
        //quickmenu active
        if (quickMenuButtonsActive)
        {
            int v = 3;

            
            if (v == 1)
            {
                //menu quest like interaction
                UpdateQuickMenuPinchPos(hand);
                UpdateQuickMenuPinchButtons(hand);

                if (pinchPrev && !pinch)
                {
                    if (quickMenuButtonHovered != null)
                    {
                        CallHoveredButtonEvent();
                        quickMenuButtonHovered = null;
                    }
                    DeactivateQuickMenu();
                }
            }
            
            if (v == 2)
            {
                //menu hololense like interaction
                UpdateQuickMenuTouchPos(hand);
                UpdateQuickMenuTouchButtons(hand);
            }
            if (v == 3)
            {
                //menu hololense like interaction
                UpdateQuickMenuTouchPos(hand);
                UpdateQuickMenuTouchButtons2(hand);
            }


        }

    }

    
    private void ActivateQuickMenu(Leap.Hand hand)
    {
        Vector3 indexTipPos = hand.GetIndex().TipPosition;

        quickMenuSpawnPos = indexTipPos;
        quickMenuSpawnForward = cameraEye.TransformDirection(Vector3.forward);
        quickMenuCanvas.SetActive(true);
    }
    public void DeactivateQuickMenu()
    {
        quickMenuCanvas.SetActive(false);
        quickMenuButtonsActive = false;
        quickMenuTimerActive = false;
        quickMenuButtonHovered = null;

        quickMenuUsedByHandRight = false;
        quickMenuUsedByHandLeft = false;
    }
    
    private void UpdateQuickMenuPinchPos(Leap.Hand hand)
    {
        int v = 2;

        //place at index & oriente towards eye
        if (v == 1)
        {
            transform.position = quickMenuSpawnPos;
            transform.LookAt(cameraEye);
        }

        //orient along eye forward at spawn & keep near hand along eye forward
        if (v == 2)
        {
            Vector3 indexTipPos = hand.GetIndex().TipPosition;

            transform.position = quickMenuSpawnPos;
            //Vector3 eyeForward = cameraEye.TransformDirection(Vector3.forward);
            Vector3 eyeForward = quickMenuSpawnForward;
            float indexTipUIDist = 0.02f;
            transform.rotation = Quaternion.LookRotation(eyeForward);

            float offsetFromIndexAlongEyeFrowardVector = transform.InverseTransformPoint(indexTipPos).z;
            transform.position += eyeForward * (offsetFromIndexAlongEyeFrowardVector - indexTipUIDist);
            //transform.position = quickMenuSpawnPos + Vector3.Normalize(spawnPosToEyeVector) * indexTipUIDist;
        }
    }
    private void UpdateQuickMenuTouchPos(Leap.Hand hand)
    {
        Vector3 indexTipPos = hand.GetIndex().TipPosition;

        transform.position = quickMenuSpawnPos;
        //Vector3 eyeForward = cameraEye.TransformDirection(Vector3.forward);
        Vector3 eyeForward = quickMenuSpawnForward;
        //float indexTipUIDist = 0.02f;
        transform.rotation = Quaternion.LookRotation(eyeForward);

        float offsetFromIndexAlongEyeFrowardVector = transform.InverseTransformPoint(indexTipPos).z;
        //transform.position += eyeForward * (offsetFromIndexAlongEyeFrowardVector - indexTipUIDist);
        float handSideOffset = hand.IsRight ? -1f : 1f;
        transform.position += (eyeForward * (0.06f)) + (transform.right * (0.1f) * handSideOffset * 0f) + (transform.up * (0.05f));
        //transform.position = quickMenuSpawnPos + Vector3.Normalize(spawnPosToEyeVector) * indexTipUIDist
    }

    private void ActivateQuickMenuTimer()
    {
        quickMenuFillIndicator.SetActive(true);
        foreach (QuickMenuButton quickMenuButton in quickMenuButtons) quickMenuButton.gameObject.SetActive(false);
    }
    private void UpdateQuickMenuTimerIndicator()
    {
        float timeElapsed = Time.time - timerStartedTime;
        float timerProgress = Mathf.InverseLerp(0, activationDelay, timeElapsed);
        quickMenuFillIndicator.GetComponent<Image>().fillAmount = timerProgress;
    }

    private void ActivateQuickMenuButtons()
    {
        quickMenuFillIndicator.SetActive(false);
        foreach (QuickMenuButton quickMenuButton in quickMenuButtons) quickMenuButton.gameObject.SetActive(true);
    }
    private void UpdateQuickMenuPinchButtons(Leap.Hand hand)
    {
        Vector3 indexTipPos = hand.GetIndex().TipPosition;

        float closestButtonDist = Mathf.Infinity;
        QuickMenuButton clostestButton = quickMenuButtons[0];
        foreach (QuickMenuButton button in quickMenuButtons)
        {
            float buttonDist = Vector3.Distance(indexTipPos, button.transform.position);
            if(buttonDist < closestButtonDist)
            {
                clostestButton = button;
                closestButtonDist = buttonDist;
            }
        }

        if(closestButtonDist > buttonClosestMaxDist) clostestButton = null;

        foreach (QuickMenuButton button in quickMenuButtons)
        {
            button.circle.color = baseColor;
        }
        if (clostestButton != null) clostestButton.circle.color = highlightColor;

        quickMenuButtonHovered = clostestButton;
    }
    private void UpdateQuickMenuTouchButtons(Leap.Hand hand)
    {
        Vector3 indexTipPos = hand.GetIndex().TipPosition;

        //getting the closest button under dist threshold
        float closestButtonDist = Mathf.Infinity;
        QuickMenuButton clostestButton = quickMenuButtons[0];
        foreach (QuickMenuButton button in quickMenuButtons)
        {
            float buttonDist = Vector3.Distance(indexTipPos, button.transform.position);
            if(buttonDist < closestButtonDist)
            {
                clostestButton = button;
                closestButtonDist = buttonDist;
            }
        }
        if (closestButtonDist > buttonClosestMaxDist) clostestButton = null;

        //recolor buttons 
        foreach (QuickMenuButton button in quickMenuButtons)
        {
            button.circle.color = baseColor;
        }
        if (clostestButton != null)
        {
            clostestButton.circle.color = highlightColor;
        }

        //apply radial fill for button hover progress
        if (clostestButton != null && quickMenuButtonHovered != null && clostestButton.gameObject.name == quickMenuButtonHovered.gameObject.name)
        {
            float timeElapsed = Time.time - quickMenuButtonHoveredStartedTime;
            float buttonTriggerDelay = 0.5f;

            clostestButton.circularFill.fillAmount = Mathf.InverseLerp(0, buttonTriggerDelay, timeElapsed);

            if (timeElapsed > buttonTriggerDelay)
            {
                clostestButton.circularFill.fillAmount = 0;
                CallHoveredButtonEvent();
                quickMenuButtonHovered = null;
                quickMenuButtonHoveredStartedTime = Time.time;
                DeactivateQuickMenu();
            }
        }
        else
        {
            if (clostestButton == null && quickMenuButtonHovered != null)
            {
                quickMenuButtonHovered.circularFill.fillAmount = 0;
                quickMenuButtonHovered = null;
            }

            if (clostestButton != null && quickMenuButtonHovered == null)
            {
                quickMenuButtonHovered = clostestButton;
                quickMenuButtonHovered.circularFill.fillAmount = 0;
                quickMenuButtonHoveredStartedTime = Time.time;
            }

        }



    }

    private void UpdateQuickMenuTouchButtons2(Leap.Hand hand)
    {
        Vector3 indexTipPos = hand.GetIndex().TipPosition;


        //get button hover candidates
        List<QuickMenuButton> hoveredButtons = new List<QuickMenuButton>();
        foreach (QuickMenuButton button in quickMenuButtons)
        {

            Vector3 indexTipRelativeToButton = button.transform.InverseTransformPoint(indexTipPos);
            float buttonZDist = indexTipRelativeToButton.z * button.transform.lossyScale.z * (-1);
            float zDistMax = 0.3f;

            Vector3 indexTipProjectedToButton = button.transform.TransformPoint(new Vector3(indexTipRelativeToButton.x, indexTipRelativeToButton.y, 0));
            float buttonXYDist = Vector3.Distance(indexTipProjectedToButton, button.transform.position);

            if (Mathf.Abs(buttonZDist) <= zDistMax && buttonXYDist <= button.radius * 0.5f)
            {
                hoveredButtons.Add(button);
            }
        }
        
        //getting the closest button under dist threshold among hover candidates
        float closestButtonDist = Mathf.Infinity;
        QuickMenuButton clostestButton = null;
        foreach (QuickMenuButton button in hoveredButtons)
        {
            float buttonDist = Vector3.Distance(indexTipPos, button.transform.position);
            if(buttonDist < closestButtonDist)
            {
                clostestButton = button;
                closestButtonDist = buttonDist;
            }
        }
        //if (closestButtonDist > buttonClosestMaxDist) clostestButton = null;


        //recolor buttons 
        foreach (QuickMenuButton button in quickMenuButtons)
        {
            button.circle.color = baseColor;
        }
        if (clostestButton != null)
        {
            clostestButton.circle.color = highlightColor;
            //clostestButton.circle.color = Color.green;
        }

        //is button pressed ?
        QuickMenuButton pressedButton = null;
        if (clostestButton != null)
        {

            Vector3 indexTipRelativeToButton = clostestButton.transform.InverseTransformPoint(indexTipPos);
            float buttonZDist = indexTipRelativeToButton.z * clostestButton.transform.lossyScale.z * (-1);
            float zDistThreshold = 0.03f;

            if (Mathf.Abs(buttonZDist) <= zDistThreshold)
            {
                pressedButton = clostestButton;
            }
        }

        //if button press same from last frame, update circulare fill with trigger timer
        if (pressedButton == null)
        {
            if (quickMenuButtonPressed != null)
            {
                quickMenuButtonPressed.circularFill.fillAmount = 0;
                quickMenuButtonPressed = null;
                quickMenuButtonPressedStartedTime = Time.time;
            }
        }
        else if (pressedButton != null)
        {
            if (quickMenuButtonPressed == null)
            {
                quickMenuButtonPressed = pressedButton;
                quickMenuButtonPressed.circularFill.fillAmount = 0;
                quickMenuButtonPressedStartedTime = Time.time;
            }
            else if (quickMenuButtonPressed != pressedButton)
            {
                quickMenuButtonPressed.circularFill.fillAmount = 0;
                quickMenuButtonPressed = pressedButton;
                quickMenuButtonPressed.circularFill.fillAmount = 0;
                quickMenuButtonPressedStartedTime = Time.time;
            }
            else if (quickMenuButtonPressed == pressedButton)
            {
                float timeElapsed = Time.time - quickMenuButtonPressedStartedTime;
                float buttonTriggerDelay = 0.8f;

                pressedButton.circularFill.fillAmount = Mathf.InverseLerp(0, buttonTriggerDelay, timeElapsed);

                if (timeElapsed > buttonTriggerDelay)
                {
                    CallButtonEvent(pressedButton);
                    pressedButton.circularFill.fillAmount = 0;
                    quickMenuButtonPressed = null;
                    quickMenuButtonPressedStartedTime = Time.time;
                    DeactivateQuickMenu();
                }
            }

        }


    }


    //QuickMenu public button methods
    public void ToggleMiniLauncher()
    {
        if (miniLauncher.active) miniLauncher.DeactivateMiniLauncher();
        else                     miniLauncher.ActivateMiniLauncher();
    }
    public void CallHoveredButtonEvent()
    {
        if(quickMenuButtonHovered.triggerEvent != null) 
            quickMenuButtonHovered.triggerEvent.Invoke();
    }

    public void CallButtonEvent(QuickMenuButton button)
    {
        if(button.triggerEvent != null)
            button.triggerEvent.Invoke();
    }

    

}
