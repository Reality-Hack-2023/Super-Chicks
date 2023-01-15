using MITHack.Robot.Game;
using UnityEngine;
using UnityEngine.UI;

namespace MITHack.Robot.Utils.UI
{
    public class GameUI : MonoBehaviour
    {
        [Header("Variables")] 
        [SerializeField]
        private bool setChildOfCamera = true;
        
        [Header("References")] 
        [SerializeField]
        private Canvas canvas;
        
        [Header("Chicken Nuggets")] 
        [SerializeField]
        private Image counterImage;
        [SerializeField]
        private TMPro.TMP_Text counterText;

        private void Update()
        {
            if (canvas)
            {
                canvas.worldCamera = CameraUtility.MainCamera;
                if (setChildOfCamera)
                {
                    var cachedTransform = transform;
                    cachedTransform.SetParent(canvas.worldCamera.transform);
                }
            }
            
            var gameManager = GameManager.Get();
            if (gameManager)
            {
                var chickensKilled = gameManager.ChickensKilled;
                if (counterText)
                {
                    counterText.enabled = chickensKilled > 0;
                    counterText.text = $"x{gameManager.ChickensKilled}";
                }

                if (counterImage)
                {
                    counterImage.enabled = chickensKilled > 0;
                }
            }
        }
    }
}