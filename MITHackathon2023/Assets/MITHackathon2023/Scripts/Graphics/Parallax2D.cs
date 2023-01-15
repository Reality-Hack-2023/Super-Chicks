using System.Collections.Generic;
using MITHack.Robot.Utils;
using UnityEngine;
using UnityEngine.Serialization;

namespace MITHack.Robot.Graphics
{
    [System.Serializable]
    public struct ParallaxLayer
    {
        [Header("References")]
        [SerializeField]
        private SpriteRenderer spriteRenderer;

        [FormerlySerializedAs("layerSpeed")]
        [Header("Variables")] 
        [SerializeField, Min(0.0f)]
        private float layerSpeedMultiplier;

        public SpriteRenderer SpriteRenderer => spriteRenderer;

        public float LayerSpeed => layerSpeedMultiplier;
    }

    public class Parallax2D : GenericSingleton<Parallax2D>
    {
        private static readonly int MainTex = Shader.PropertyToID("_MainTex");

        [Header("Parallax References")]
        [SerializeField]
        private List<ParallaxLayer> parallaxLayers;

        [Header("Parallax Variables")] 
        [SerializeField, Min(0.0f)]
        private float parallaxSpeed = 1.0f;

        public float ParallaxSpeed
        {
            get => parallaxSpeed;
            set => parallaxSpeed = Mathf.Max(value, 0.0f);
        }

        protected override void Awake()
        {
            base.Awake();
            InitializeLayers();
        }

        protected virtual void Update()
        {
            UpdateParallaxLayer(Time.deltaTime);
        }

        private void UpdateParallaxLayer(float deltaTime)
        {
            for (var index = 0; index < parallaxLayers.Count; ++index)
            {
                var parallaxLayer = parallaxLayers[index];
                var spriteRenderer = parallaxLayer.SpriteRenderer;
                if (!spriteRenderer) continue;

                var material = spriteRenderer.material;
                if (!material) continue;
                
                var textureOffset = material.GetTextureOffset(MainTex);
                textureOffset.x += parallaxLayer.LayerSpeed * deltaTime;
                material.SetTextureOffset(MainTex, textureOffset);
            }
        }

        private void InitializeLayers()
        {
            parallaxLayers ??= new List<ParallaxLayer>();
            for (var index = 0; index < parallaxLayers.Count; ++index)
            {
                var layer = parallaxLayers[index];
                var spriteRenderer = layer.SpriteRenderer;
                if (!spriteRenderer) continue;
                spriteRenderer.drawMode = SpriteDrawMode.Tiled;
            }
        }
    }
}
