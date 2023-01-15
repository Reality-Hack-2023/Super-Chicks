using System;
using MITHack.Robot.Spawner;
using MITHack.Robot.Utils;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace MITHack.Robot.Entities
{
    public class ChickenTarget : MonoBehaviour, IPooledObject
    {
        #region targets
        
        public struct ChickenTargetAllocContext
        {
            public Vector3? position;
            public Quaternion? rotation;
            public Vector3? scale;
        }
        
        #endregion
        
        #region statics

        private static ObjectPoolInstance<Prefab<ChickenTarget>, ChickenTarget, ChickenTargetAllocContext> ChickenTargetPool;

        public static void Initialize(int capacity, in Prefab<ChickenTarget> allocator)
        {
            if (ChickenTargetPool != null)
            {
                return;
            }
            ChickenTargetPool =
                new ObjectPoolInstance<Prefab<ChickenTarget>, ChickenTarget, ChickenTargetAllocContext>(
                    capacity, allocator);
            ChickenTargetPool.Initialize();
        }
        
        /// <summary>
        /// Allocates a new chicken target.
        /// </summary>
        /// <param name="ctx">The chicken target context.</param>
        /// <returns>The new chicken target, or null otherwise.</returns>
        public static ChickenTarget Allocate(in ChickenTargetAllocContext ctx)
        {
            ChickenTarget chickenTarget = null;
            if ((ChickenTargetPool?.Allocate(ref chickenTarget, ctx)) ?? false)
            {
                return chickenTarget;
            }
            return null;
        }

        /// <summary>
        /// Deallocates the chicken target from the pool.
        /// </summary>
        /// <param name="chickenTarget">The associated chicken target.</param>
        public static void DeAllocate(ref ChickenTarget chickenTarget)
        {
            ChickenTargetPool?.DeAllocate(ref chickenTarget);
        }
        
        #endregion

        [Header("Variables")]
        [SerializeField, Min(0.0f)]
        private float totalLifeLength = 2.0f;

        private float _currentLifeLength = 0.0f;

        public void OnInitialized<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool) 
            where TObjectAllocContext : unmanaged
        {
            if (pool != ChickenTargetPool)
            {
                return;
            }
            gameObject.SetActive(false);
        }

        public void OnAllocated<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool, in TObjectAllocContext context) 
            where TObjectAllocContext : unmanaged
        {
            if (pool != ChickenTargetPool)
            {
                return;
            }
            var cpy = context;
            var allocContext = UnsafeUtility.As<TObjectAllocContext, ChickenTargetAllocContext>(ref cpy);

            var cachedTransform = transform;
            if (allocContext.position.HasValue)
            {
                cachedTransform.position = allocContext.position.Value;
            }
            if (allocContext.rotation.HasValue)
            {
                cachedTransform.rotation = allocContext.rotation.Value;
            }
            if (allocContext.scale.HasValue)
            {
                cachedTransform.localScale = allocContext.scale.Value;
            }
            gameObject.SetActive(true);
            _currentLifeLength = totalLifeLength;
        }
        

        public void OnDeAllocated<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool) 
            where TObjectAllocContext : unmanaged
        {
            gameObject.SetActive(false);
        }
        
        public void DeAllocate()
        {
            var cpy = this;
            ChickenTargetPool.DeAllocate(ref cpy);
        }

        private void Update()
        {
            if (_currentLifeLength > 0.0f)
            {
                _currentLifeLength -= Time.deltaTime;
                if (_currentLifeLength <= 0.0f)
                {
                    DeAllocate();
                }
            }
        }
    }
}