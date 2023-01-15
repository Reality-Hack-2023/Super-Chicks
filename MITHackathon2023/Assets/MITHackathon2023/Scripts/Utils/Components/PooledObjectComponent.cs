using MITHack.Robot.Spawner;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace MITHack.Robot.Utils.Components
{
    public class PooledObjectComponent : MonoBehaviour, IPooledObject
    {
        public struct PooledObjectSpawnContext
        {
            public Vector3? position;
            public Quaternion? rotation;
            public Vector3? scale;
        }
        
        public IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectSpawnContext> initializedEvent;
        public IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectSpawnContext> allocatedEvent;
        public IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectSpawnContext> deallocatedEvent;

        private IObjectPool<PooledObjectComponent, PooledObjectSpawnContext> _currentObjectPool = null;

        public IObjectPool<PooledObjectComponent, PooledObjectSpawnContext> CurrentObjectPool => _currentObjectPool;

        public void DeAllocate()
        {
            if (CurrentObjectPool != null)
            {
                var cpy = this;
                CurrentObjectPool.DeAllocate(ref cpy);
            }
        }
        
        public void OnInitialized<TSelf, TAllocContext>(IObjectPool<TSelf, TAllocContext> pool) 
            where TAllocContext : unmanaged
        {
            if (pool is IObjectPool<PooledObjectComponent, PooledObjectSpawnContext> pooledObjects)
            {
                _currentObjectPool = pooledObjects;
                gameObject.SetActive(false);
                initializedEvent?.Invoke(this, pooledObjects);
            }
        }

        public void OnAllocated<TSelf, TAllocContext>(IObjectPool<TSelf, TAllocContext> pool,
            in TAllocContext allocContext)
            where TAllocContext : unmanaged
        {
            if (pool is IObjectPool<PooledObjectComponent, PooledObjectSpawnContext> pooledObjects)
            {
                // Already know that the type is a pooled object spawn context, so we can just cpy.
                var cpy = allocContext;
                var ctx = UnsafeUtility.As<TAllocContext, PooledObjectSpawnContext>(ref cpy);
                
                var cachedTransform = transform;
                if (ctx.rotation.HasValue)
                {
                    cachedTransform.rotation = ctx.rotation.Value;
                }
                if (ctx.position.HasValue)
                {
                    cachedTransform.position = ctx.position.Value;
                }
                if (ctx.scale.HasValue)
                {
                    cachedTransform.localScale = ctx.scale.Value;
                }

                gameObject.SetActive(true);
                allocatedEvent?.Invoke(this, pooledObjects);
            }
        }

        public void OnDeAllocated<TSelf, TAllocContext>(IObjectPool<TSelf, TAllocContext> pool) 
            where TAllocContext : unmanaged
        {
            if (pool is IObjectPool<PooledObjectComponent, PooledObjectSpawnContext> pooledObjects)
            {
                gameObject.SetActive(false);
                deallocatedEvent?.Invoke(this, pooledObjects);
            }
        }
    }
}