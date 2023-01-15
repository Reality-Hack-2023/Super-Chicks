using MITHack.Robot.Spawner;
using MITHack.Robot.Utils.Components;
using UnityEngine;

namespace MITHack.Robot.Entities.Projectile
{
    [RequireComponent(typeof(PooledObjectComponent))]
    public class RobotProjectile : MonoBehaviour
    {
        public System.Action<RobotEntity.ChickenKilledContext> ProjectileHitEvent;
        
        [Header("Variables")] 
        [SerializeField, Min(0.0f)]
        private float projectileSpeed = 100.0f;
        [SerializeField, Min(0.0f)]
        private float timeToDestroy = 3.0f;

        [Header("References")] [SerializeField]
        private new Rigidbody rigidbody;
        [SerializeField]
        private CollisionEventsListener3D collisionEventsListener;
        
        private IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> _allocatedEvent;
        private IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> _deallocatedEvent;
        private CollisionEventsListener3D.CollisionEventsListenerGenericDelegate<CollisionEventsListener3D.TriggerEventContext> _onTriggerEvent;

        private PooledObjectComponent _pooledObjectComponent;
        private float _currentDestroyTime = 0.0f;
        
        private Rigidbody Rigidbody => rigidbody ??= GetComponent<Rigidbody>();
        
        private void Awake()
        {
            _pooledObjectComponent = GetComponent<PooledObjectComponent>();
            _allocatedEvent = OnAllocated;
            _deallocatedEvent = OnDeallocated;
            _onTriggerEvent = OnTriggerEvent;
        }

        private void OnEnable()
        {
            if (_pooledObjectComponent)
            {
                _pooledObjectComponent.allocatedEvent += _allocatedEvent;
                _pooledObjectComponent.deallocatedEvent += _deallocatedEvent;
            }

            if (collisionEventsListener)
            {
                collisionEventsListener.TriggerEnterEvent
                    += _onTriggerEvent;
            }
        }

        private void OnDisable()
        {
            if (_pooledObjectComponent)
            {
                _pooledObjectComponent.allocatedEvent -= _allocatedEvent;
                _pooledObjectComponent.deallocatedEvent -= _deallocatedEvent;
            }

            if (collisionEventsListener)
            {
                collisionEventsListener.TriggerEnterEvent
                    -= _onTriggerEvent;
            }
        }

        private void Update()
        {
            if (_currentDestroyTime > 0.0f)
            {
                _currentDestroyTime -= Time.deltaTime;
                if (_currentDestroyTime <= 0.0f)
                {
                    _pooledObjectComponent.DeAllocate();
                    _currentDestroyTime = 0.0f;
                }
            }
        }

        private void OnAllocated(PooledObjectComponent pooledObject, IObjectPool<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> pool)
        {
            if (Rigidbody)
            {
                _currentDestroyTime = timeToDestroy;
                var cachedTransform = transform;
                Rigidbody.AddForce(cachedTransform.forward * projectileSpeed, ForceMode.Impulse);
            }
        }
        
        private void OnDeallocated(PooledObjectComponent pooledObject, IObjectPool<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> pool)
        {
            var cachedTransform = transform;
            cachedTransform.position = Vector3.zero;
            
            if (Rigidbody)
            {
                Rigidbody.velocity = Vector3.zero;
                Rigidbody.angularVelocity = Vector3.zero;
                Rigidbody.Sleep();
            }
        }
        
        private void OnTriggerEvent(CollisionEventsListener3D.TriggerEventContext context)
        {
            var other = context.otherCollider;
            if (!other)
            {
                return;
            }

            var component = other.GetComponent<ChickenEntity>()
                            ?? other.GetComponentInParent<ChickenEntity>();
            if (component
                && component.isActiveAndEnabled)
            {
                component.Explode();
                ProjectileHitEvent?.Invoke(new RobotEntity.ChickenKilledContext()
                {
                    chickenEntity = component
                });
            }
            _pooledObjectComponent.DeAllocate();
        }
    }
}