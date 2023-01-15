using System;
using UnityEngine;

namespace MITHack.Robot.Utils.Components
{
    [RequireComponent(typeof(Collider))]
    public class CollisionEventsListener3D : MonoBehaviour
    {
        #region defines
        
        public delegate void CollisionEventsListenerGenericDelegate<in TContext>(TContext context);

        public struct CollisionEventContext
        {
            public CollisionEventsListener3D collisionEventsListener3D;
            public Collision collision;
        }

        public struct TriggerEventContext
        {
            public CollisionEventsListener3D collisionEventsListener3D;
            public Collider otherCollider;
        }
        
        #endregion

        public CollisionEventsListenerGenericDelegate<CollisionEventContext> CollisionEnterEvent;
        public CollisionEventsListenerGenericDelegate<CollisionEventContext> CollisionExitEvent;
        public CollisionEventsListenerGenericDelegate<TriggerEventContext> TriggerEnterEvent;
        public CollisionEventsListenerGenericDelegate<TriggerEventContext> TriggerExitEvent;

        private Collider _collider;

        public TCollider GetCollider<TCollider>()
            where TCollider : Collider
        {
            return _collider as TCollider;
        }

        public Collider GetCollider() => _collider;

        private void OnCollisionEnter(Collision collision)
        {
            CollisionEnterEvent?.Invoke(new CollisionEventContext
            {
                collision = collision,
                collisionEventsListener3D = this
            });
        }

        private void OnCollisionExit(Collision other)
        {
            CollisionExitEvent?.Invoke(new CollisionEventContext
            {
                collision = other,
                collisionEventsListener3D = this
            });
        }

        private void OnTriggerEnter(Collider other)
        {
            TriggerEnterEvent?.Invoke(new TriggerEventContext
            {
                collisionEventsListener3D = this,
                otherCollider = other
            });
        }

        private void OnTriggerExit(Collider other)
        {
            TriggerExitEvent?.Invoke(new TriggerEventContext
            {
                collisionEventsListener3D = this,
                otherCollider = other
            });
        }
    }
}