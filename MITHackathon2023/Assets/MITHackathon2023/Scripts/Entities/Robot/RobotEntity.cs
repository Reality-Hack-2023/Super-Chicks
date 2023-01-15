using System.Collections.Generic;
using MITHack.Robot.Entities.Projectile;
using MITHack.Robot.Game;
using MITHack.Robot.Spawner;
using MITHack.Robot.Utils;
using MITHack.Robot.Utils.Components;
using UnityEngine;

namespace MITHack.Robot.Entities
{
    public class RobotEntity : GenericSingleton<RobotEntity>
    {
        #region defines
        
        public enum RobotEntityState
        {
            StateAlive,
            StateDead
        }

        public struct RobotEntityStateChangeContext
        {
            public RobotEntityState prev;
            public RobotEntityState next;
        }

        public struct ChickenKilledContext
        {
            public ChickenEntity chickenEntity;
        }
        
        public delegate void RobotEntityGenericDelegate<in TContext>(TContext context);
        
        #endregion

        public RobotEntityGenericDelegate<RobotEntityStateChangeContext> StateChangedEvent;
        public System.Action<ChickenKilledContext> ChickenKilledEvent;

        [Header("Projectile")] 
        [SerializeField, Min(0.0f)]
        private float projectileTimeBetweenEachFire = 0.2f;
        [Space]
        [SerializeField]
        private Prefab<PooledObjectComponent> projectilePrefab;
        [SerializeField]
        private Transform shootLocation;

        [Header("References")] 
        [SerializeField]
        private RobotMovement robotMovement;

        [Header("UI")] 
        [SerializeField]
        private Sprite lifeSprite;
        [SerializeField]
        private Sprite noLifeSprite;
        [SerializeField] 
        private List<SpriteRenderer> livesSprites;
        
        private IPooledObject.PooledObjectDelegate<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> _onProjectileDeallocated;

        private ObjectPoolInstance<Prefab<PooledObjectComponent>, PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> _objectPool;
        private RobotEntityState _robotEntityState = RobotEntityState.StateAlive;

        private float _currentProjectileTime = 0.0f;
        
        public RobotEntityState EntityState => _robotEntityState;

        public RobotMovement RobotMovement => robotMovement ??= GetComponent<RobotMovement>();

        protected override void Awake()
        {
            base.Awake();

            _objectPool ??=
                new ObjectPoolInstance<Prefab<PooledObjectComponent>, PooledObjectComponent,
                    PooledObjectComponent.PooledObjectSpawnContext>(64, projectilePrefab);
            _onProjectileDeallocated = OnProjectileDeallocated;
        }


        private void Start()
        {
            _objectPool?.Initialize();
            _currentProjectileTime = projectileTimeBetweenEachFire;
        }

        private void OnDestroy()
        {
            _objectPool.DeInitialize();
        }

        private void Update()
        {
            UpdateLives();
            
            switch (EntityState)
            {
                case RobotEntityState.StateAlive:
                {
                    if (_currentProjectileTime > 0.0f)
                    {
                        _currentProjectileTime -= Time.deltaTime;
                
                        if (_currentProjectileTime <= 0.0f)
                        {
                            _currentProjectileTime = projectileTimeBetweenEachFire;
                            Fire();
                        }
                    }
                    break;
                }
            }
        }

        private void SetState(RobotEntityState entityState)
        {
            if (_robotEntityState != entityState)
            {
                StateChangedEvent?.Invoke(new RobotEntityStateChangeContext
                {
                    prev = _robotEntityState,
                    next = entityState
                });
                _robotEntityState = entityState;
            }
        }
        
        public void Kill()
        {
            if (EntityState != RobotEntityState.StateAlive)
            {
                return;
            }
            SetState(RobotEntityState.StateDead);
            Revive();
        }

        private void UpdateLives()
        {
            var gameManager = GameManager.Get();
            if (!gameManager)
            {
                foreach (var sprite in livesSprites)
                {
                    if (!sprite) continue;
                    sprite.enabled = false;
                }
                return;
            }

            var currentLives = gameManager.CurrentLives;
            for (var spriteLifeIndex = 0; spriteLifeIndex < livesSprites.Count; ++spriteLifeIndex)
            {
                var sprite = livesSprites[spriteLifeIndex];
                if (!sprite) continue;
                var associatedLife = spriteLifeIndex + 1;
                sprite.sprite = associatedLife <= currentLives ? lifeSprite : noLifeSprite;
            }
        }

        public void Revive()
        {
            SetState(RobotEntityState.StateAlive);
        }

        public void Fire()
        {
            var location = shootLocation ? shootLocation : transform;
            PooledObjectComponent objectPooledReference = null;
            if (_objectPool?.Allocate(ref objectPooledReference, new PooledObjectComponent.PooledObjectSpawnContext
                {
                    position = location.position,
                    rotation = location.rotation
                }) ?? false)
            {
                var robotProjectile = objectPooledReference.GetComponent<RobotProjectile>();
                if (robotProjectile)
                {
                    robotProjectile.ProjectileHitEvent += ChickenKilledEvent;
                }

                objectPooledReference.deallocatedEvent += _onProjectileDeallocated;
            }
        }
        
        private void OnProjectileDeallocated(PooledObjectComponent pooledObject,
            IObjectPool<PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext> pool)
        {
            if (pooledObject)
            {
                var robotProjectile = pooledObject.GetComponent<RobotProjectile>();
                if (robotProjectile)
                {
                    robotProjectile.ProjectileHitEvent -= ChickenKilledEvent;
                }
            }
        }

        
        private void OnDrawGizmos()
        {
            var gameManager = GameManager.Get();
            if (gameManager)
            {
                var currentLives = gameManager.CurrentLives;
                switch (currentLives)
                {
                    case 0:
                    {
                        Gizmos.color = Color.red;
                        break;
                    }
                    case 1:
                    {
                        Gizmos.color = new Color(1.0f,115.0f / 255.0f,0);
                        break;
                    }
                    case 2:
                    {
                        Gizmos.color = new Color(1.0f, 213.0f / 255.0f, 0.0f);
                        break;
                    }
                    case 3:
                    {
                        Gizmos.color = new Color(0.0f, 1.0f, 38.0f / 255.0f);
                        break;
                    }
                }
                Gizmos.DrawSphere(transform.position, 0.2f);
            }
        }
    }
}