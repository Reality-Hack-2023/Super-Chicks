using System.Collections.Generic;
using MITHack.Robot.Utils.Components;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;
using UnityEngine.Serialization;
using Object = UnityEngine.Object;
using Random = UnityEngine.Random;

namespace MITHack.Robot.Spawner
{
    
    public class Spawner :  MonoBehaviour
    {
        #region defines

        [System.Serializable]
        private enum SpawnerSpawnType
        {
            [InspectorName("Spawn in Order")]
            SpawnType_InOrder,
            [InspectorName("Spawn Randomly")]
            SpawnType_Random
        }
        
        private enum SpawnerState
        {
            [InspectorName("Initial Delay")]
            State_InitialDelay,
            [InspectorName("Spawner Delay")]
            State_SpawnerDelay
        }

        /// <summary>
        /// The allocator for the prefab spawner.
        /// </summary>
        [System.Serializable]
        public struct PrefabSpawnerAllocator : IObjectPoolAllocator<PooledObjectComponent>, ISerializationCallbackReceiver
        {
            [SerializeField]
            private GameObject prefab;
            
            public PooledObjectComponent Allocate()
            {
                Debug.Assert(prefab, "Prefab Must Exist for this Allocator.");
                var allocated = Object.Instantiate(prefab);
                if (!allocated) return null;
                var allocatedObject = allocated.GetComponent<PooledObjectComponent>();
                return allocatedObject;
            }

            public void DeAllocate(ref PooledObjectComponent obj)
            {
                if (!obj)
                {
                    obj = null;
                    return;
                }
                Object.Destroy(obj.gameObject);
                obj = null;
            }

            public void OnBeforeSerialize()
            {
                if (prefab)
                {
                    var pooledObject = prefab.GetComponent<PooledObjectComponent>();
                    if (!pooledObject)
                    {
                        prefab = null;
                    }
                }
            }

            public void OnAfterDeserialize()
            {
            }
        }
        
        #endregion
        
        [Header("Variables")] 
        [SerializeField, Min(0.0f)] 
        private float minSpawnDelay = 0.5f;
        [Space]
        [SerializeField, Min(0.0f)]
        private float startSpawnTimeDiff = 6.0f;
        [SerializeField]
        private float endSpawnTimeDiff = 0.5f;
        [Space]
        [SerializeField, Min(1)]
        private int totalNumberSpawnedBetweenTimeChange = 3;
        [FormerlySerializedAs("totalTimeSubtracted")] [SerializeField, Min(0.1f)]
        private float totalTimeDiffSubtracted = 0.5f;

        [Header("Spawner Variables")] 
        [SerializeField, Min(1)]
        private int totalPrefabs = 32;
        [SerializeField]
        private PrefabSpawnerAllocator prefab;
        [Space] 
        [SerializeField]
        private SpawnerSpawnType spawnType;
        [SerializeField]
        private List<Transform> spawnPoints;


        private ObjectPoolInstance<PrefabSpawnerAllocator, PooledObjectComponent, 
            PooledObjectComponent.PooledObjectSpawnContext> _prefabObjectPool;
        private SpawnerState _spawnerState = SpawnerState.State_InitialDelay;
        private int _totalSpawned = 0;
        private float _totalSpawnTimeDifference = 0.0f;
        private float _currentTimeDifference = 0.0f;

        private int _lastSpawnPoint = -1;

        public int TotalSpawned => _totalSpawned;

        private void Awake()
        {
            _prefabObjectPool ??= new ObjectPoolInstance<PrefabSpawnerAllocator, PooledObjectComponent, PooledObjectComponent.PooledObjectSpawnContext>(
                totalPrefabs, prefab);
        }

        private void Start()
        {
            _totalSpawnTimeDifference = _currentTimeDifference = minSpawnDelay;
            _prefabObjectPool?.Initialize();
        }

        private void OnDestroy()
        {
            _prefabObjectPool?.DeInitialize();
        }

        private void Update()
        {
            UpdateSpawnerTime(Time.deltaTime);
        }

        private void UpdateSpawnerTime(float deltaTime)
        {
            if (_currentTimeDifference > 0.0f)
            {
                _currentTimeDifference -= deltaTime;
                if (_currentTimeDifference <= 0.0f)
                {
                    ExecuteSpawn();
                }
            }
        }

        /// <summary>
        /// Spawns the prefab and updates everything else in the spawner.
        /// </summary>
        private void ExecuteSpawn()
        {
            // Instantiates the Game Object.
            if (!Spawn())
            {
                return;
            }
            _totalSpawned++;

            if (_spawnerState == SpawnerState.State_InitialDelay)
            {
                _currentTimeDifference = _totalSpawnTimeDifference = startSpawnTimeDiff;
                _spawnerState = SpawnerState.State_SpawnerDelay;
                return;
            }

            if (_totalSpawned % totalNumberSpawnedBetweenTimeChange == 0
                && _totalSpawned != 0)
            {
                _totalSpawnTimeDifference = Mathf.Max(
                    _totalSpawnTimeDifference - totalTimeDiffSubtracted,
                    endSpawnTimeDiff);
            }
            _currentTimeDifference = _totalSpawnTimeDifference;
        }

        private bool Spawn()
        { 
            var selectSpawnPoint = SelectSpawnPoint();
            if (!selectSpawnPoint)
            {
                return false;
            }

            PooledObjectComponent instantiated = null;
            if ((_prefabObjectPool?.Allocate(ref instantiated, new PooledObjectComponent.PooledObjectSpawnContext()
                {
                    position = selectSpawnPoint.position,
                    rotation = selectSpawnPoint.rotation
                })) ?? false)
            {
                return true;
            }
            return false;
        }

        private Transform SelectSpawnPoint()
        {
            var transformCount = spawnPoints.Count;
            if (transformCount <= 0)
            {
                return null;
            }
            
            switch (spawnType)
            {
                case SpawnerSpawnType.SpawnType_Random:
                {
                    int random;
                    do
                    {
                        random = Random.Range(0, transformCount);
                    } while (random == _lastSpawnPoint);
                    _lastSpawnPoint = random;
                    return spawnPoints[random];
                }
                case SpawnerSpawnType.SpawnType_InOrder:
                {
                    var currentSpawnPoint = ++_lastSpawnPoint;
                    return spawnPoints[currentSpawnPoint % transformCount];
                }
            }
            return null;
        }
    }
}