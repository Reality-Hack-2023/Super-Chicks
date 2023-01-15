using System.Collections.Generic;

namespace MITHack.Robot.Spawner
{
    public interface IPooledObject
    {
        public delegate void PooledObjectDelegate<TSelf, out TObjectAllocContext>(TSelf self, IObjectPool<TSelf, TObjectAllocContext> pool) 
            where TObjectAllocContext : unmanaged;

        /// <summary>
        /// Deallocates the pooled object.
        /// </summary>
        /// <returns>True if deallocated, false otherwise.</returns>
        public void DeAllocate();

        /// <summary>
        /// Called when the object is initialized by the pool.
        /// </summary>
        /// <param name="pool">The pool.</param>
        /// <typeparam name="TSelf">The object pool associated with the object.</typeparam>
        /// <typeparam name="TObjectAllocContext">The allocation context.</typeparam>
        public void OnInitialized<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool) 
            where TObjectAllocContext : unmanaged;

        /// <summary>
        /// Called when the object is allocated by the pool.
        /// </summary>
        /// <param name="pool">The object pool.</param>
        /// <param name="context">The allocation context.</param>
        /// <typeparam name="TSelf">The type of deallocated object.</typeparam>
        /// <typeparam name="TObjectAllocContext">The allocation context.</typeparam>
        public void OnAllocated<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool,
            in TObjectAllocContext context) 
            where TObjectAllocContext : unmanaged;

        /// <summary>
        /// Called when the object is allocated by the pool.
        /// </summary>
        /// <param name="pool">The object pool.</param>
        /// <typeparam name="TSelf">The type of deallocated object.</typeparam>
        /// <typeparam name="TObjectAllocContext">The allocation context.</typeparam>
        public void OnDeAllocated<TSelf, TObjectAllocContext>(IObjectPool<TSelf, TObjectAllocContext> pool) 
            where TObjectAllocContext : unmanaged;
    }

    /// <summary>
    /// Interface for allocating the object.
    /// </summary>
    /// <typeparam name="TObject">The associated object we want to allocate.</typeparam>
    public interface IObjectPoolAllocator<TObject>
    {
        public TObject Allocate();

        /// <summary>
        /// Represents when the allocator explicitly deallocates the object.
        /// </summary>
        /// <returns>The Object Associated.</returns>
        public void DeAllocate(ref TObject obj);
    }
    
    public interface IObjectPool { }

    public interface IObjectPool<TObject, in TObjectAllocContext> : IObjectPool
        where TObjectAllocContext : unmanaged
    {
        /// <summary>
        /// Allocates the object pool.
        /// </summary>
        /// <param name="obj">The object.</param>
        /// <param name="context">The context for allocating the object.</param>
        /// <typeparam name="TObject">The type of the object.</typeparam>
        /// <returns>True or false.</returns>
        public bool Allocate(ref TObject obj, TObjectAllocContext context);

        /// <summary>
        /// Allocates to the object pool.
        /// </summary>
        /// <param name="obj">The object.</param>
        /// <param name="context">The context for allocating the object.</param>
        /// <typeparam name="TInheritedObject">The inherited type of the object.</typeparam>
        /// <returns>True or false.</returns>
        public bool Allocate<TInheritedObject>(ref TInheritedObject obj, TObjectAllocContext context)
            where TInheritedObject : TObject;


        /// <summary>
        /// Deallocates from the object pool.
        /// </summary>
        /// <param name="obj">The object.</param>
        public void DeAllocate(ref TObject obj);
        
        /// <summary>
        /// Deallocates from the object pool.
        /// </summary>
        /// <param name="obj">The object.</param>
        /// <typeparam name="TInheritedObject">The inherited type of object.</typeparam>
        public void DeAllocate<TInheritedObject>(ref TInheritedObject obj)
            where TInheritedObject : TObject;
    }

    /// <summary>
    /// Represents an instance of an object pool.
    /// </summary>
    /// <typeparam name="TObjectPoolAllocator">Represents the allocator for the object pool.</typeparam>
    /// <typeparam name="TObject">The type of object.</typeparam>
    /// <typeparam name="TObjectAllocContext">The allocation context.</typeparam>
    public class ObjectPoolInstance<TObjectPoolAllocator, TObject, TObjectAllocContext> : IObjectPool<TObject, TObjectAllocContext>
        where TObject : class
        where TObjectPoolAllocator : struct, IObjectPoolAllocator<TObject>
        where TObjectAllocContext : unmanaged
    {
        private struct SpawnedObject
        {
            public bool enabled;
        }

        private readonly Dictionary<TObject, SpawnedObject> _spawnedObjects;
        
        private TObjectPoolAllocator _allocator;
        private readonly int _capacity;

        private bool _initialized = false;

        public bool Initialized => _initialized;
        
        public ObjectPoolInstance(int capacity, in TObjectPoolAllocator allocator)
        {
            _allocator = allocator;
            _capacity = capacity;
            _spawnedObjects = new Dictionary<TObject, SpawnedObject>();
        }

        public void Initialize()
        {
            if (_initialized)
            {
                return;
            }

            for (var index = 0; index < _capacity; ++index)
            {
                var instantiated = _allocator.Allocate();
                if (instantiated == null) continue;

                if (instantiated is IPooledObject pooledObject)
                {
                    pooledObject.OnInitialized(this);
                }
                
                _spawnedObjects.Add(instantiated, new SpawnedObject
                {
                    enabled = false
                });
            }
            _initialized = true;
        }

        public bool Allocate(ref TObject obj, TObjectAllocContext context) => Allocate<TObject>(ref obj, context);
        
        public bool Allocate<TInheritedObject>(ref TInheritedObject obj, TObjectAllocContext context)
            where TInheritedObject : TObject
        {
            if (!_initialized)
            {
                return false;
            }

            var nextObject = GetNextOpenObject<TInheritedObject>();
            if (nextObject == null)
            {
                return false;
            }
            obj = nextObject;
            // Updates the object pool data.
            var spawnedObjectData = _spawnedObjects[obj];
            spawnedObjectData.enabled = true;
            _spawnedObjects[obj] = spawnedObjectData;

            if (obj is IPooledObject pooledObject)
            {
                pooledObject.OnAllocated(this, context);
            }
            return true;
        }

        public void DeAllocate(ref TObject obj) => DeAllocate<TObject>(ref obj);

        public void DeAllocate<TInheritedObject>(ref TInheritedObject obj)
            where TInheritedObject : TObject
        {
            if (!_initialized)
            {
                return;
            }

            if (_spawnedObjects.TryGetValue(obj, out var outValue)
                && outValue.enabled)
            {
                if (obj is IPooledObject pooledObject)
                {
                    pooledObject.OnDeAllocated(this);
                }
                outValue.enabled = false;
                _spawnedObjects[obj] = outValue;
                obj = default;
            }
        }

        public void DeInitialize()
        {
            _initialized = false;
            foreach (var instance in _spawnedObjects)
            {
                var spawnedObject = instance.Key;
                _allocator.DeAllocate(ref spawnedObject);
            }
            _spawnedObjects.Clear();
        }

        /// <summary>
        /// Gets the next open object in the object pool.
        /// </summary>
        /// <returns>The associated object.</returns>
        private TInheritedObject GetNextOpenObject<TInheritedObject>()
        {
            foreach (var obj in _spawnedObjects)
            {
                if (!obj.Value.enabled
                    && obj.Key is TInheritedObject inherited)
                {
                    return inherited;
                }
            }
            return default;
        }
    }
}