using MITHack.Robot.Spawner;
using UnityEngine;

namespace MITHack.Robot.Utils
{
    [System.Serializable]
    public struct Prefab : IObjectPoolAllocator<GameObject>
    {
        [SerializeField]
        private GameObject prefab;

        public GameObject Instantiated()
        {
            return prefab ? Object.Instantiate(prefab) : null;
        }

        GameObject IObjectPoolAllocator<GameObject>.Allocate()
        {
            if (!prefab)
            {
                return null;
            }
            return Object.Instantiate(prefab);
        }

        public void DeAllocate(ref GameObject obj)
        {
            Object.Destroy(obj);
            obj = null;
        }
    }

    [System.Serializable]
    public struct Prefab<TComponent> : IObjectPoolAllocator<TComponent>, ISerializationCallbackReceiver
    {
        [SerializeField]
        private GameObject prefab;

        public TComponent Instantiate()
        {
            if (!prefab)
            {
                return default;
            }
            var instantiated = Object.Instantiate(prefab);
            var instantiatedComponent = instantiated.GetComponent<TComponent>();
            return instantiatedComponent;
        }

        TComponent IObjectPoolAllocator<TComponent>.Allocate()
        {
            return Instantiate();
        }

        void IObjectPoolAllocator<TComponent>.DeAllocate(ref TComponent obj)
        {
            if (obj is Component c
                && c)
            {
                Object.Destroy(c.gameObject);
                obj = default;
            }
        }

        public void OnBeforeSerialize()
        {
            if (prefab)
            {
                var component = prefab.GetComponent<TComponent>();
                if (component == null)
                {
                    prefab = null;
                }
            }
        }

        public void OnAfterDeserialize()
        {
        }
    }
}