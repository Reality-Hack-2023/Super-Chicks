using UnityEngine;

namespace MITHack.Robot.Utils
{
    public class GenericSingleton<TSelf> : MonoBehaviour
        where TSelf : GenericSingleton<TSelf>
    {
        #region statics

        private static TSelf _protectedSelf = null;

        public static TSelf Get() => _protectedSelf;

        public static TSelfInherited Get<TSelfInherited>()
            where TSelfInherited : TSelf
        {
            return _protectedSelf as TSelfInherited;
        }
        
        #endregion

        [Header("Singleton Variables")]
        [SerializeField]
        private bool dontDestroyOnLoad;

        protected virtual void Awake()
        {
            if (_protectedSelf
                && _protectedSelf != this)
            {
                Destroy(gameObject);
                return;
            }

            _protectedSelf = (TSelf)this;
            if (dontDestroyOnLoad)
            {
                DontDestroyOnLoad(gameObject);
            }
        }
    }
}