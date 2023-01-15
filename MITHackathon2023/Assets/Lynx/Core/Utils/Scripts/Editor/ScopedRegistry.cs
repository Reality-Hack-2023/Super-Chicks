/**
 * @file ScopedRegistry.cs
 * 
 * @author GC
 * 
 * @brief Helper for scoped registries management.
 */

using System.Collections.Generic;

namespace lynx
{
    [System.Serializable]
    public class ScopedRegistry
    {
        public string name;
        public string url;
        public List<string> scopes = new List<string>();

        public ScopedRegistry(string name, string url, List<string> scopes)
        {
            this.name = name;
            this.url = url;
            this.scopes = scopes;
        }
    }
}