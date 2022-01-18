#if !UNITY_2017_1_OR_NEWER
using System;

namespace AOT
{
    public sealed class MonoPInvokeCallback : Attribute
    {
        public MonoPInvokeCallback(Type type) { }
    }
}
#endif