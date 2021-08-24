using System.Runtime.InteropServices;
using System.Security;

namespace BulletSharp
{
    public static class Profiling
    {
        [DllImport(Native.Dll, CallingConvention = Native.Conv), SuppressUnmanagedCodeSecurity]
        public static extern void DumpProfilingData();
    }
}