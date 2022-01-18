BulletSharp
===

This is a fork of the BulletSharpPInvoke project. The c# wrappers have been modified slightly so that they work in Unity3d and Mono 2.0 instead of .Net 4.5.

BulletSharp is a .NET wrapper for the [Bullet](http://bulletphysics.org/) physics library.

This version uses Platform Invoke. There is also an equivalent version written in C++/CLI: https://github.com/AndresTraks/BulletSharp

libbulletc is a C interface to Bullet. It compiles into a .dll or .so file that exports Bullet functions.

BulletSharpPInvoke is a .NET library that proxies calls from .NET to libbulletc.

BulletSharpGen generates partial code for BulletSharp P/Invoke, libbulletc and also BulletSharp C++/CLI based on Bullet header files.

The benefit of P/Invoke over C++/CLI is that it runs on all platforms that support P/Invoke into shared user-mode libraries (Windows, Unix, Mac OS). See also [Supported platforms](https://github.com/AndresTraks/BulletSharp/wiki/Supported-platforms).

# How to install

## From remote repository
Add in `Packages/manifest.json` to `dependencies`:

```javascript
"com.phong13.bulletsharp": "git+ssh://git@github.com/azur-games/BulletSharpPInvoke.git#0.9.9-patch.12+azur.f3e9703f",
```

## From local path
<details>
	<summary>From local repository</summary>

	"com.phong13.bulletsharp": "file:///D/repos/BulletSharpPInvoke/.git#0.9.9-patch.12+azur.f3e9703f",
</details>

<details>
	<summary>From local working copy</summary>

	"com.phong13.bulletsharp": "file:D:/repos/BulletSharpPInvoke/",
</details>

<details>
	<summary>What is the difference?</summary>
	<p>
		Local repository is resolved just like normal Git repository with optionally specified revision.<br />
		Local working copy is being copied as is into dependent project, without running any Git process.
	</p>
</details>

# Access to this package
## Over SSH
Normally, access over SSH should do a trick.

## Over HTTPS (strongly not recommended) ⚠️
If some of your teammates are not using SSH, in this case you should provide PAT (Personal Access Token) in repo URL.

Follow this steps:
1. Create your own [PAT](https://docs.github.com/en/github/authenticating-to-github/creating-a-personal-access-token).
2. Paste your user name and PAT here:
	```
	"com.phong13.bulletsharp": "https://USERNAME:PAT@github.com/azur-games/BulletSharpPInvoke.git#0.9.9-patch.12+azur.f3e9703f",
	```
3. Commit `Packages/manifest.json` to share package and access with your team.
4. Check it out and use.
5. Revoke access via given PAT [here](https://github.com/settings/tokens), if needed.