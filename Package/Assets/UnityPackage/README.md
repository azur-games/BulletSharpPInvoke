# ECS Framework
UPM package implementing ECS framework with code generation to avoid GC overhead.

## How to install

### From remote repository
Add in `Packages/manifest.json` to `dependencies`:

```javascript
"com.aig.ecs": "git+ssh://git@github.com/azur-games/ecs-framework.git#0.1.0",
```

### From local path
<details>
	<summary>From local repository</summary>

	"com.aig.ecs": "file:///D/repos/ecs-framework/.git#0.1.0",
</details>

<details>
	<summary>From local working copy</summary>

	"com.aig.ecs": "file:D:/repos/ecs-framework/Assets",
</details>

<details>
	<summary>What is the difference?</summary>
	<p>
		Local repository is resolved just like normal Git repository with optionally specified revision.<br />
		Local working copy is being copied as is into dependent project, without running any Git process.
	</p>
</details>

## Access to this package
### Over SSH
Normally, access over SSH should do a trick.

### Over HTTPS (strongly not recommended) ⚠️
If some of your teammates are not using SSH, in this case you should provide PAT (Personal Access Token) in repo URL.

Follow this steps:
1. Create your own [PAT](https://docs.github.com/en/github/authenticating-to-github/creating-a-personal-access-token).
2. Paste your user name and PAT here:
	```
	"com.aig.ecs": "https://USERNAME:PAT@github.com/azur-games/ecs-framework.git#0.1.0",
	```
3. Commit `Packages/manifest.json` to share package and access with your team.
4. Check it out and use.
5. Revoke access via given PAT [here](https://github.com/settings/tokens), if needed.
