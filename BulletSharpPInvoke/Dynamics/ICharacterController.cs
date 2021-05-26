using BulletSharp.Math;

namespace BulletSharp
{
	public interface ICharacterController : IAction
	{
        bool CanJump { get; }
        bool OnGround { get; }
        void Jump();
        void PlayerStep(CollisionWorld collisionWorld, float dt);
        void PreStep(CollisionWorld collisionWorld);
		void Reset(CollisionWorld collisionWorld);
        void SetWalkDirection(Vector3 walkDirection);
		void Warp(ref Vector3 origin);
	}
}
