using BulletSharp.Math;

namespace BulletSharp
{
    public class ResolveStepUp
	{
		public bool IsStep;
		public Vector3 RealPosWorld;
		public float DistSqr;
		
		private readonly DynamicCharacterController _controller;
		private readonly CollisionWorld _world;
		private Vector3 _stepPos;
		private Vector3 _stepNormal;
		private float _originHeight;
		private Vector3 _stepLocal;
		private float _normalTolerance = 0.3f;
		private float _stepSearchOvershoot = 0.01f;
		private float _stepDirectionMinDot = 0.3f;
		private readonly ClosestRayResultCallback _closestRay;
		
		public ResolveStepUp(DynamicCharacterController controller, CollisionWorld world, short staticRaycatGroup, short staticRaycastMask)
		{
			_controller = controller;
			_world = world;
			_closestRay = new ClosestRayResultCallback();
			_closestRay.CollisionFilterGroup = staticRaycatGroup;
			_closestRay.CollisionFilterMask = staticRaycastMask;
		}

		public void Resolve(ManifoldPoint cp)
		{
			_stepPos = cp.PositionWorldOnB;
			_stepNormal = cp.NormalWorldOnB;
			
			_originHeight = _controller.CapsuleHalfHeight + _controller.CapsuleRadius;
			var transform = _controller.RigidBody.WorldTransform;
			IsStep = CheckPreconditions(ref transform) && FindRealPoint(ref transform) && canFit(ref transform);
			if (IsStep)
			{
				Vector3 origin = transform.Origin;
				origin.Y = origin.Y - _originHeight;
				DistSqr = (origin - RealPosWorld).Length;
			}
		}

		private bool CheckPreconditions(ref Matrix transform)
		{
			if (_controller.RigidBody.LinearVelocity.LengthSquared < 0.001f)
			{
				return false;
			}

			/* A step has little vertical component in its normal.
			 * This strategy is not perfect, as it does not work with end points of the
			 * ramps, but I do not have a solution for this, at the moment.
			 *
			 * Nice trick by https://cobertos.com/blog/post/how-to-climb-stairs-unity3d/
			 */
			/*if (System.Math.Abs(mStepNormal.Y) > mNormalTolerance)
			{
				return false;
			}*/
			/* We can step only when we are on ground, but since while we are doing
			this check the characater cannot move, if we consider the lowest point of
			the capsule as center, we have a constant error, that might be negligible.
			On the other hand, we could also just store the various collisions and use
			the correct ground point after we detected it. */
			Vector3 origin = transform.Origin;
			{
				float approximateHeight = _stepPos.Y - origin.Y + _originHeight;
				if (approximateHeight >= _controller.MaxStepHeight)
				{
					return false;
				}
			}

			Matrix toLocal = transform;
			toLocal.Invert();
			/* Don't step if it's in a direction opposite to our movement.
			 * This is a quick test, but likely not enough accurate. */
			var transformed = Vector3.TransformCoordinate(_stepPos, toLocal);
			_stepLocal = new Vector3(transformed.X, transformed.Y, transformed.Z);
			Vector3 stepDir = new Vector3(_stepLocal.X, 0, _stepLocal.Z);
			stepDir.Normalize();
			var moveDir = _controller.RigidBody.LinearVelocity;
			moveDir.Normalize();
			if (Vector3.Dot(stepDir, moveDir) < _stepDirectionMinDot)
			{
				return false;
			}

			/*
			_closestRay.Setup(ref origin, ref mStepPos);
			mWorld.RayTest(origin, mStepPos, _closestRay);
			if (_closestRay.HasHit && (1 - _closestRay.ClosestHitFraction) > 0.0001f)
			{
				return false;
			}*/

			return true;
		}

		private bool FindRealPoint(ref Matrix transform)
		{
			/* Look for the real height of the step.
			 * Move a bit along the ground-collision direction, horizontally,
			 * and vertically use 0 and the max step height. */
			Vector3 minTargetPoint = new Vector3(_stepLocal.X, 0, _stepLocal.Z);
			minTargetPoint *= 1 + _stepSearchOvershoot / minTargetPoint.Length;
			// As above: we are using capsule lowest point, rather than ground
			minTargetPoint.Y -= _originHeight;

			Vector3 maxTargetPoint = minTargetPoint;
			maxTargetPoint.Y += _controller.MaxStepHeight + _stepSearchOvershoot;
			minTargetPoint = Vector3.TransformCoordinate(minTargetPoint, transform);
			maxTargetPoint = Vector3.TransformCoordinate(maxTargetPoint, transform);

			_closestRay.Setup(ref maxTargetPoint, ref minTargetPoint);
			_world.RayTest(maxTargetPoint, minTargetPoint, _closestRay);
			if (!_closestRay.HasHit)
			{
				return false;
			}
			
			if ((1 - _closestRay.ClosestHitFraction) < 0.0001f)
			{
				// Almost at the minimum point, can walk normally
				return false;
			}

			RealPosWorld = _closestRay.HitPointWorld;
			return true;
		}

		private bool canFit(ref Matrix transform)
		{
			// Not very robust test, but still better than nothing
			Vector3 horFrom = RealPosWorld;
			horFrom.Y += _originHeight;
			Vector3 horTo = new Vector3(_stepLocal.X, 0, _stepLocal.Z);
			horTo *= _controller.CapsuleRadius / horTo.Length;
			horTo = horFrom + transform.Origin * horTo;
			Vector3 vertFrom = RealPosWorld;
			Vector3 vertTo = horFrom;
			vertTo.Y += _originHeight;

			_closestRay.Setup(ref vertFrom, ref vertTo);
			_world.RayTest(vertFrom, vertTo, _closestRay);
			if (_closestRay.HasHit)
			{
				return false;
			}

			_closestRay.Setup(ref horFrom, ref horTo);
			_world.RayTest(horFrom, horTo, _closestRay);
			if (_closestRay.HasHit)
			{
				return false;
			}

			return true;
		}
	};
}