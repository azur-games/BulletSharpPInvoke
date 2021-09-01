using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;
using Vector4 = BulletSharp.Math.Vector4;

namespace BulletSharp
{
    public class FindGroundAndSteps : ContactResultCallback
    {
	    public bool HaveGround;
	    public Vector3 GroundPoint;
	    public Vector3 GroundNormal;
	    public bool HaveStep;
	    public Vector3 StepPoint;
	    
	    private Vector3 _stepNormal;
	    private readonly DynamicCharacterController _controller;
	    private readonly CollisionWorld _world;
	    private float _stepDistSqr;
	    private readonly ResolveStepUp _resolveStepUp;
	    
		public FindGroundAndSteps(DynamicCharacterController controller, CollisionWorld world, short staticRaycastGroup, short staticRaycastMask)
		{
			_controller = controller;
			_world = world;
			_resolveStepUp = new ResolveStepUp(_controller, _world, staticRaycastGroup, staticRaycastMask);
			CollisionFilterGroup = staticRaycastGroup;
			CollisionFilterMask = staticRaycastMask;
		}

		public void Reset()
		{
			HaveGround = false;
			HaveStep = false;
		}

		public override float AddSingleResult(ManifoldPoint cp, CollisionObjectWrapper colObj0, int partId0, int index0,
			CollisionObjectWrapper colObj1, int partId1, int index1)
		{
			if (colObj0.CollisionObject == _controller.RigidBody)
			{
				CheckGround(cp);
				_resolveStepUp.Resolve(cp);
				if (_resolveStepUp.IsStep)
				{
					if (!HaveStep || _resolveStepUp.DistSqr < _stepDistSqr)
					{
						StepPoint = _resolveStepUp.RealPosWorld;
						_stepNormal = cp.NormalWorldOnB;
						_stepDistSqr = _resolveStepUp.DistSqr;
					}
					HaveStep = true;
				}
			}

			// By looking at btCollisionWorld.cpp, it seems Bullet ignores this value
			return 0;
		}
		
		public void CheckGround(ManifoldPoint cp)
		{
			if (HaveGround)
			{
				return;
			}

			Matrix inverse = _controller.RigidBody.WorldTransform;
			inverse.Invert();
			Vector3 localPoint = Vector3.TransformCoordinate(cp.PositionWorldOnB, inverse);
			localPoint = new Vector3(localPoint.X, localPoint.Y + _controller.CapsuleHalfHeight, localPoint.Z);

			float r = localPoint.Length;
			float cosTheta = localPoint.Y / r;

			if (System.Math.Abs(r - _controller.CapsuleRadius) <= _controller.GroundSearchMargin
			    && cosTheta < _controller.MaxCosGround)
			{
				HaveGround = true;
				GroundPoint = cp.PositionWorldOnB;
				GroundNormal = cp.NormalWorldOnB;
			}
		}

		public Vector3 getInvNormal()
		{
			if (!HaveStep) {
				return new  Vector3(0, 0, 0);
			}

			/*
			 Original code
			 btMatrix3x3 frame;
			// Build it as step to world
			frame[2].setValue(0, 0, 1);
			frame[1] = mStepNormal;
			frame[0] = frame[1].cross(frame[2]);
			frame[0].normalize();
			// Convert it to world to step
			frame = frame.transpose();
			return frame[1];
			*/
			
			Matrix frame = Matrix.Zero;
			// Build it as step to world
			frame.Column3 = new Vector4(0, 0, 1, 0);
			frame.Column2 = new Vector4(_stepNormal, 0);
			var crossed = Vector3.Cross(
				new Vector3(frame.Column2.X, frame.Column2.Y, frame.Column2.Z),
				new Vector3(frame.Column3.X, frame.Column3.Y, frame.Column3.Z));
			frame.Column1 = new Vector4(crossed.X, crossed.Y, crossed.Z, 0);
			frame.Column1.Normalize();
			// Convert it to world to step
			frame.Transpose();
			return new Vector3(frame.Column2.X, frame.Column2.Y, frame.Column2.Z);
		}
    };
}