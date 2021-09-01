using BulletSharp.Math;
using Vector3 = BulletSharp.Math.Vector3;

namespace BulletSharp
{
    public class DynamicCharacterController : IAction
    {
	    public RigidBody RigidBody
	    {
		    get;
	    }
	    
	    public float MaxLinearVelocity = 11;
	    public float WalkAcceleration = 30.0f;
	    public float JumpSpeed = 10;
	    public float SpeedDamping = 0.1f;
	    public float MaxStepHeight = 1.2f;
	    public float SteppingSpeed = 5;
	    public float CapsuleRadius;
	    public float CapsuleHalfHeight;
	    public float MaxCosGround = -0.70710678f;
	    public float GroundSearchMargin = 0.01f;
	    public Vector3 GroundNormal => _groundNormal;

	    public bool OnGround
	    {
		    get;
		    private set;
	    }
	    private float _maxLinearVelocitySquared => MaxLinearVelocity * MaxLinearVelocity;
	    private Vector3 _targetSpeed;
	    private bool _isJumping;
	    private Vector3 _jumpDir;
	    private bool _stepping;
	    private Vector3 _steppingTo;
	    private Vector3 _steppingInvNormal;
	    private Vector3 _groundNormal;
	    private readonly FindGroundAndSteps _findGroundAndSteps;
	    private Vector3 _gravity;

	    public DynamicCharacterController(CollisionWorld collisionWorld, RigidBody body, 
		    CapsuleShape shape, short staticRaycastGroup, short staticRaycastMask)
	    {
		    _gravity = body.Gravity;
		    _findGroundAndSteps = new FindGroundAndSteps(this, collisionWorld, staticRaycastGroup, staticRaycastMask);
		    RigidBody = body;
		    CapsuleRadius = shape.Radius;
		    CapsuleHalfHeight = shape.HalfHeight;
		    RigidBody.Friction = 0;
		    RigidBody.RollingFriction = 0;
		    SetupBody();
		    ResetStatus();
	    }

	    public void UpdateAction(CollisionWorld collisionWorld, float deltaTimeStep)
	    {
		    _findGroundAndSteps.Reset();
		    collisionWorld.ContactTest(RigidBody, _findGroundAndSteps);
		    OnGround = _findGroundAndSteps.HaveGround;
		    _groundNormal = _findGroundAndSteps.GroundNormal;

		    UpdateVelocity(deltaTimeStep);
		    if (_stepping || _findGroundAndSteps.HaveStep)
		    {
			    if (!_stepping)
			    {
				    _steppingTo = _findGroundAndSteps.StepPoint;
				    _steppingInvNormal = _findGroundAndSteps.getInvNormal();
			    }
			    StepUp(deltaTimeStep);
		    }

		    if (OnGround || _stepping)
		    {
			    /* Avoid going down on ramps, if already on ground, and clearGravity()
			    is not enough */
			    RigidBody.Gravity = Vector3.Zero;
		    }
		    else
		    {
			    RigidBody.Gravity = _gravity;
		    }
	    }

	    public void DebugDraw(IDebugDraw debugDrawer)
	    {
		    if (_stepping)
		    {
			    var b = new Vector3(0, 0, 1);
			    var color = new Vector3(0, 0.3f, 1);
			    debugDrawer.DrawContactPoint(ref _steppingTo, ref b, 0, 1000,
				    ref color);
		    }
	    }
	    
	    public void SetTargetSpeed(Vector3 targetSpeed)
	    {
		    _targetSpeed = targetSpeed;
	    }

	    public void ResetStatus()
	    {
		    _targetSpeed = new Vector3(0, 0, 0);
		    _isJumping = false;
		    OnGround = false;
		    RigidBody.Gravity = _gravity;
		    CancelStep();
	    }

		public void Jump(Vector3 dir)
		{
			if (!OnGround)
			{
				return;
			}

			_isJumping = true;
			dir.Y = 0;
			_jumpDir = dir;
			if (dir.LengthSquared < 0.001f)
			{
				_jumpDir = new Vector3(0, 0, 1);
			}
			_jumpDir += Vector3.UnitY;
			_jumpDir.Normalize();
		}

		public void SetupBody()
		{
			RigidBody.SetSleepingThresholds(0.0f, 0.0f);
			RigidBody.AngularFactor = Vector3.Zero;
		}

		public void UpdateVelocity(float dt)
		{
			var basis = RigidBody.MotionState.WorldTransform;
			Matrix inv = basis;
			inv.Invert();

			Vector3 linearVelocity = Vector3.TransformNormal(RigidBody.LinearVelocity, inv);
			
			if (_targetSpeed.LengthSquared < 0.0001f && OnGround)
			{
				linearVelocity *= SpeedDamping;
			}
			else if (OnGround || linearVelocity.Y > 0)
			{
				var velXz = new Vector3(linearVelocity.X, 0, linearVelocity.Z);
				var moveXz = new Vector3(_targetSpeed.X, 0, _targetSpeed.Z);
				float velDelta = (moveXz - velXz).Length;
				float t = System.Math.Min(1, WalkAcceleration / (velDelta + 0.00001f));
				velXz = Vector3.Lerp(velXz, moveXz, t);

				float speed2 = velXz.X * velXz.X + velXz.Z * velXz.Z;
				float correction = 1;
				if (velXz.LengthSquared > _maxLinearVelocitySquared)
				{
					correction = (float)System.Math.Sqrt(_maxLinearVelocitySquared / speed2);
				}
				
				linearVelocity.X = velXz.X * correction;
				linearVelocity.Z = velXz.Z * correction;
			}

			if (_isJumping)
			{
				linearVelocity += JumpSpeed * _jumpDir;
				_isJumping = false;
				CancelStep();
			}

			RigidBody.LinearVelocity = Vector3.TransformNormal(linearVelocity, RigidBody.MotionState.WorldTransform);
		}

		public void StepUp(float dt)
		{
			Matrix transform = RigidBody.MotionState.WorldTransform;

			if (!_stepping)
			{
				_stepping = true;
				_steppingTo.Y += CapsuleHalfHeight + CapsuleRadius;
			}

			/* Bullet reacts with velocity also with kinematic bodies, althogh this
			should not change anything, until we do not go back to dynamic body. */

			Vector3 origin = transform.Origin;
			float hor = Vector3.Dot(origin - _steppingTo, _steppingInvNormal);

			Vector3 stepDir = _steppingTo - origin;
			stepDir.Y = 0;
			stepDir.Normalize();

			float speed = Vector3.Dot(stepDir, Vector3.TransformNormal(_targetSpeed, transform.Basis))
			                 * SteppingSpeed;
			origin.Y += speed * dt * 2;
			float dv = _steppingTo.Y - origin.Y;

			if (dv <= -1 * MaxStepHeight || _targetSpeed.LengthSquared < 0.0001f || speed <= 0)
			{
				CancelStep();
			}
			else if (dv < CapsuleRadius)
			{
				float dh = (float)System.Math.Sqrt(dv * (2 * CapsuleRadius - dv));
				if (dh < System.Math.Abs(hor))
				{
					float advance = System.Math.Min(dh * System.Math.Sign(hor) - hor, speed * dt);
					var addVec = _steppingInvNormal * advance;
					transform.Origin = transform.Origin + addVec;
				}
			}
			RigidBody.MotionState.WorldTransform = transform;
		}

		private void CancelStep()
		{
			_stepping = false;
		}

		public void SetJumpSpeed(float newSpeed)
		{
			JumpSpeed = newSpeed;
		}
    }
}
