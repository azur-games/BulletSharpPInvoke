using BulletSharp.Math;

namespace BulletSharp
{
    public class KinematicCharacterController : ICharacterController
    {
        protected Vector3 ComputeReflectionDirection(ref Vector3 direction, ref Vector3 normal)
        {
            float dot;
            Vector3.Dot(ref direction, ref normal, out dot);
            return direction - (2.0f * dot) * normal;
        }

        protected Vector3 ParallelComponent(ref Vector3 direction, ref Vector3 normal)
        {
            float magnitude;
            Vector3.Dot(ref direction, ref normal, out magnitude);
            return normal * magnitude;
        }

        protected Vector3 PerpindicularComponent(ref Vector3 direction, ref Vector3 normal)
        {
            return direction - ParallelComponent(ref direction, ref normal);
        }

        protected bool RecoverFromPenetration(CollisionWorld collisionWorld)
        {
            Vector3 minAabb, maxAabb;
            m_convexShape.GetAabb(m_ghostObject.WorldTransform, out minAabb, out maxAabb);
            collisionWorld.Broadphase.SetAabbRef(m_ghostObject.BroadphaseHandle,
                         ref minAabb,
                         ref maxAabb,
                         collisionWorld.Dispatcher);

            bool penetration = false;

            collisionWorld.Dispatcher.DispatchAllCollisionPairs(m_ghostObject.OverlappingPairCache, collisionWorld.DispatchInfo, collisionWorld.Dispatcher);

            m_currentPosition = m_ghostObject.WorldTransform.Origin;

            float maxPen = 0f;
            for (int i = 0; i < m_ghostObject.OverlappingPairCache.NumOverlappingPairs; i++)
            {
                m_manifoldArray.Clear();

                BroadphasePair collisionPair = m_ghostObject.OverlappingPairCache.OverlappingPairArray[i];

                CollisionObject obj0 = collisionPair.Proxy0.ClientObject as CollisionObject;
                CollisionObject obj1 = collisionPair.Proxy1.ClientObject as CollisionObject;

                if ((obj0 != null && !obj0.HasContactResponse) || (obj1 != null && !obj1.HasContactResponse))
                    continue;

                collisionPair.GetAllContactManifolds(m_manifoldArray);

                for (int j = 0; j < m_manifoldArray.Count; j++)
                {
                    var manifoldId = AlignedManifoldArray.btAlignedManifoldArray_at(m_manifoldArray._native, j);
                    var bodyId = PersistentManifold.btPersistentManifold_getBody0(manifoldId);
                    var numContacts = PersistentManifold.btPersistentManifold_getNumContacts(manifoldId);
                    
                    float directionSign = bodyId == m_ghostObject._native ? -1f : 1f;
                    for (int p = 0; p < numContacts; p++)
                    {
                        var manifoldPointId = PersistentManifold.btPersistentManifold_getContactPoint(manifoldId, p);
                        float dist = ManifoldPoint.btManifoldPoint_getDistance(manifoldPointId);
                        if (dist < 0.0f)
                        {
                            Vector3 normalWorldOnB;
                            ManifoldPoint.btManifoldPoint_getNormalWorldOnB(manifoldPointId, out normalWorldOnB);
                            if (dist < maxPen)
                            {
                                maxPen = dist;
                            }

                            var counterPenDir = normalWorldOnB * directionSign;;
                            m_currentPosition += counterPenDir * dist;
                            penetration = true;
                            if (counterPenDir.Dot(Vector3.UnitY) > 0)
                            {
                                m_verticalVelocity = 0;
                            }
                        }
                    }
                }
            }
            Matrix newTrans = m_ghostObject.WorldTransform;
            newTrans.Origin = m_currentPosition;
            m_ghostObject.WorldTransform = newTrans;
            return penetration;
        }

        protected void StepUp(CollisionWorld collisionWorld)
        {
            Matrix start, end;
            m_targetPosition = m_currentPosition + Vector3.UnitY * (m_stepHeight + (m_verticalOffset > 0.0f ? m_verticalOffset : 0.0f));

            start = Matrix.Translation(m_currentPosition + Vector3.UnitY * (m_convexShape.Margin + m_addedMargin));
            end = Matrix.Translation(m_targetPosition);

            _callback.Me = m_ghostObject;
            _callback.Up = -Vector3.UnitY;
            _callback.MinSlopeDot = 0.7071f;
            _callback.CollisionFilterGroup = GhostObject.BroadphaseHandle.CollisionFilterGroup;
            _callback.CollisionFilterMask = GhostObject.BroadphaseHandle.CollisionFilterMask;
            _callback.ClosestHitFraction = 1;
            if (m_useGhostObjectSweepTest)
            {
                
                m_ghostObject.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
            }
            else
            {
                collisionWorld.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, 0f);
            }

            if (_callback.HasHit)
            {
                if (Vector3.Dot(_callback.HitNormalWorld, Vector3.UnitY) > 0.0)
                {
                    m_currentStepOffset = m_stepHeight * _callback.ClosestHitFraction;
                    if (m_interpolateUp)
                    {
                        Vector3.Lerp(ref m_currentPosition, ref m_targetPosition, _callback.ClosestHitFraction, out m_currentPosition);
                    }
                    else
                    {
                        m_currentPosition = m_targetPosition;
                    }
                }
                m_verticalVelocity = 0.0f;
                m_verticalOffset = 0.0f;
            }
            else
            {
                m_currentStepOffset = m_stepHeight;
                m_currentPosition = m_targetPosition;
            }

        }
        protected void UpdateTargetPositionBasedOnCollision(ref Vector3 hitNormal, float tangentMag, float normalMag)
        {
            Vector3 movementDirection = m_targetPosition - m_currentPosition;
            float movementLength = movementDirection.Length;
            if (movementLength > MathUtil.SIMD_EPSILON)
            {
                movementDirection.Normalize();

                Vector3 reflectDir = ComputeReflectionDirection(ref movementDirection, ref hitNormal);
                reflectDir.Normalize();

                Vector3 parallelDir, perpindicularDir;

                parallelDir = ParallelComponent(ref reflectDir, ref hitNormal);
                perpindicularDir = PerpindicularComponent(ref reflectDir, ref hitNormal);

                m_targetPosition = m_currentPosition;
                if (normalMag != 0.0f)
                {
                    Vector3 perpComponent = perpindicularDir * (normalMag * movementLength);
                    m_targetPosition += perpComponent;
                }
            }
        }

        protected void StepForwardAndStrafe(CollisionWorld collisionWorld, ref Vector3 walkMove)
        {
            Matrix start = Matrix.Identity, end = Matrix.Identity;
            m_targetPosition = m_currentPosition + walkMove;

            float fraction = 1.0f;
            float distance2 = (m_currentPosition - m_targetPosition).LengthSquared;

            int maxIter = 10;

            while (fraction > 0.01f && maxIter-- > 0)
            {
                start.Origin = (m_currentPosition);
                end.Origin = (m_targetPosition);

                Vector3 sweepDirNegative = m_currentPosition - m_targetPosition;

                _callback.MinSlopeDot = 0;
                _callback.Up = sweepDirNegative;
                _callback.CollisionFilterGroup = GhostObject.BroadphaseHandle.CollisionFilterGroup;
                _callback.CollisionFilterMask = GhostObject.BroadphaseHandle.CollisionFilterMask;
                _callback.ClosestHitFraction = 1;
                float margin = m_convexShape.Margin;
                m_convexShape.Margin = margin + m_addedMargin;


                if (m_useGhostObjectSweepTest)
                {
                    m_ghostObject.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                }
                else
                {
                    collisionWorld.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                }

                m_convexShape.Margin = margin;


                fraction -= _callback.ClosestHitFraction;

                if (_callback.HasHit)
                {
                    Vector3 hitNormalWorld = _callback.HitNormalWorld;
                    UpdateTargetPositionBasedOnCollision(ref hitNormalWorld, 0f, 1f);
                    Vector3 currentDir = m_targetPosition - m_currentPosition;
                    distance2 = currentDir.LengthSquared;
                    if (distance2 > MathUtil.SIMD_EPSILON)
                    {
                        currentDir.Normalize();
                        /* See Quake2: "If velocity is against original velocity, stop ead to avoid tiny oscilations in sloping corners." */
                        Vector3.Dot(ref currentDir, ref m_normalizedDirection, out var dot);
                        if (dot <= 0.0f)
                        {
                            break;
                        }
                    }
                    else
                    {
                        break;
                    }
                }
                else
                {
                    m_currentPosition = m_targetPosition;
                }
            }

        }
        protected void StepDown(CollisionWorld collisionWorld, float dt)
        {
            Matrix start, end, end_double;
            bool runonce = false;

            Vector3 orig_position = m_targetPosition;

            float downVelocity = (m_verticalVelocity < 0.0f ? -m_verticalVelocity : 0.0f) * dt;
            if (downVelocity > 0.0 && downVelocity > m_fallSpeed
                && (m_wasOnGround || !m_wasJumping))
            {
                downVelocity = m_fallSpeed;
            }

            Vector3 step_drop = Vector3.UnitY * (m_currentStepOffset + downVelocity);
            m_targetPosition -= step_drop;

            _callback.MinSlopeDot = m_maxSlopeCosine;
            _callback.Up = Vector3.UnitY;
            _callback.CollisionFilterGroup = GhostObject.BroadphaseHandle.CollisionFilterGroup;
            _callback.CollisionFilterMask = GhostObject.BroadphaseHandle.CollisionFilterMask;
            bool hasFirstHit;
            float hitFraction;
            Vector3 pointWorld;
            
            while (true)
            {
                start = Matrix.Translation(m_currentPosition);
                end = Matrix.Translation(m_targetPosition);

                //set double test for 2x the step drop, to check for a large drop vs small drop
                end_double = Matrix.Translation(m_targetPosition - step_drop);
                
                bool has_hit_first = false;
                bool has_hit_second = false;
                if (m_useGhostObjectSweepTest)
                {
                    _callback.ClosestHitFraction = 1;
                    m_ghostObject.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                    pointWorld = _callback.HitPointWorld;
                    has_hit_first = _callback.HasHit;
                    hitFraction = _callback.ClosestHitFraction;
                    if (!has_hit_first)
                    {
                        _callback.ClosestHitFraction = 1;
                        //test a double fall height, to see if the character should interpolate it's fall (full) or not (partial)
                        m_ghostObject.ConvexSweepTest(m_convexShape, start, end_double, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                        has_hit_second = _callback.HasHit;
                    }
                }
                else
                {
                    _callback.ClosestHitFraction = 1;
                    // this works....
                    collisionWorld.ConvexSweepTestRef(m_convexShape, ref start, ref end, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                    pointWorld = _callback.HitPointWorld;
                    has_hit_first = _callback.HasHit;
                    hitFraction = _callback.ClosestHitFraction;
                    if (!has_hit_first)
                    {
                        _callback.ClosestHitFraction = 1;
                        //test a double fall height, to see if the character should interpolate it's fall (large) or not (small)
                        m_ghostObject.ConvexSweepTest(m_convexShape, start, end_double, _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);
                        has_hit_second = _callback.HasHit;
                    }
                }

                hasFirstHit = has_hit_first;
                float downVelocity2 = (m_verticalVelocity < 0.0f ? -m_verticalVelocity : 0.0f) * dt;

                bool has_hit;
                if (bounce_fix == true)
                    has_hit = has_hit_first || has_hit_second;
                else
                    has_hit = has_hit_second;

                if (downVelocity2 > 0.0f && downVelocity2 < m_stepHeight && has_hit == true && runonce == false
                            && (m_wasOnGround || !m_wasJumping))
                {
                    //redo the velocity calculation when falling a small amount, for fast stairs motion
                    //for larger falls, use the smoother/slower interpolated movement by not touching the target position

                    m_targetPosition = orig_position;
                    downVelocity = m_stepHeight;

                    Vector3 step_drop2 = Vector3.UnitY * (m_currentStepOffset + downVelocity);
                    m_targetPosition -= step_drop2;
                    runonce = true;
                    continue; //re-run previous tests
                }
                break;
            }
            
            if (hasFirstHit || runonce)
            {
                // we dropped a fraction of the height -> hit floor
                float fraction = (m_currentPosition.Y - pointWorld.Y) / 2;

                if (bounce_fix)
                {
                    if (full_drop)
                    {
                        Vector3.Lerp(ref m_currentPosition, ref m_targetPosition, hitFraction, out m_currentPosition);
                    }
                    else
                    {
                        //due to errors in the closestHitFraction variable when used with large polygons, calculate the hit fraction manually
                        Vector3.Lerp(ref m_currentPosition, ref m_targetPosition, fraction, out m_currentPosition);
                    }
                }
                else
                {
                    Vector3.Lerp(ref m_currentPosition, ref m_targetPosition, hitFraction, out m_currentPosition);
                }

                full_drop = false;

                m_verticalVelocity = 0.0f;
                m_verticalOffset = 0.0f;
                m_wasJumping = false;

            }
            else
            {
                // we dropped the full height
                full_drop = true;

                if (bounce_fix == true)
                {
                    downVelocity = (m_verticalVelocity < 0.0f ? -m_verticalVelocity : 0.0f) * dt;
                    if (downVelocity > m_fallSpeed && (m_wasOnGround || !m_wasJumping))
                    {
                        m_targetPosition += step_drop; //undo previous target change
                        downVelocity = m_fallSpeed;
                        step_drop = Vector3.UnitY * (m_currentStepOffset + downVelocity);
                        m_targetPosition -= step_drop;
                    }
                }
                m_currentPosition = m_targetPosition;
            }
        }

        public KinematicCharacterController(PairCachingGhostObject ghostObject, ConvexShape convexShape, float stepHeight, int upAxis = 1)
        {
            m_addedMargin = 0.02f;
            m_walkDirection = Vector3.Zero;
            m_useGhostObjectSweepTest = true;
            m_ghostObject = ghostObject;
            m_stepHeight = stepHeight;
            m_convexShape = convexShape;
            m_useWalkDirection = true;	// use walk direction by default, legacy behavior
            m_velocityTimeInterval = 0.0f;
            m_verticalVelocity = 0.0f;
            m_verticalOffset = 0.0f;
            Gravity = 9.8f * 3; // 3G acceleration.
            m_fallSpeed = 55.0f; // Terminal velocity of a sky diver in m/s.
            m_jumpSpeed = 10.0f; // ?
            m_wasOnGround = false;
            m_wasJumping = false;
            m_interpolateUp = true;
            MaxSlope = MathUtil.DegToRadians(45.0f);
            m_currentStepOffset = 0;
            full_drop = false;
            bounce_fix = false;
            _callback = new KinematicClosestNotMeConvexResultCallback(m_ghostObject, -Vector3.UnitY, 0.7071f);
        }

        private KinematicClosestNotMeConvexResultCallback _callback;
        ///btActionInterface interface
        public virtual void UpdateAction(CollisionWorld collisionWorld, float deltaTime)
        {
            if (!m_skipUpdate)
            {
                PreStep(collisionWorld);
                PlayerStep(collisionWorld, deltaTime);
            }
        }

        ///btActionInterface interface
        public void DebugDraw(IDebugDraw debugDrawer)
        {
        }

        public void SetUpInterpolate(bool v)
        {
        }

        public void SetSkipUpdate(bool skip)
        {
            m_skipUpdate = skip;
        }
        public virtual void SetWalkDirection(ref Vector3 walkDirection)
        {
            m_useWalkDirection = true;
            m_walkDirection = walkDirection;
            m_normalizedDirection = GetNormalizedVector(ref m_walkDirection);
        }

        public virtual void SetWalkDirection(Vector3 walkDirection)
        {
            SetWalkDirection(ref walkDirection);
        }

        public void SetVelocityForTimeInterval(ref Vector3 velocity, float timeInterval)
        {
            m_useWalkDirection = false;
            m_walkDirection = velocity;
            m_normalizedDirection = GetNormalizedVector(ref m_walkDirection);
            m_velocityTimeInterval = timeInterval;
        }

        public void SetVelocityForTimeInterval(Vector3 velocity, float timeInterval)
        {
            SetVelocityForTimeInterval(ref velocity, timeInterval);
        }

        public void Reset(CollisionWorld collisionWorld)
        {
            m_verticalVelocity = 0.0f;
            m_verticalOffset = 0.0f;
            m_wasOnGround = false;
            m_wasJumping = false;
            m_walkDirection = Vector3.Zero;
            m_velocityTimeInterval = 0.0f;

            //clear pair cache
            HashedOverlappingPairCache cache = m_ghostObject.OverlappingPairCache;
            while (cache.OverlappingPairArray.Count > 0)
            {
                cache.RemoveOverlappingPair(cache.OverlappingPairArray[0].Proxy0, cache.OverlappingPairArray[0].Proxy1, collisionWorld.Dispatcher);
            }
        }

        public void Warp(ref Vector3 origin)
        {
            m_ghostObject.WorldTransform = Matrix.Translation(origin);
        }

        public void PreStep(CollisionWorld collisionWorld)
        {
            int numPenetrationLoops = 0;
            while (RecoverFromPenetration(collisionWorld))
            {
                numPenetrationLoops++;
                if (numPenetrationLoops > 4)
                {
                    break;
                }
            }

            m_currentPosition = m_ghostObject.WorldTransform.Origin;
            m_targetPosition = m_currentPosition;

        }

        public void PlayerStep(CollisionWorld collisionWorld, float dt)
        {
            if (!m_useWalkDirection && m_velocityTimeInterval <= 0.0)
            {
                return;		// no motion
            }

            m_wasOnGround = OnGround;

            // Update fall velocity.
            m_verticalVelocity -= Gravity * dt;
            if (m_verticalVelocity > 0.0f && m_verticalVelocity > m_jumpSpeed)
            {
                m_verticalVelocity = m_jumpSpeed;
            }
            if (m_verticalVelocity < 0.0f && System.Math.Abs(m_verticalVelocity) > System.Math.Abs(m_fallSpeed))
            {
                m_verticalVelocity = -System.Math.Abs(m_fallSpeed);
            }
            m_verticalOffset = m_verticalVelocity * dt;


            Matrix xform = m_ghostObject.WorldTransform;

            StepUp(collisionWorld);
            if (m_useWalkDirection)
            {
                var delta = dt * m_walkDirection;
                StepForwardAndStrafe(collisionWorld, ref delta);
            }
            StepDown(collisionWorld, dt);

            xform.Origin = m_currentPosition;
            m_ghostObject.WorldTransform = xform;
        }

        public void SetFallSpeed(float fallSpeed)
        {
            m_fallSpeed = fallSpeed;
        }

        public void SetJumpSpeed(float jumpSpeed)
        {
            m_jumpSpeed = jumpSpeed;
        }

        public bool CanJump
        {
            get { return OnGround; }
        }

        public void Jump()
        {
            if (CanJump)
            {
                m_verticalVelocity = m_jumpSpeed;
                m_wasJumping = true;
            }

        }

        public float Gravity { get; set; }
        public float MaxSlope
        {
            get { return m_maxSlopeRadians; }
            set
            {
                m_maxSlopeRadians = value;
                m_maxSlopeCosine = (float)System.Math.Cos(value);
            }
        }

        public PairCachingGhostObject GhostObject
        {
            get { return m_ghostObject; }
        }

        public void SetUseGhostSweepTest(bool useGhostObjectSweepTest)
        {
            m_useGhostObjectSweepTest = useGhostObjectSweepTest;
        }

        public bool OnGround
        {
            get { return m_verticalVelocity == 0.0f && m_verticalOffset == 0.0f; }
        }

        public static Vector3 GetNormalizedVector(ref Vector3 v)
        {
            if (v.Length < MathUtil.SIMD_EPSILON)
            {
                return Vector3.Zero;
            }
            return Vector3.Normalize(v);
        }


        protected float m_halfHeight;

        protected PairCachingGhostObject m_ghostObject;
        protected ConvexShape m_convexShape;//is also in m_ghostObject, but it needs to be convex, so we store it here to avoid upcast

        protected float m_verticalVelocity;
        protected float m_verticalOffset;


        protected float m_fallSpeed;
        protected float m_jumpSpeed;
        protected float m_maxSlopeRadians; // Slope angle that is set (used for returning the exact value)
        protected float m_maxSlopeCosine;  // Cosine equivalent of m_maxSlopeRadians (calculated once when set, for optimization)
        protected float m_stepHeight;
        protected float m_addedMargin;//@todo: remove this and fix the code
        ///this is the desired walk direction, set by the user
        protected Vector3 m_walkDirection;
        protected Vector3 m_normalizedDirection;
        //some internal variables
        protected Vector3 m_currentPosition;
        float m_currentStepOffset;
        protected Vector3 m_targetPosition;
        ///keep track of the contact manifolds
        protected AlignedManifoldArray m_manifoldArray = new AlignedManifoldArray();
        protected bool m_wasOnGround;
        protected bool m_wasJumping;
        protected bool m_useGhostObjectSweepTest;
        protected bool m_useWalkDirection;
        protected float m_velocityTimeInterval;
        protected bool m_interpolateUp;
        protected bool full_drop;
        protected bool bounce_fix;
        protected bool m_skipUpdate = false;
    }
    
    public class KinematicClosestNotMeConvexResultCallback : ClosestConvexResultCallback
    {
        static Vector3 zero = new Vector3();

        public KinematicClosestNotMeConvexResultCallback(CollisionObject me, Vector3 up, float minSlopeDot)
            : base(ref zero, ref zero)
        {
            Me = me;
            Up = up;
            MinSlopeDot = minSlopeDot;
        }

        public override float AddSingleResult(LocalConvexResult convexResult, bool normalInWorldSpace)
        {
            if (convexResult.HitCollisionObject == Me)
            {
                return 1.0f;
            }

            if (!convexResult.HitCollisionObject.HasContactResponse)
            {
                return 1.0f;
            }

            Vector3 hitNormalWorld;
            if (normalInWorldSpace)
            {
                hitNormalWorld = convexResult.HitNormalLocal;
            }
            else
            {
                // need to transform normal into worldspace
                hitNormalWorld = Vector3.TransformCoordinate(convexResult.HitNormalLocal, convexResult.HitCollisionObject.WorldTransform.Basis);
            }

            float dotUp;
            Vector3.Dot(ref Up, ref hitNormalWorld, out dotUp);
            if (dotUp < MinSlopeDot)
            {
                return 1.0f;
            }

            return base.AddSingleResult(convexResult, normalInWorldSpace);
        }

        public CollisionObject Me { get; set; }
        public Vector3 Up;
        public float MinSlopeDot { get; set; }
    }
}
