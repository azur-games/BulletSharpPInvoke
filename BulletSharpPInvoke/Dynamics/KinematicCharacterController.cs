using BulletSharp.Math;
using UnityEngine;
using Vector3 = BulletSharp.Math.Vector3;

namespace BulletSharp
{
    public class KinematicCharacterController : ICharacterController
    {
        public KinematicCharacterController(PairCachingGhostObject ghostObject, ConvexShape convexShape, float stepHeight, int upAxis = 1)
        {
            m_upAxis = upAxis;
            m_walkDirection = Vector3.Zero;
            m_ghostObject = ghostObject;
            m_stepHeight = stepHeight;
            m_convexShape = convexShape;
            Gravity = 9.8f * 3; // 3G acceleration.
            m_jumpSpeed = 10.0f; // ?
            m_onGround = false;
            MaxSlope = MathUtil.DegToRadians(45.0f);
            m_currentStepOffset = 0;
            _callback = new KinematicClosestNotMeConvexResultCallback(m_ghostObject, Vector3.UnitY, 0.7071f);
        }

        private KinematicClosestNotMeConvexResultCallback _callback;
        public virtual void UpdateAction(CollisionWorld collisionWorld, float deltaTime)
        {
            PreStep(collisionWorld);
            PlayerStep(collisionWorld, deltaTime);
        }
        
        public void PreStep(CollisionWorld collisionWorld)
        {
            m_currentPosition = m_ghostObject.WorldTransform.Origin;
            m_wasOnGround = false;
            //move a bit down
            //recover from penetration, try to find ground support
            int numPenetrationLoops = 0; 
            while(RecoverFromPenetration(collisionWorld))
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
            Matrix start = Matrix.Identity;
            Matrix end = Matrix.Identity;
            
            float fraction = 1.0f;
            
            if (!m_wasOnGround || m_verticalVelocity > 0)
            {
                m_currentPosition = m_currentPosition + Vector3.UnitY * m_verticalVelocity * dt;
                m_verticalVelocity -= Gravity * dt;
            }
            else
            {
                m_verticalVelocity = 0;
            }
            
            if (m_walkDirection.LengthSquared > 0.01f)
            {
                m_targetPosition = m_currentPosition + m_walkDirection * dt * fraction;
                
                int maxIter = 10;
                start.Origin = (m_currentPosition);
                end.Origin = (m_targetPosition);
                
                while (fraction > 0.01f && maxIter-- > 0)
                {
                    Vector3 sweepDirNegative = m_currentPosition - m_targetPosition;
                    
                    _callback.MinSlopeDot = 0;
                    _callback.Up = sweepDirNegative;
                    _callback.CollisionFilterGroup = GhostObject.BroadphaseHandle.CollisionFilterGroup;
                    _callback.CollisionFilterMask = GhostObject.BroadphaseHandle.CollisionFilterMask;
                    _callback.ClosestHitFraction = 1;
                    float margin = m_convexShape.Margin;
                    m_convexShape.Margin = margin;
                    m_ghostObject.ConvexSweepTestRef(m_convexShape, ref start, ref end, 
                        _callback, collisionWorld.DispatchInfo.AllowedCcdPenetration);

                    m_convexShape.Margin = margin;
                    fraction -= _callback.ClosestHitFraction;
                    
                    if (_callback.HasHit)
                    {
                        Vector3 hitNormalWorld = _callback.HitNormalWorld;
                        
                        UpdateTargetPositionBasedOnCollision(ref hitNormalWorld);
                        
                        Vector3 currentDir = m_targetPosition - m_currentPosition;
                        
                        if (currentDir.LengthSquared > MathUtil.SIMD_EPSILON)
                        {
                            currentDir.Normalize();
                            float dot;
                            Vector3.Dot(ref currentDir, ref m_walkDirection, out dot);
                            if (dot <= 0.0f)
                            {
                                break;
                            }
                        }
                        else
                        {
                            break;
                        }
                        
                        var newPos = start.Origin + (end.Origin - start.Origin) * _callback.ClosestHitFraction;
                        start = Matrix.Translation(newPos);
                        end = Matrix.Translation(m_targetPosition);
                    }
                    else
                    {
                        // we moved whole way
                        m_currentPosition = m_targetPosition;
                    }
                }
            }
            
            m_ghostObject.WorldTransform = Matrix.Translation(m_currentPosition);
        }
        
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
        
        protected void UpdateTargetPositionBasedOnCollision(ref Vector3 hitNormal)
        {
            Vector3 movementDirection = m_targetPosition - m_currentPosition;
            float movementLength = movementDirection.Length;
            
            if (movementLength > MathUtil.SIMD_EPSILON)
            {
                movementDirection.Normalize();
                Vector3 reflectDir = ComputeReflectionDirection(ref movementDirection, ref hitNormal);
                reflectDir.Normalize();
                Vector3 perpindicularDir;
                perpindicularDir = PerpindicularComponent(ref reflectDir, ref hitNormal);
                m_targetPosition = m_currentPosition;
                Vector3 perpComponent = perpindicularDir * movementLength;
                perpComponent.Y = 0;
                m_targetPosition += perpComponent;
            }
        }
        
        /*
        protected void StepForwardAndStrafe(CollisionWorld collisionWorld, ref Vector3 walkMove)
        {
            m_targetPosition = m_currentPosition + walkMove;
            float distance2 = (m_currentPosition - m_targetPosition).LengthSquared;

            if (m_touchingContact)
            {
                float dot;
                Vector3.Dot(ref m_normalizedDirection, ref m_touchingNormal, out dot);
                if (dot > 0.0f)
                {
                    //interferes with step movement
                    //UpdateTargetPositionBasedOnCollision(ref m_touchingNormal, 0.0f, 1.0f);
                }
            }

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
            }
        }*/
        
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
                        
                        Vector3 normalWorldOnB;
                        ManifoldPoint.btManifoldPoint_getNormalWorldOnB(manifoldPointId, out normalWorldOnB);
                        var resultNormal = normalWorldOnB * directionSign;
                        
                        m_touchingNormal = resultNormal;
                        m_wasOnGround = true;

                        if (dist < 0.0f)
                        {
                            if (dist < maxPen)
                            {
                                maxPen = dist;
                            }
                            
                            m_currentPosition += resultNormal * dist * 0.2f;
                            penetration = true;
                        }
                    }
                }
            }
            Matrix newTrans = m_ghostObject.WorldTransform;
            newTrans.Origin = m_currentPosition;
            m_ghostObject.WorldTransform = newTrans;
            return penetration;
        }
        
        public void DebugDraw(IDebugDraw debugDrawer)
        {
        }
        public virtual void SetWalkDirection(Vector3 walkDirection)
        {
            m_walkDirection = walkDirection;
        }

        public void Reset(CollisionWorld collisionWorld)
        {
            m_verticalVelocity = 0.0f;
            m_verticalOffset = 0.0f;
            m_wasOnGround = false;
            m_walkDirection = Vector3.Zero;

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

        public bool CanJump
        {
            get { return OnGround; }
        }

        public void Jump()
        {
            if (CanJump)
            {
                m_verticalVelocity = m_jumpSpeed;
            }
        }

        public float Gravity { get; set; }

        public float MaxSlope
        {
            get { return m_maxSlopeRadians; }
            set
            {
                m_maxSlopeRadians = value;
            }
        }

        public PairCachingGhostObject GhostObject
        {
            get { return m_ghostObject; }
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
        
        protected AlignedManifoldArray m_manifoldArray = new AlignedManifoldArray();
        protected PairCachingGhostObject m_ghostObject;
        protected float m_verticalVelocity;
        protected float m_verticalOffset;
        protected float m_jumpSpeed;
        protected float m_maxSlopeRadians;
        protected Vector3 m_walkDirection;
        protected Vector3 m_touchingNormal;
        float m_currentStepOffset;
        protected bool m_wasOnGround;
        protected int m_upAxis;
        private float m_stepHeight;
        private ConvexShape m_convexShape;
        private bool m_onGround;
        protected Vector3 m_currentPosition;
        protected Vector3 m_targetPosition;
    }

    public class KinematicClosestNotMeRayResultCallback : ClosestRayResultCallback
    {
        static Vector3 zero = new Vector3();

        public KinematicClosestNotMeRayResultCallback(CollisionObject me)
            : base(ref zero, ref zero)
        {
            _me = me;
        }

        public override float AddSingleResult(LocalRayResult rayResult, bool normalInWorldSpace)
        {
            if (rayResult.CollisionObject == _me)
                return 1.0f;

            return base.AddSingleResult(rayResult, normalInWorldSpace);
        }

        protected CollisionObject _me;
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
