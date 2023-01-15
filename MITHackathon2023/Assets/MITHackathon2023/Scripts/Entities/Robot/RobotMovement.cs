using UnityEngine;

namespace MITHack.Robot.Entities
{
    public class RobotMovement : MonoBehaviour
    {
        [Header("Move Variables")] 
        [SerializeField, Min(0.0f)]
        private float initialMoveSpeed = 1.0f;
        [Space] 
        [SerializeField] 
        private bool enableGravity = false;

        [Header("References")] 
        [SerializeField]
        private CharacterController characterController;
        
        private float _movementSpeed = 0.0f;
        private Vector2 _movementDirection = Vector2.zero;

        public Vector3 MovementDirection
        {
            get
            {
                var movementForward = TargetForward * _movementDirection.y
                                      + TargetRight * _movementDirection.x;
                movementForward.Normalize();
                return movementForward;
            }
        }

        public Vector3 Velocity => MovementDirection * MoveSpeed;

        public Vector3 TargetForward => transform.forward;

        public Vector3 TargetRight => transform.right;

        public float MoveSpeed => _movementSpeed;

        private void Awake()
        {
            SetMoveSpeed(initialMoveSpeed);
        }

        private void Update()
        {
            Move(Velocity * Time.deltaTime);
        }

        public void SetMoveSpeed(float moveSpeed)
        {
            _movementSpeed = moveSpeed;
        }

        public void SetDirection(Vector2 direction)
        {
            _movementDirection = direction;
        }

        private void Move(Vector3 deltaPosition)
        {
            if (characterController)
            {
                if (enableGravity)
                {
                    characterController.SimpleMove(deltaPosition);
                }
                else
                {
                    characterController.Move(deltaPosition);
                }
                return;
            }
            var cachedTransform = transform;
            cachedTransform.position += deltaPosition;
        }
    }
}