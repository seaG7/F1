using UnityEngine;

public class WheelDistance : MonoBehaviour
{
    [SerializeField] private LayerMask groundLayer;
    [HideInInspector] public float distanceToGround = 0.0f;

    private void Update()
    {
        Physics.Raycast(transform.position, -transform.up, out RaycastHit hitInfo, 100, groundLayer);
        Debug.DrawRay(transform.position, -transform.up * KartController.Instance.groundRayLength, Color.magenta);

        distanceToGround = hitInfo.distance;
    }
}