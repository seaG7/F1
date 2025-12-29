using UnityEngine;

public class WheelFix : MonoBehaviour
{
    private void Update()
    {
        transform.localEulerAngles = Vector3.zero;
    }
}