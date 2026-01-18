using UnityEngine;

public class MoveCam : MonoBehaviour
{
    [Header("Targeting")]
    public Transform target; // Drag your Airplane here
    public Vector3 offset = new Vector3(15, 0, 0); // Distance behind/above

    [Header("Smoothing")]
    public float smoothSpeed = 0.125f; // Lower = smoother follow

    void LateUpdate()
    {
        if (target == null) return;

        // 1. Calculate the desired position based on the offset
        // We add the offset to the target's position, ignoring target's rotation
        Vector3 desiredPosition = target.position + offset;

        // 2. Smoothly interpolate from current position to desired position
        Vector3 smoothedPosition = Vector3.Lerp(transform.position, desiredPosition, smoothSpeed);

        // 3. Apply the position
        transform.position = smoothedPosition;

    }
}
