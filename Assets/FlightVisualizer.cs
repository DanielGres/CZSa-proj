using UnityEngine;
using System.Collections.Generic;

public class FlightVisualizer : MonoBehaviour
{
    private LongitudinalFlight flightScript;

    [Header("Settings")]
    public bool showTruePath = true;
    public bool showSensorGhost = true;
    public bool showEstimatePath = true;

    private List<Vector3> truePath = new List<Vector3>();
    private List<Vector3> estimatePath = new List<Vector3>();
    private Vector3 currentNoisyPos;

    void Start() {
        flightScript = GetComponent<LongitudinalFlight>();
    }

    void Update() {
        if (truePath.Count > 500) truePath.RemoveAt(0);
        truePath.Add(transform.position);

        if (estimatePath.Count > 500) estimatePath.RemoveAt(0);

        // change this to show predicted position based also on velocities
        Vector3 ekfWorldPos = new Vector3(0, -flightScript.posZ, flightScript.posX);
        
        estimatePath.Add(ekfWorldPos);
    }

    void OnDrawGizmos() {
        if (flightScript == null) return;

        // 1. DRAW TRUE PATH (White)
        if (showTruePath) {
            Gizmos.color = Color.white;
            for (int i = 1; i < truePath.Count; i++) {
                Gizmos.DrawLine(truePath[i - 1], truePath[i]);
            }
        }

        // 2. DRAW EKF ESTIMATE (Green) - The "Cleaned" version
        if (showEstimatePath) {
            Gizmos.color = Color.green;
            for (int i = 1; i < estimatePath.Count; i++) {
                Gizmos.DrawLine(estimatePath[i - 1], estimatePath[i]);
            }
        }

        // 3. DRAW SENSOR NOISE (Red) - This will flicker wildly
        if (showSensorGhost) {
            float[] noisy = flightScript.GetNoisySensorData();
            // Project the noisy 'u' and 'w' into a temporary visual jitter
            Vector3 noiseOffset = new Vector3(Random.Range(-1, 1), noisy[1] - flightScript.trueW, noisy[0] - flightScript.trueU);
            Gizmos.color = Color.red;
            Gizmos.DrawWireCube(transform.position + noiseOffset, Vector3.one * 0.5f);
            Gizmos.DrawRay(transform.position, noiseOffset * 2f);
        }
    }
}