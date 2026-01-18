using UnityEngine;
using System.Collections.Generic;
using System.IO; // Added for File I/O
using System.Text; // Added for StringBuilder

public class Comparer : MonoBehaviour
{
    private LongitudinalFlight flight;
    
    [Header("UI Settings")]
    public int historyLength = 200;
    public Vector2 graphSize = new Vector2(250, 100);

    // Storage for the data
    private Dictionary<string, List<float[]>> dataHistory = new Dictionary<string, List<float[]>>();
    private string[] labels = { "U_FwdVel", "W_VertVel", "Q_PitchRate", "Theta_Pitch", "PosX", "PosZ" };
    private Color trueColor = Color.green;
    private Color sensorColor = Color.red;
    private Color estimateColor = Color.cyan;

    // List to store ALL data for the final file export (not just the history buffer)
    private List<string> csvRows = new List<string>();

    void Start() {
        flight = GetComponent<LongitudinalFlight>();
        foreach (var label in labels) dataHistory[label] = new List<float[]>();

        // Add CSV Header
        string header = "Timestamp,";
        foreach (var label in labels) {
            header += $"{label}_True,{label}_Sensor,{label}_Estimate,";
        }
        csvRows.Add(header.TrimEnd(','));
    }

    void FixedUpdate() {
        float[] sensor = flight.z;
        float[] estimate = { flight.u, flight.w, flight.q, flight.theta, flight.posX, flight.posZ };
        float[] actual = { flight.trueU, flight.trueW, flight.trueQ, flight.trueTheta, transform.position.z, -transform.position.y };

        // 1. Update the UI Graph History
        for (int i = 0; i < 6; i++) {
            float[] frameData = { actual[i], sensor[i], estimate[i] };
            dataHistory[labels[i]].Add(frameData);
            if (dataHistory[labels[i]].Count > historyLength) dataHistory[labels[i]].RemoveAt(0);
        }

        // 2. Log data for CSV export
        StringBuilder sb = new StringBuilder();
        sb.Append(Time.time.ToString("F4") + ",");
        for (int i = 0; i < 6; i++) {
            sb.Append($"{actual[i]},{sensor[i]},{estimate[i]},");
        }
        csvRows.Add(sb.ToString().TrimEnd(','));
    }

    void OnGUI() {
        GUI.color = Color.white;
        GUILayout.BeginArea(new Rect(10, 10, 600, 800));
        
        GUILayout.Label("EKF PERFORMANCE: GREEN (True) | RED (Sensor) | CYAN (EKF Estimate)");

        // Export Button
        if (GUILayout.Button("SAVE DATA TO CSV", GUILayout.Width(200), GUILayout.Height(30))) {
            SaveDataToCSV();
        }

        for (int i = 0; i < 6; i++) {
            DrawGraph(labels[i], dataHistory[labels[i]], i);
        }
        GUILayout.EndArea();
    }

    public void SaveDataToCSV() {
        string filePath = Path.Combine(Application.dataPath, "FlightDataLog.csv");
        
        try {
            File.WriteAllLines(filePath, csvRows);
            Debug.Log($"<color=green>Data successfully saved to: {filePath}</color>");
        } catch (System.Exception e) {
            Debug.LogError($"Failed to save CSV: {e.Message}");
        }
    }

    void DrawGraph(string label, List<float[]> history, int index) {
        if (history.Count < 2) return;

        Rect rect = new Rect(0, 30 + (index * (graphSize.y + 20)), graphSize.x, graphSize.y);
        GUI.Box(rect, label);

        // Simple auto-scaling logic
        float min = float.MaxValue, max = float.MinValue;
        foreach (var val in history) {
            foreach (var v in val) {
                if (v < min) min = v;
                if (v > max) max = v;
            }
        }
        float range = Mathf.Max(0.1f, max - min);

        for (int s = 0; s < 3; s++) { // 0: True, 1: Sensor, 2: Estimate
            Color c = (s == 0) ? trueColor : (s == 1) ? sensorColor : estimateColor;
            for (int i = 0; i < history.Count - 1; i++) {
                Vector2 p1 = new Vector2(
                    rect.x + (float)i / historyLength * rect.width,
                    rect.yMax - ((history[i][s] - min) / range) * rect.height
                );
                Vector2 p2 = new Vector2(
                    rect.x + (float)(i + 1) / historyLength * rect.width,
                    rect.yMax - ((history[i + 1][s] - min) / range) * rect.height
                );
                Drawing.DrawLine(p1, p2, c, 1.5f);
            }
        }
    }
}

// Helper class to draw lines in OnGUI
public static class Drawing {
    private static Texture2D lineTex;
    public static void DrawLine(Vector2 p1, Vector2 p2, Color color, float width) {
        if (!lineTex) { lineTex = new Texture2D(1, 1); }
        Matrix4x4 matrix = GUI.matrix;
        float angle = Mathf.Atan2(p2.y - p1.y, p2.x - p1.x) * Mathf.Rad2Deg;
        GUI.color = color;
        GUIUtility.RotateAroundPivot(angle, p1);
        GUI.DrawTexture(new Rect(p1.x, p1.y, (p2 - p1).magnitude, width), lineTex);
        GUI.matrix = matrix;
    }
}