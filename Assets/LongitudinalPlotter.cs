using UnityEngine;
using UnityEngine.UI;
using System.Collections.Generic;

public class LongitudinalPlotter : MonoBehaviour
{
    [Header("References")]
    public Transform aircraft;
    public LineRenderer pathLine;
    public RectTransform graphArea;
    public Text altitudeText;
    public Text distanceText;

    [Header("Graph Scaling")]
    public float pixelsPerMeter = 2.0f; // Scale: how many UI pixels represent 1 meter
    public int maxPoints = 1000;

    private List<Vector2> flightData = new List<Vector2>();
    private Vector3 startPos;

    void Start()
    {
        startPos = aircraft.position;
        pathLine.useWorldSpace = false;
        pathLine.positionCount = 0;
    }

    void Update()
    {
        // 1. Calculate longitudinal coordinates
        // X = Distance traveled from start, Y = Current Altitude
        float xDist = Vector3.Distance(new Vector3(aircraft.position.x, 0, aircraft.position.z), 
                                       new Vector3(startPos.x, 0, startPos.z));
        float yAlt = aircraft.position.y;

        Vector2 currentPoint = new Vector2(xDist, yAlt);

        // 2. Only add point if the aircraft has moved significantly (e.g., 0.5 meters)
        if (flightData.Count == 0 || Vector2.Distance(flightData[flightData.Count - 1], currentPoint) > 0.5f)
        {
            flightData.Add(currentPoint);
            if (flightData.Count > maxPoints) flightData.RemoveAt(0);
            DrawGraph();
        }

        // 3. Update UI Labels
        if (altitudeText) altitudeText.text = $"Alt: {yAlt:F1} m";
        if (distanceText) distanceText.text = $"Dist: {xDist:F1} m";
    }

    void DrawGraph()
    {
        pathLine.positionCount = flightData.Count;
        
        // We want the graph to "follow" the aircraft, so we offset by the latest point
        Vector2 offset = flightData[flightData.Count - 1];

        for (int i = 0; i < flightData.Count; i++)
        {
            // Calculate position relative to the current aircraft position
            // This keeps the "head" of the line at the center or right of your UI box
            float relativeX = (flightData[i].x - offset.x) * pixelsPerMeter;
            float relativeY = (flightData[i].y - offset.y) * pixelsPerMeter;

            pathLine.SetPosition(i, new Vector3(relativeX, relativeY, 0));
        }
    }
}
