using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using  UnityEditor;
#endif

public class FitCurveSample : MonoBehaviour
{
    [SerializeField] private List<Vector3> points;
    [SerializeField] private float maxError = 15;
    private List<Vector3> bezierPoints;
    private void OnValidate()
    {
        GenerateBezier();
    }

    void GenerateBezier()
    {
        if (points.Count >= 2)
        {
            bezierPoints = FitCurveUtil.FitCurve(points, maxError, false);
        }
    }

    private void OnDrawGizmos()
    {
        for (int i = 0; i < (points.Count-1); i++)
        {
            Gizmos.color = Color.white;
            Gizmos.DrawWireCube(points[i], Vector3.one*0.1f);
            Gizmos.color = Color.gray;
            Gizmos.DrawLine(points[i], points[i+1]);
        }
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(points[points.Count-1], Vector3.one*0.1f);

#if UNITY_EDITOR
        if (bezierPoints != null && bezierPoints.Count >= 4)
        {
            for (int i = 0; i < bezierPoints.Count; i += 4)
            {
                Handles.DrawBezier(bezierPoints[i], bezierPoints[i + 3], bezierPoints[i + 1], bezierPoints[i + 2],
                    Color.cyan, null, 1);
            }
        }
#endif
    }
}
