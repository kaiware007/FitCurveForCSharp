using System.Collections.Generic;
using System.Linq;
using UnityEngine;

/// <summary>
/// FitCurve for C#
/// Base : https://github.com/soswow/fit-curve by soswow
/// </summary>
public static class FitCurveUtil
{
    /// <summary>
    /// FitCurve
    /// </summary>
    /// <param name="points">Curve of points</param>
    /// <param name="maxError">Maximum error distance</param>
    /// <param name="isMergeLastFirst">Merge end and start control points for multiple Beziers</param>
    /// <returns></returns>
    public static List<Vector3> FitCurve(List<Vector3> points, float maxError, bool isMergeLastFirst = true)
    {
        points = points.Distinct().ToList();

        if (points.Count < 2)
        {
            return null;
        }

        Vector3 leftTangent = (points[1] - points[0]).normalized;
        Vector3 rightTangent = (points[points.Count - 2] - points[points.Count - 1]).normalized;

        return FitCubic(points, leftTangent, rightTangent, maxError, isMergeLastFirst);
    }

    static List<Vector3> FitCubic(List<Vector3> points, Vector3 leftTangent, Vector3 rightTangent, float error, bool isMergeLastFirst)
    {
        const int maxIterations = 20;

        List<Vector3> outputs = new List<Vector3>();
        
        if (points.Count == 2)
        {
            var dist = Vector3.Distance(points[0], points[1]) / 3f;
            outputs.Add(points[0]);
            outputs.Add(points[0] + leftTangent * dist);
            outputs.Add(points[1] + rightTangent * dist);
            outputs.Add(points[1]);
            return outputs;
        }

        var u = ChordLengthParameterize(points);

        var bezCurve = GenerateBezier(points, u, leftTangent, rightTangent);

        float maxError;
        int splitPoint;
        ComputeMaxError(points, bezCurve, u, out maxError, out splitPoint);

        
        if (maxError == 0f || maxError < error)
        {
            return bezCurve;
        }

        if (maxError < error * error)
        {
            var uPrime = u;
            var prevErr = maxError;
            var prevSplit = splitPoint;

            for (int i = 0; i < maxIterations; i++)
            {
                uPrime = Reparameterize(bezCurve, points, uPrime);
                
                bezCurve = GenerateBezier(points, uPrime, leftTangent, rightTangent);
                ComputeMaxError(points, bezCurve, u, out maxError, out splitPoint);
                
                if (maxError < error)
                {
                    return bezCurve;
                } else if (splitPoint == prevSplit)
                {
                    //If the development of the fitted curve grinds to a halt,
                    //we abort this attempt (and try a shorter curve):
                    var errChange = maxError / prevErr;
                    if ((errChange > 0.9999f) && (errChange < 1.0001f))
                    {
                        break;
                    }
                }

                prevErr = maxError;
                prevSplit = splitPoint;
            }
        }
        
        //Fitting failed -- split at max error point and fit recursively
        List<Vector3> beziers = new List<Vector3>();
        
        //To create a smooth transition from one curve segment to the next, we
        //calculate the line between the points directly before and after the
        //center, and use that as the tangent both to and from the center point.
        var centerVector = points[splitPoint - 1] - points[splitPoint + 1];
        //However, this won't work if they're the same point, because the line we
        //want to use as a tangent would be 0. Instead, we calculate the line from
        //that "double-point" to the center point, and use its tangent.
        if (centerVector.x == 0f && centerVector.y == 0f && centerVector.z == 0f)
        {
            //[x,y] -> [-y,x]: http://stackoverflow.com/a/4780141/1869660
            centerVector = points[splitPoint - 1] - points[splitPoint];
            centerVector = new Vector3(-centerVector.y, centerVector.x, centerVector.z);
        }

        var toCenterTangent = centerVector.normalized;
        var fromCenterTangent = -toCenterTangent;

        var points1 = points.GetRange(0, splitPoint+1);
        var points2 = points.GetRange(splitPoint, points.Count - splitPoint);
        beziers.AddRange(FitCubic(points1, leftTangent, toCenterTangent, error, isMergeLastFirst));
        if (isMergeLastFirst)
        {
            var back = FitCubic(points2, fromCenterTangent, rightTangent, error, isMergeLastFirst);
            back.RemoveAt(0);
            beziers.AddRange(back);
        }
        else
        {
            beziers.AddRange(FitCubic(points2, fromCenterTangent, rightTangent, error, isMergeLastFirst));
        }

        return beziers;
    }

    static List<Vector3> GenerateBezier(List<Vector3> points, List<float> parameters, Vector3 leftTangent, Vector3 rightTangent)
    {
        var firstPoint = points[0];
        var lastPoint = points[points.Count - 1];

        var outputs = new List<Vector3> {firstPoint, Vector3.zero, Vector3.zero, lastPoint};
        
        // Compute the A's
        var A = new Vector3[parameters.Count, 2];
        for (int i = 0; i < parameters.Count; i++)
        {
            var u = parameters[i];
            var ux = 1f - u;
            A[i, 0] = leftTangent * 3f * u * (ux * ux);
            A[i, 1] = rightTangent * 3f * ux * (u * u);
        }
        
        // Create the C and X matrices
        var C = new float[2, 2];
        var X = new float[2];
        for (int i = 0; i < points.Count; i++)
        {
            var u = parameters[i];
            C[0, 0] += Vector3.Dot(A[i,0], A[i,0]);
            C[0, 1] += Vector3.Dot(A[i,0], A[i,1]);
            C[1, 0] = C[0, 1];
            C[1, 1] += Vector3.Dot(A[i,1], A[i,1]);

            var tmp = points[i] - BezierQ(firstPoint, firstPoint, lastPoint, lastPoint, u);

            X[0] += Vector3.Dot(A[i, 0], tmp);
            X[1] += Vector3.Dot(A[i, 1], tmp);
        }
        
        // Compute ther determinants of C and X
        var det_C0_C1 = (C[0, 0] * C[1, 1]) - (C[1, 0] * C[0, 1]);
        var det_C0_X  = (C[0, 0] * X[1]) - (C[1, 0] * X[0]);
        var det_X_C1  = (X[0] * C[1, 1]) - (X[1] * C[0, 1]);
        
        // Finally, derive alpha values
        var alpha_l = det_C0_C1 == 0f ? 0f : det_X_C1 / det_C0_C1;
        var alpha_r = det_C0_C1 == 0f ? 0f : det_C0_X / det_C0_C1;
        
        // If alpha negative, use the Wu/Barsky heuristic (see text).
        // If alpha is 0, you get coincident control points that lead to
        // divide by zero in any subsequent NewtonRaphsonRootFind() call.
        var segLength = Vector3.Distance(firstPoint, lastPoint);
        var epsilon = (float)1e-06 * segLength;
        if (alpha_l < epsilon || alpha_r < epsilon)
        {
            // Fall back on standard (probably inaccurate) formula, and subdivide further if needed.
            outputs[1] = firstPoint + leftTangent * segLength / 3f;
            outputs[2] = lastPoint + rightTangent * segLength / 3f;
        }
        else
        {
            // First and last control points of the Bezier curve are
            // positioned exactly at the first and last data points
            // Control points 1 and 2 are positioned an alpha distance out
            // on the tangent vectors, left and right, respectively
            outputs[1] = firstPoint + leftTangent * alpha_l;
            outputs[2] = lastPoint + rightTangent * alpha_r;
        }

        return outputs;
    }

    static List<float> Reparameterize(List<Vector3> bez, List<Vector3> points, List<float> parameters)
    {
        return parameters.Select((p, i) => NewtonRaphsonRootFind(bez, points[i], p)).ToList();
    }

    static float NewtonRaphsonRootFind(List<Vector3> bez, Vector3 point, float u)
    {
        /*
        Newton's root finding algorithm calculates f(x)=0 by reiterating
        x_n+1 = x_n - f(x_n)/f'(x_n)
        We are trying to find curve parameter u for some point p that minimizes
        the distance from that point to the curve. Distance point to curve is d=q(u)-p.
        At minimum distance the point is perpendicular to the curve.
        We are solving
        f = q(u)-p * q'(u) = 0
        with
        f' = q'(u) * q'(u) + q(u)-p * q''(u)
        gives
        u_n+1 = u_n - |q(u_n)-p * q'(u_n)| / |q'(u_n)**2 + q(u_n)-p * q''(u_n)|
    */
        var d = BezierQ(bez[0], bez[1], bez[2], bez[3], u) - point;
        var qprime = BezierQPrime(bez[0], bez[1], bez[2], bez[3], u);
        var qprime2 = BezierQPrimeQPrime(bez[0], bez[1], bez[2], bez[3], u);
        var numerator = d.x * qprime.x + d.y * qprime.y;
        var sqprime = Vector3.Scale(qprime, qprime);
        var denominator = (sqprime.x + sqprime.y + sqprime.z) + 2f * (d.x * qprime2.x + d.y * qprime2.y);

        if (denominator == 0f)
        {
            return u;
        }
        else
        {
            return u - (numerator / denominator);
        }
    }
    
    static List<float> ChordLengthParameterize(List<Vector3> points)
    {
        float prevU = 0;
        Vector3 prevP = Vector3.zero;
        var u = new List<float>();
        
        for (int i = 0; i < points.Count; i++)
        {
            float currU = i != 0 ? prevU + Vector3.Distance(points[i], prevP) : 0;
            u.Add(currU);

            prevU = currU;
            prevP = points[i];
        }
        
        return u.Select(x => x / prevU).ToList();
    }

    static void ComputeMaxError(List<Vector3> points, List<Vector3> bez, List<float> parameters, 
        out float maxDist, out int splitPoint)
    {
        maxDist = 0;
        splitPoint = points.Count / 2;
        var t_distMap = MapTtoRelativeDistances(bez, 10);

        for (int i = 0; i < points.Count; i++)
        {
            var point = points[i];
            
            // Find 't' for a point on the bez curve that's as close to 'point' as possible:
            var t = Find_t(bez, parameters[i], t_distMap, 10);

            var v = BezierQ(bez[0], bez[1], bez[2], bez[3], t) - point;
            var dist = v.sqrMagnitude;

            if (dist >= maxDist)
            {
                maxDist = dist;
                splitPoint = i;
            }
        }
    }
    
    // Sample 't's and map them to relative distances along the curve:
    static List<float> MapTtoRelativeDistances(List<Vector3> bez, int B_parts)
    {
        float sumLen = 0;
        var B_t_dist = new List<float>() {0};
        var B_t_prev = bez[0];
        
        for (int i = 1; i <= B_parts; i++)
        {
            var B_t_curr = BezierQ(bez[0], bez[1], bez[2], bez[3], (float) i / B_parts);
            sumLen += Vector3.Distance(B_t_curr, B_t_prev);
            
            B_t_dist.Add(sumLen);
            B_t_prev = B_t_curr;
        }
        
        // Normalize B_length to the same interval as the parameter distances; 0 to 1:
        return B_t_dist.Select(x => x / sumLen).ToList();
    }

    static float Find_t(List<Vector3> bez, float param, List<float> t_distMap, int B_parts)
    {
        if (param < 0f)
        {
            return 0;
        }

        if (param > 1f)
        {
            return 1;
        }
        
        /*
       'param' is a value between 0 and 1 telling us the relative position
       of a point on the source polyline (linearly from the start (0) to the end (1)).
       To see if a given curve - 'bez' - is a close approximation of the polyline,
       we compare such a poly-point to the point on the curve that's the same
       relative distance along the curve's length.
       But finding that curve-point takes a little work:
       There is a function "B(t)" to find points along a curve from the parametric parameter 't'
       (also relative from 0 to 1: http://stackoverflow.com/a/32841764/1869660
                                   http://pomax.github.io/bezierinfo/#explanation),
       but 't' isn't linear by length (http://gamedev.stackexchange.com/questions/105230).
       So, we sample some points along the curve using a handful of values for 't'.
       Then, we calculate the length between those samples via plain euclidean distance;
       B(t) concentrates the points around sharp turns, so this should give us a good-enough outline of the curve.
       Thus, for a given relative distance ('param'), we can now find an upper and lower value
       for the corresponding 't' by searching through those sampled distances.
       Finally, we just use linear interpolation to find a better value for the exact 't'.
       More info:
           http://gamedev.stackexchange.com/questions/105230/points-evenly-spaced-along-a-bezier-curve
           http://stackoverflow.com/questions/29438398/cheap-way-of-calculating-cubic-bezier-length
           http://steve.hollasch.net/cgindex/curves/cbezarclen.html
           https://github.com/retuxx/tinyspline
        */

        float t = 0;
        for (int i = 1; i <= B_parts; i++)
        {
            if (param <= t_distMap[i])
            {
                var tMin = (float)(i - 1) / B_parts;
                var tMax = (float) i / B_parts;
                var lenMin = t_distMap[i - 1];
                var lenMax = t_distMap[i];

                t = (param - lenMin) / (lenMax - lenMin) * (tMax - tMin) + tMin;
                break;
            }
        }

        return t;
    }
    
    static Vector3 BezierQ(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        var tx = 1f - t;
        var pA = p0 * tx * tx * tx;
        var pB = p1 * 3 * tx * tx * t;
        var pC = p2 * 3 * tx * t * t;
        var pD = p3 * t * t * t;
        return (pA + pB) + (pC + pD);
    }

    static Vector3 BezierQPrime(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        var tx = 1f - t;
        var pA = (p1 - p0) * 3f * tx * tx;
        var pB = (p2 - p1) * 6f * tx * t;
        var pC = (p3 - p2) * 3f * t * t;
        return pA + pB + pC;
    }

    static Vector3 BezierQPrimeQPrime(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        return ((p2 - p1 * 2f) + p0) * 6f * (1f - t) + ((p3 - p2 * 2f) + p1) * 6f * t;
    }
}
