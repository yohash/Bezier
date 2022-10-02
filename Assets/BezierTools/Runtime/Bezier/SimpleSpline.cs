using UnityEngine;

namespace Yohash.Bezier
{
  /// <summary>
  /// The Simple Spline class operates on a 3-point spline, taking in
  ///   (1) start point
  ///   (2) control point
  ///   (3) end point
  /// Points along the curve are accessed as a %-of-the-whole
  /// (ie. 1 is the end point, 0.5 is the midpoint)
  /// </summary>
  public static class SimpleSpline
  {
    /// <summary>
    /// Moves along a 2 point curve. Note that this is a straight line
    /// </summary>
    /// <returns>The along2 point cuve.</returns>
    /// <param name="start">Start.</param>
    /// <param name="end">End.</param>
    /// <param name="t">Line percent (0 = start, 1 = end)</param>
    public static Vector3 MoveAlong2PointCurve(Vector3 start, Vector3 end, float t)
    {
      return (1 - t) * start + t * end;
    }

    /// <summary>
    /// Moves along the 3 point curve.
    /// </summary>
    /// <returns>Vector3 point request along curve</returns>
    /// <param name="start">Start.</param>
    /// <param name="end">End.</param>
    /// <param name="middle">Middle - the control point.</param>
    /// <param name="t">Line percent (0 = start, 1 = end). This line point will be weighted by ArcPoint, and isnt linear</param>
    public static Vector3 MoveAlong3PointCurve(Vector3 start, Vector3 end, Vector3 middle, float t)
    {
      Vector3 point1 = Mathf.Pow((1 - t), 2) * start;
      Vector3 point2 = 2 * t * (1 - t) * middle;
      Vector3 point3 = Mathf.Pow(t, 2) * end;
      return point1 + point2 + point3;
    }

    public static float ApproximateLengthOf3PointCurve(Vector3 start, Vector3 middle, Vector3 end, float numSegments)
    {
      Vector3 startSegment, endSegment;
      float t = 0;
      float totalDistance = 0;
      float increment = 1 / numSegments;
      for (int i = 0; i < numSegments; i++) {
        startSegment = MoveAlong3PointCurve(start, end, middle, t);
        t += increment;
        endSegment = MoveAlong3PointCurve(start, end, middle, t);
        totalDistance += Vector3.Distance(startSegment, endSegment);
      }
      return totalDistance;
    }

    public static float ApproximateLengthOf3PointCurveSegment(Vector3 start, Vector3 middle, Vector3 end, float t, float nextT, float numSegments)
    {
      Vector3 startSegment, endSegment;
      float totalDistance = 0;
      float increment = (nextT - t) / numSegments;
      for (int i = 0; i < numSegments; i++) {
        startSegment = MoveAlong3PointCurve(start, end, middle, t);
        t += increment;
        endSegment = MoveAlong3PointCurve(start, end, middle, t);
        totalDistance += Vector3.Distance(startSegment, endSegment);
      }
      return totalDistance;
    }
  }
}
