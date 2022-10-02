// Copyright 2019 Virtual Repetitions LLC. All Rights Reserved
using UnityEngine;

namespace Yohash.Bezier
{
  [System.Serializable]
  /// <summary>
  /// Parameterized quadratic bezier.
  ///
  /// The standard Quadratic Bezier is a 3-point spline, taking in
  ///   (1) start point
  ///   (2) control point
  ///   (3) end point
  ///
  /// Most Bezier functions take in a "percent" variable with returns the vector on the
  /// curve corresponding to this percent.
  /// This percent is not necessarily going to be the "percent of the line by distance",
  /// as the points in a Bezier are not spaced evenly (dependant on the control point).
  ///
  /// The parameterized quadractic bezier takes in the 3 standard points, and immidiately
  /// parameterizes them by distance, creating a look uptable. This allows the user to hand
  /// in a standard percentage (of path by distance)
  /// and return a guarenteed position assuming a fixed-distance-spacing between all points
  /// on the curve.
  ///
  /// For more information on Beziers, including the recommendation to use a look-up-table,
  /// check out:
  /// https://pomax.github.io/bezierinfo/
  /// https://www.geometrictools.com/Documentation/MovingAlongCurveSpecifiedSpeed.pdf
  ///
  /// Jason Heebl
  /// 2017-11-7
  /// </summary>
  public class ParameterizedBezier
  {
    public Vector3[] Line;

    [SerializeField] private float[] distancePerPercent;
    [SerializeField] private float[] pathPercent;

    [SerializeField] private int numberOfPoints;

    [SerializeField] private Vector3 startPoint;
    [SerializeField] private Vector3 endPoint;
    [SerializeField] private Vector3 controlPoint;

    public Vector3 GetStartPoint() { return startPoint; }
    public Vector3 GetEndPoint() { return endPoint; }
    public Vector3 GetControlPoint() { return controlPoint; }

    // **********************************************************************************************
    //    CONSTRUCTORS (and re-constructors)
    // **********************************************************************************************
    public ParameterizedBezier(Vector3 _start, Vector3 _end, Vector3 _control)
    {
      RebuildPath(_start, _end, _control);
    }

    public ParameterizedBezier(Vector3 _start, Vector3 _end, Vector3 _control, int _numberOfPoints)
    {
      RebuildPath(_start, _end, _control, _numberOfPoints);
    }

    public void RebuildPath(Vector3 NewStart, Vector3 NewEnd, Vector3 NewControl)
    {
      RebuildPath(NewStart, NewEnd, NewControl, 100);
    }

    public void RebuildPath(Vector3 NewStart, Vector3 NewEnd, Vector3 NewControl, int NewNumberOfPoints)
    {
      startPoint = NewStart;
      endPoint = NewEnd;
      controlPoint = NewControl;
      numberOfPoints = NewNumberOfPoints;
      buildParameterizedBezier();
    }

    // **********************************************************************************************
    //    TOOLS for public access to parameterized points
    // **********************************************************************************************
    /// <summary>
    /// Gets the curve total distance.
    /// </summary>
    public float GetTotalCurveDistance()
    {
      return distancePerPercent[numberOfPoints - 1];
    }

    /// <summary>
    /// Given a desired distance, this function calculates and returns the corresponding
    /// world position of the path. If distance is > the length of the path, the endpoint
    /// is returned.
    /// </summary>
    /// <param name="distance"></param>
    /// <returns></returns>
    public Vector3 GetPointAtDistanceAlongPath(float distance)
    {
      if (distance > GetTotalCurveDistance()) {
        return endPoint;
      }
      // return the linearized path point at this percentage
      return GetCorrespondingLinearizedPathPoint(distance / GetTotalCurveDistance());
    }

    /// <summary>
    /// Get an array of vector3 points, comprising the points of the path, linearly
    /// spaced (determined by number of points), starting at a given percent.
    /// </summary>
    /// <param name="desiredPercent"></param>
    /// <returns></returns>
    public int GetLineIndexStartingAtLinearizedPercent(float desiredPercent)
    {
      desiredPercent = Mathf.Clamp01(desiredPercent);
      // the actual distance as a fraction of the total distance
      float actualDistance = desiredPercent * GetTotalCurveDistance();

      // scan along the saved Distances over Time until we find the point where the
      // distance matches the actual arc distance
      int startIndex = 0;
      for (int i = 0; i < numberOfPoints; i++) {
        if (actualDistance > distancePerPercent[i]) {
          startIndex = i;
        }
      }

      return startIndex + 1;
    }

    /// <summary>
    /// Given a desired Percent, this function calculates and returns the corresponding
    /// linearized path point. This value is the linearized point.
    /// </summary>
    /// <param name="desiredPercent"></param>
    /// <returns></returns>
    public Vector3 GetCorrespondingLinearizedPathPoint(float desiredPercent)
    {
      desiredPercent = Mathf.Clamp01(desiredPercent);
      // get the linearizedPathPercent
      float linearizedPathPercent = GetCorrespondingLinearizedPathPercent(desiredPercent);
      // obtain the next position from path percent
      Vector3 nextPosition = QuadraticBezierCurve(linearizedPathPercent);
      return nextPosition;
    }


    /// <summary>
    /// Given a desired Percent, this function calculates and returns the corresponding
    /// linearized path percent. This value can be used with the non-linear
    /// QuadraticBezierCurve() to get the point on the line corresponding to desiredPercent
    /// of the curve distance
    /// </summary>
    /// <returns>The corresponding linearized path percent.</returns>
    /// <param name="desiredPercent">Desired percent, as a percent of total distance.</param>
    public float GetCorrespondingLinearizedPathPercent(float desiredPercent)
    {
      desiredPercent = Mathf.Clamp01(desiredPercent);
      // the actual distance as a fraction of the total distance
      float actualDistance = desiredPercent * GetTotalCurveDistance();

      // scan along the saved Distances over Time until we find the point where the
      // distance matches the actual arc distance
      int nearestIndex = 0;
      for (int i = 0; i < numberOfPoints; i++) {
        if (actualDistance > distancePerPercent[i]) {
          nearestIndex = i;
        }
      }
      // get the fractional distance (how far in-between recorded distances is our requested distance)
      float FractionalDistance = (actualDistance - distancePerPercent[nearestIndex]) /
        (distancePerPercent[nearestIndex + 1] - distancePerPercent[nearestIndex]);
      // linearly interpolate between these results to get the actual parameterized percent value
      float actualPercent = Mathf.Lerp(
        pathPercent[nearestIndex],
        pathPercent[nearestIndex + 1],
        FractionalDistance
      );

      return actualPercent;
    }


    /// <summary>
    /// Returns the point on the path as a percent of path-distance. This assumes a fixed-distance-spacing
    /// between all points along the Quadratic Bezier.
    /// This method utilizes a linear PathPercent and Distance relationship.
    /// </summary>
    /// <returns>The point corresponding to the input percent, as a fixed percent of distance</returns>
    /// <param name="pathPercent">Path percent.</param>
    public Vector3 LinearizedQuadraticBezierCurve(float desiredPercent)
    {
      if (desiredPercent <= 0) {
        return startPoint;
      }

      if (desiredPercent >= 1) {
        return endPoint;
      }

      // get the linearized path percent corresponding to the desired percent
      float actualPercent = GetCorrespondingLinearizedPathPercent(desiredPercent);

      return QuadraticBezierCurve(startPoint, endPoint, controlPoint, actualPercent);
    }


    /// <summary>
    /// Returns the point corresponding to t, which is bounded on [0,1].
    /// The function is non-linear.
    /// Computes by:
    ///    p = 1 - t
    ///    point1 = p * p * start;
    ///    point2 = 2 * t * p * ctrl;
    ///    point3 = t * t * end;
    /// return point1 + point2 + point3
    /// </summary>
    /// <param name="t">t</param>
    public static Vector3 QuadraticBezierCurve(Vector3 start, Vector3 end, Vector3 ctrl, float t)
    {
      float p = 1 - t;
      float A = p * p;
      float B = 2 * t * p;
      float C = t * t;
      // manually construct return vector3 to avoid using Unity Vector3 maths, as this is a HEAVY
      // call method, and the use of Vector3 math methods stacks up enough that this return here
      // has produced significant processing improvements
      return new Vector3(
        A * start.x + B * ctrl.x + C * end.x,
        A * start.y + B * ctrl.y + C * end.y,
        A * start.z + B * ctrl.z + C * end.z
      );
    }
    public static Vector2 QuadraticBezierCurve(Vector2 start, Vector2 end, Vector2 ctrl, float t)
    {
      float p = 1 - t;
      float A = p * p;
      float B = 2 * t * p;
      float C = t * t;
      // manually construct the return vector2
      return new Vector2(
        A * start.x + B * ctrl.x + C * end.x,
        A * start.y + B * ctrl.y + C * end.y
      );
    }

    public Vector3 QuadraticBezierCurve(float t)
    {
      return QuadraticBezierCurve(startPoint, endPoint, controlPoint, t);
    }

    // **********************************************************************************************
    //    PRIVATE BEZIER ASSEMBLY FUNCTIONS and TOOLS
    // **********************************************************************************************
    /// <summary>
    /// Builds the parameterized bezier based on previously stored startPoint, endPoint, and controlPoint.
    /// </summary>
    private void buildParameterizedBezier()
    {
      if (pathPercent == null || pathPercent.Length != numberOfPoints) {
        pathPercent = new float[numberOfPoints];
        distancePerPercent = new float[numberOfPoints];
        Line = new Vector3[numberOfPoints];
      }

      // define looping variables
      float increment = 1f / (numberOfPoints - 1f);
      float currentPercent = 0f;

      // initialize the first values
      pathPercent[0] = 0;
      distancePerPercent[0] = 0;
      Line[0] = startPoint;

      // look over the number of points in our look up table
      for (int i = 1; i < numberOfPoints; i++) {
        // increment our current path percent, and record it
        currentPercent += increment;
        pathPercent[i] = Mathf.Min(currentPercent, 1f);

        // get the distance between this percent (pathPercent[i]) and the last one (pathPercent[i-1])
        distancePerPercent[i] = approximateLengthOfQuadraticBezierCurveSegment(startPoint, endPoint, controlPoint, pathPercent[i - 1], pathPercent[i], 4);
        // accumulate distances
        distancePerPercent[i] += distancePerPercent[i - 1];
        // compute this point
        Line[i] = QuadraticBezierCurve(currentPercent);
      }
    }

    /// <summary>
    /// Approximates the length of a complete quadratic bezier curve.
    /// Curve is bounded by argument ranging [0,1]
    /// </summary>
    private float approximateLengthOfQuadraticBezierCurve(Vector3 start, Vector3 end, Vector3 ctrl, float numSegments)
    {
      Vector3 startSegment, endSegment;

      float t = 0;
      float totalDistance = 0;
      float increment = 1f / numSegments;

      for (int i = 0; i < numSegments; i++) {
        startSegment = QuadraticBezierCurve(start, end, ctrl, t);
        t += increment;
        endSegment = QuadraticBezierCurve(start, end, ctrl, t);
        totalDistance += Vector3.Distance(startSegment, endSegment);
      }

      return totalDistance;
    }

    /// <summary>
    /// Approximates the length of segment of a quadratic bezier curve segment.
    /// Curve is bounded by argument ranging [t,tNext]
    /// </summary>
    /// <returns>The length of quadratic bezier curve segment.</returns>
    private float approximateLengthOfQuadraticBezierCurveSegment(Vector3 start, Vector3 end, Vector3 ctrl, float t, float tNext, float numSegments)
    {
      Vector3 startSegment, endSegment;

      float totalDistance = 0;
      float increment = (tNext - t) / numSegments;

      for (int i = 0; i < numSegments; i++) {
        startSegment = QuadraticBezierCurve(start, end, ctrl, t);
        t += increment;
        endSegment = QuadraticBezierCurve(start, end, ctrl, t);
        totalDistance += Vector3.Distance(startSegment, endSegment);
      }

      return totalDistance;
    }
  }
}
