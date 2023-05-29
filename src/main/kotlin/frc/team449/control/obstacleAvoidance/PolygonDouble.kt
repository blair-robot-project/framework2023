package frc.team449.control.obstacleAvoidance

import java.awt.Point
import java.awt.Polygon
import java.awt.Rectangle
import java.awt.Shape
import java.awt.geom.*
import java.io.Serializable

/*

   Licensed to the Apache Software Foundation (ASF) under one or more
   contributor license agreements.  See the NOTICE file distributed with
   this work for additional information regarding copyright ownership.
   The ASF licenses this file to You under the Apache License, Version 2.0
   (the "License"); you may not use this file except in compliance with
   the License.  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

*/

/**
 * This class is a Polygon with double coordinates.
 *
 * @version $Id: Polygon2D.java 594018 2007-11-12 04:17:41Z cam $
 */
class PolygonDouble : Shape, Cloneable, Serializable {
  /**
   * The total number of points. The value of `npoints`
   * represents the number of valid points in this `Polygon`.
   */
  var npoints = 0

  /**
   * The array of *x* coordinates. The value of [npoints][.npoints] is
   * equal to the
   * number of points in this `Polygon2D`.
   */
  var xpoints: DoubleArray

  /**
   * The array of *x* coordinates. The value of [npoints][.npoints] is
   * equal to the
   * number of points in this `Polygon2D`.
   */
  var ypoints: DoubleArray

  /**
   * Bounds of the Polygon2D.
   *
   * @see .getBounds
   */
  protected var bounds: Rectangle2D? = null
  private var path: GeneralPath? = null
  private var closedPath: GeneralPath? = null

  /**
   * Creates an empty Polygon2D.
   */
  constructor() {
    xpoints = DoubleArray(4)
    ypoints = DoubleArray(4)
  }

  /**
   * Constructs and initializes a `Polygon2D` from the specified
   * Rectangle2D.
   *
   * @param rec the Rectangle2D
   * @throws NullPointerException rec is `null`.
   */
  constructor(rec: Rectangle2D?) {
    if (rec == null) {
      throw IndexOutOfBoundsException("null Rectangle")
    }
    npoints = 4
    xpoints = DoubleArray(4)
    ypoints = DoubleArray(4)
    xpoints[0] = rec.minX
    ypoints[0] = rec.minY
    xpoints[1] = rec.maxX
    ypoints[1] = rec.minY
    xpoints[2] = rec.maxX
    ypoints[2] = rec.maxY
    xpoints[3] = rec.minX
    ypoints[3] = rec.maxY
    calculatePath()
  }

  /**
   * Constructs and initializes a `Polygon2D` from the specified
   * Polygon.
   *
   * @param pol the Polygon
   * @throws NullPointerException pol is `null`.
   */
  constructor(pol: Polygon?) {
    if (pol == null) {
      throw IndexOutOfBoundsException("null Polygon")
    }
    npoints = pol.npoints
    xpoints = DoubleArray(pol.npoints)
    ypoints = DoubleArray(pol.npoints)
    for (i in 0 until pol.npoints) {
      xpoints[i] = pol.xpoints[i].toDouble()
      ypoints[i] = pol.ypoints[i].toDouble()
    }
    calculatePath()
  }

  /**
   * Constructs and initializes a `Polygon2D` from the specified
   * parameters.
   *
   * @param xpoints an array of *x* coordinates
   * @param ypoints an array of *y* coordinates
   * @param npoints the total number of points in the `Polygon2D`
   * @throws NegativeArraySizeException if the value of
   * `npoints` is negative.
   * @throws IndexOutOfBoundsException  if `npoints` is
   * greater than the length of
   * `xpoints`
   * or the length of `ypoints`.
   * @throws NullPointerException       if `xpoints` or
   * `ypoints` is `null`.
   */
  constructor(xpoints: DoubleArray, ypoints: DoubleArray) {
    if (xpoints.size != ypoints.size) {
      throw IndexOutOfBoundsException("npoints > xpoints.length || npoints > ypoints.length")
    }
    npoints = xpoints.size
    this.xpoints = DoubleArray(npoints)
    this.ypoints = DoubleArray(npoints)
    System.arraycopy(xpoints, 0, this.xpoints, 0, npoints)
    System.arraycopy(ypoints, 0, this.ypoints, 0, npoints)
    calculatePath()
  }

  /**
   * Constructs and initializes a `Polygon2D` from the specified
   * parameters.
   *
   * @param xpoints an array of *x* coordinates
   * @param ypoints an array of *y* coordinates
   * @param npoints the total number of points in the `Polygon2D`
   * @throws NegativeArraySizeException if the value of
   * `npoints` is negative.
   * @throws IndexOutOfBoundsException  if `npoints` is
   * greater than the length of
   * `xpoints`
   * or the length of `ypoints`.
   * @throws NullPointerException       if `xpoints` or
   * `ypoints` is `null`.
   */
  constructor(xpoints: IntArray, ypoints: IntArray, npoints: Int) {
    if (npoints > xpoints.size || npoints > ypoints.size) {
      throw IndexOutOfBoundsException("npoints > xpoints.length || npoints > ypoints.length")
    }
    this.npoints = npoints
    this.xpoints = DoubleArray(npoints)
    this.ypoints = DoubleArray(npoints)
    for (i in 0 until npoints) {
      this.xpoints[i] = xpoints[i].toDouble()
      this.ypoints[i] = ypoints[i].toDouble()
    }
    calculatePath()
  }

  /**
   * Resets this `Polygon` object to an empty polygon.
   */
  fun reset() {
    npoints = 0
    bounds = null
    path = GeneralPath()
    closedPath = null
  }

  public override fun clone(): Any {
    val pol = PolygonDouble()
    for (i in 0 until npoints) {
      pol.addPoint(xpoints[i], ypoints[i])
    }
    return pol
  }

  private fun calculatePath() {
    path = GeneralPath()
    path!!.moveTo(xpoints[0], ypoints[0])
    for (i in 1 until npoints) {
      path!!.lineTo(xpoints[i], ypoints[i])
    }
    bounds = path!!.bounds2D
    closedPath = null
  }

  private fun updatePath(x: Double, y: Double) {
    closedPath = null
    if (path == null) {
      path = GeneralPath(GeneralPath.WIND_EVEN_ODD)
      path!!.moveTo(x, y)
      bounds = Rectangle2D.Double(x, y, 0.0, 0.0)
    } else {
      path!!.lineTo(x, y)
      var _xmax = bounds!!.maxX
      var _ymax = bounds!!.maxY
      var _xmin = bounds!!.minX
      var _ymin = bounds!!.minY
      if (x < _xmin) _xmin = x else if (x > _xmax) _xmax = x
      if (y < _ymin) _ymin = y else if (y > _ymax) _ymax = y
      bounds = Rectangle2D.Double(_xmin, _ymin, _xmax - _xmin, _ymax - _ymin)
    }
  }

  val polyline2D: Polyline2D
    /*
        * get the associated {@link Polyline2D}.
        */
    get() {
      val pol = Polyline2D(xpoints, ypoints, npoints)
      pol.addPoint(xpoints[0], ypoints[0])
      return pol
    }
  val polygon: Polygon
    get() {
      val _xpoints = IntArray(npoints)
      val _ypoints = IntArray(npoints)
      for (i in 0 until npoints) {
        _xpoints[i] = xpoints[i].toInt() // todo maybe rounding is better ?
        _ypoints[i] = ypoints[i].toInt()
      }
      return Polygon(_xpoints, _ypoints, npoints)
    }

  fun addPoint(p: Point2D) {
    addPoint(p.x, p.y)
  }

  /**
   * Appends the specified coordinates to this `Polygon2D`.
   *
   * @param x the specified x coordinate
   * @param y the specified y coordinate
   */
  fun addPoint(x: Double, y: Double) {
    if (npoints == xpoints.size) {
      var tmp: DoubleArray
      tmp = DoubleArray(npoints * 2)
      System.arraycopy(xpoints, 0, tmp, 0, npoints)
      xpoints = tmp
      tmp = DoubleArray(npoints * 2)
      System.arraycopy(ypoints, 0, tmp, 0, npoints)
      ypoints = tmp
    }
    xpoints[npoints] = x
    ypoints[npoints] = y
    npoints++
    updatePath(x, y)
  }

  /**
   * Determines whether the specified [Point] is inside this
   * `Polygon`.
   *
   * @param p the specified `Point` to be tested
   * @return `true` if the `Polygon` contains the
   * `Point`; `false` otherwise.
   * @see .contains
   */
  operator fun contains(p: Point): Boolean {
    return contains(p.x, p.y)
  }

  /**
   * Determines whether the specified coordinates are inside this
   * `Polygon`.
   *
   *
   *
   * @param x the specified x coordinate to be tested
   * @param y the specified y coordinate to be tested
   * @return `true` if this `Polygon` contains
   * the specified coordinates, (*x*,&nbsp;*y*);
   * `false` otherwise.
   */
  fun contains(x: Int, y: Int): Boolean {
    return contains(x.toDouble(), y.toDouble())
  }

  /**
   * Returns the high precision bounding box of the [Shape].
   *
   * @return a [Rectangle2D] that precisely
   * bounds the `Shape`.
   */
  override fun getBounds2D(): Rectangle2D {
    return bounds!!
  }

  override fun getBounds(): Rectangle? {
    return if (bounds == null) null else bounds!!.bounds
  }

  /**
   * Determines if the specified coordinates are inside this
   * `Polygon`. For the definition of
   * *insideness*, see the class comments of [Shape].
   *
   * @param x the specified x coordinate
   * @param y the specified y coordinate
   * @return `true` if the `Polygon` contains the
   * specified coordinates; `false` otherwise.
   */
  override fun contains(x: Double, y: Double): Boolean {
    if (npoints <= 2 || !bounds!!.contains(x, y)) {
      return false
    }
    updateComputingPath()
    return closedPath!!.contains(x, y)
  }

  private fun updateComputingPath() {
    if (npoints >= 1) {
      if (closedPath == null) {
        closedPath = path!!.clone() as GeneralPath
        closedPath!!.closePath()
      }
    }
  }

  /**
   * Tests if a specified [Point2D] is inside the boundary of this
   * `Polygon`.
   *
   * @param p a specified `Point2D`
   * @return `true` if this `Polygon` contains the
   * specified `Point2D`; `false`
   * otherwise.
   * @see .contains
   */
  override fun contains(p: Point2D): Boolean {
    return contains(p.x, p.y)
  }

  /**
   * Tests if the interior of this `Polygon` intersects the
   * interior of a specified set of rectangular coordinates.
   *
   * @param x the x coordinate of the specified rectangular
   * shape's top-left corner
   * @param y the y coordinate of the specified rectangular
   * shape's top-left corner
   * @param w the width of the specified rectangular shape
   * @param h the height of the specified rectangular shape
   * @return `true` if the interior of this
   * `Polygon` and the interior of the
   * specified set of rectangular
   * coordinates intersect each other;
   * `false` otherwise.
   */
  override fun intersects(x: Double, y: Double, w: Double, h: Double): Boolean {
    if (npoints <= 0 || !bounds!!.intersects(x, y, w, h)) {
      return false
    }
    updateComputingPath()
    return closedPath!!.intersects(x, y, w, h)
  }

  /**
   * Tests if the interior of this `Polygon` intersects the
   * interior of a specified `Rectangle2D`.
   *
   * @param r a specified `Rectangle2D`
   * @return `true` if this `Polygon` and the
   * interior of the specified `Rectangle2D`
   * intersect each other; `false`
   * otherwise.
   */
  override fun intersects(r: Rectangle2D): Boolean {
    return intersects(r.x, r.y, r.width, r.height)
  }

  /**
   * Tests if the interior of this `Polygon` entirely
   * contains the specified set of rectangular coordinates.
   *
   * @param x the x coordinate of the top-left corner of the
   * specified set of rectangular coordinates
   * @param y the y coordinate of the top-left corner of the
   * specified set of rectangular coordinates
   * @param w the width of the set of rectangular coordinates
   * @param h the height of the set of rectangular coordinates
   * @return `true` if this `Polygon` entirely
   * contains the specified set of rectangular
   * coordinates; `false` otherwise.
   */
  override fun contains(x: Double, y: Double, w: Double, h: Double): Boolean {
    if (npoints <= 0 || !bounds!!.intersects(x, y, w, h)) {
      return false
    }
    updateComputingPath()
    return closedPath!!.contains(x, y, w, h)
  }

  /**
   * Tests if the interior of this `Polygon` entirely
   * contains the specified `Rectangle2D`.
   *
   * @param r the specified `Rectangle2D`
   * @return `true` if this `Polygon` entirely
   * contains the specified `Rectangle2D`;
   * `false` otherwise.
   * @see .contains
   */
  override fun contains(r: Rectangle2D): Boolean {
    return contains(r.x, r.y, r.width, r.height)
  }

  /**
   * Returns an iterator object that iterates along the boundary of this
   * `Polygon` and provides access to the geometry
   * of the outline of this `Polygon`. An optional
   * [AffineTransform] can be specified so that the coordinates
   * returned in the iteration are transformed accordingly.
   *
   * @param at an optional `AffineTransform` to be applied to the
   * coordinates as they are returned in the iteration, or
   * `null` if untransformed coordinates are desired
   * @return a [PathIterator] object that provides access to the
   * geometry of this `Polygon`.
   */
  override fun getPathIterator(at: AffineTransform): PathIterator? {
    updateComputingPath()
    return if (closedPath == null) null else closedPath!!.getPathIterator(at)
  }

  /**
   * Returns an iterator object that iterates along the boundary of
   * the `Polygon2D` and provides access to the geometry of the
   * outline of the `Shape`. Only SEG_MOVETO, SEG_LINETO, and
   * SEG_CLOSE point types are returned by the iterator.
   * Since polygons are already flat, the `flatness` parameter
   * is ignored.
   *
   * @param at       an optional `AffineTransform` to be applied to the
   * coordinates as they are returned in the iteration, or
   * `null` if untransformed coordinates are desired
   * @param flatness the maximum amount that the control points
   * for a given curve can vary from colinear before a subdivided
   * curve is replaced by a straight line connecting the
   * endpoints. Since polygons are already flat the
   * `flatness` parameter is ignored.
   * @return a `PathIterator` object that provides access to the
   * `Shape` object's geometry.
   */
  override fun getPathIterator(at: AffineTransform, flatness: Double): PathIterator? {
    return getPathIterator(at)
  }
}

/*
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements. See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
/**
 * This class has the same behavior than [Polygon2D], except that
 * the figure is not closed.
 *
 * @version $Id: Polyline2D.java 594018 2007-11-12 04:17:41Z cam $
 */
class Polyline2D : Shape, Cloneable, Serializable {
  /**
   * The total number of points. The value of `npoints`
   * represents the number of points in this `Polyline2D`.
   */
  var npoints = 0

  /**
   * The array of *x* coordinates. The value of [npoints][.npoints] is
   * equal to the
   * number of points in this `Polyline2D`.
   */
  var xpoints: DoubleArray

  /**
   * The array of *x* coordinates. The value of [npoints][.npoints] is
   * equal to the
   * number of points in this `Polyline2D`.
   */
  var ypoints: DoubleArray

  /**
   * Bounds of the Polyline2D.
   *
   * @see .getBounds
   */
  protected var bounds: Rectangle2D? = null
  private var path: GeneralPath? = null
  private var closedPath: GeneralPath? = null

  /**
   * Creates an empty Polyline2D.
   */
  constructor() {
    xpoints = DoubleArray(4)
    ypoints = DoubleArray(4)
  }

  /**
   * Constructs and initializes a `Polyline2D` from the specified
   * parameters.
   *
   * @param xpoints an array of *x* coordinates
   * @param ypoints an array of *y* coordinates
   * @param npoints the total number of points in the
   * `Polyline2D`
   * @throws NegativeArraySizeException if the value of
   * `npoints` is negative.
   * @throws IndexOutOfBoundsException  if `npoints` is
   * greater than the length of
   * `xpoints`
   * or the length of `ypoints`.
   * @throws NullPointerException       if `xpoints` or
   * `ypoints` is `null`.
   */
  constructor(xpoints: DoubleArray, ypoints: DoubleArray, npoints: Int) {
    if (npoints > xpoints.size || npoints > ypoints.size) {
      throw IndexOutOfBoundsException("npoints > xpoints.length || npoints > ypoints.length")
    }
    this.npoints = npoints
    this.xpoints = DoubleArray(npoints + 1) // make space for one more to close the polyline
    this.ypoints = DoubleArray(npoints + 1) // make space for one more to close the polyline
    System.arraycopy(xpoints, 0, this.xpoints, 0, npoints)
    System.arraycopy(ypoints, 0, this.ypoints, 0, npoints)
    calculatePath()
  }

  /**
   * Constructs and initializes a `Polyline2D` from the specified
   * parameters.
   *
   * @param xpoints an array of *x* coordinates
   * @param ypoints an array of *y* coordinates
   * @param npoints the total number of points in the `Polyline2D`
   * @throws NegativeArraySizeException if the value of
   * `npoints` is negative.
   * @throws IndexOutOfBoundsException  if `npoints` is
   * greater than the length of
   * `xpoints`
   * or the length of `ypoints`.
   * @throws NullPointerException       if `xpoints` or
   * `ypoints` is `null`.
   */
  constructor(xpoints: IntArray, ypoints: IntArray, npoints: Int) {
    if (npoints > xpoints.size || npoints > ypoints.size) {
      throw IndexOutOfBoundsException("npoints > xpoints.length || npoints > ypoints.length")
    }
    this.npoints = npoints
    this.xpoints = DoubleArray(npoints)
    this.ypoints = DoubleArray(npoints)
    for (i in 0 until npoints) {
      this.xpoints[i] = xpoints[i].toDouble()
      this.ypoints[i] = ypoints[i].toDouble()
    }
    calculatePath()
  }

  constructor(line: Line2D) {
    npoints = 2
    xpoints = DoubleArray(2)
    ypoints = DoubleArray(2)
    xpoints[0] = line.x1
    xpoints[1] = line.x2
    ypoints[0] = line.y1
    ypoints[1] = line.y2
    calculatePath()
  }

  /**
   * Resets this `Polyline2D` object to an empty polygon.
   * The coordinate arrays and the data in them are left untouched
   * but the number of points is reset to zero to mark the old
   * vertex data as invalid and to start accumulating new vertex
   * data at the beginning.
   * All internally-cached data relating to the old vertices
   * are discarded.
   * Note that since the coordinate arrays from before the reset
   * are reused, creating a new empty `Polyline2D` might
   * be more memory efficient than resetting the current one if
   * the number of vertices in the new polyline data is significantly
   * smaller than the number of vertices in the data from before the
   * reset.
   */
  fun reset() {
    npoints = 0
    bounds = null
    path = GeneralPath()
    closedPath = null
  }

  public override fun clone(): Any {
    val pol = Polyline2D()
    for (i in 0 until npoints) {
      pol.addPoint(xpoints[i], ypoints[i])
    }
    return pol
  }

  private fun calculatePath() {
    path = GeneralPath()
    path!!.moveTo(xpoints[0], ypoints[0])
    for (i in 1 until npoints) {
      path!!.lineTo(xpoints[i], ypoints[i])
    }
    bounds = path!!.bounds2D
    closedPath = null
  }

  private fun updatePath(x: Double, y: Double) {
    closedPath = null
    if (path == null) {
      path = GeneralPath(GeneralPath.WIND_EVEN_ODD)
      path!!.moveTo(x, y)
      bounds = Rectangle2D.Double(x, y, 0.0, 0.0)
    } else {
      path!!.lineTo(x, y)
      var _xmax = bounds!!.maxX
      var _ymax = bounds!!.maxY
      var _xmin = bounds!!.minX
      var _ymin = bounds!!.minY
      if (x < _xmin) _xmin = x else if (x > _xmax) _xmax = x
      if (y < _ymin) _ymin = y else if (y > _ymax) _ymax = y
      bounds = Rectangle2D.Double(_xmin, _ymin, _xmax - _xmin, _ymax - _ymin)
    }
  }

  fun addPoint(p: Point2D) {
    addPoint(p.x, p.y)
  }

  /**
   * Appends the specified coordinates to this `Polyline2D`.
   *
   *
   * If an operation that calculates the bounding box of this
   * `Polyline2D` has already been performed, such as
   * `getBounds` or `contains`, then this
   * method updates the bounding box.
   *
   * @param x the specified x coordinate
   * @param y the specified y coordinate
   * @see java.awt.Polygon.getBounds
   *
   * @see java.awt.Polygon.contains
   */
  fun addPoint(x: Double, y: Double) {
    if (npoints == xpoints.size) {
      var tmp: DoubleArray
      tmp = DoubleArray(npoints * 2)
      System.arraycopy(xpoints, 0, tmp, 0, npoints)
      xpoints = tmp
      tmp = DoubleArray(npoints * 2)
      System.arraycopy(ypoints, 0, tmp, 0, npoints)
      ypoints = tmp
    }
    xpoints[npoints] = x
    ypoints[npoints] = y
    npoints++
    updatePath(x, y)
  }

  /**
   * Gets the bounding box of this `Polyline2D`.
   * The bounding box is the smallest [Rectangle] whose
   * sides are parallel to the x and y axes of the
   * coordinate space, and can completely contain the `Polyline2D`.
   *
   * @return a `Rectangle` that defines the bounds of this
   * `Polyline2D`.
   */
  override fun getBounds(): Rectangle? {
    return if (bounds == null) null else bounds!!.bounds
  }

  private fun updateComputingPath() {
    if (npoints >= 1) {
      if (closedPath == null) {
        closedPath = path!!.clone() as GeneralPath
        closedPath!!.closePath()
      }
    }
  }

  /**
   * Determines whether the specified [Point] is inside this
   * `Polyline2D`.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  operator fun contains(p: Point?): Boolean {
    return false
  }

  /**
   * Determines if the specified coordinates are inside this
   * `Polyline2D`.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  override fun contains(x: Double, y: Double): Boolean {
    return false
  }

  /**
   * Determines whether the specified coordinates are inside this
   * `Polyline2D`.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  fun contains(x: Int, y: Int): Boolean {
    return false
  }

  /**
   * Returns the high precision bounding box of the [Shape].
   *
   * @return a [Rectangle2D] that precisely
   * bounds the `Shape`.
   */
  override fun getBounds2D(): Rectangle2D {
    return bounds!!
  }

  /**
   * Tests if a specified [Point2D] is inside the boundary of this
   * `Polyline2D`.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  override fun contains(p: Point2D): Boolean {
    return false
  }

  /**
   * Tests if the interior of this `Polygon` intersects the
   * interior of a specified set of rectangular coordinates.
   *
   * @param x the x coordinate of the specified rectangular
   * shape's top-left corner
   * @param y the y coordinate of the specified rectangular
   * shape's top-left corner
   * @param w the width of the specified rectangular shape
   * @param h the height of the specified rectangular shape
   * @return `true` if the interior of this
   * `Polygon` and the interior of the
   * specified set of rectangular
   * coordinates intersect each other;
   * `false` otherwise.
   */
  override fun intersects(x: Double, y: Double, w: Double, h: Double): Boolean {
    if (npoints <= 0 || !bounds!!.intersects(x, y, w, h)) {
      return false
    }
    updateComputingPath()
    return closedPath!!.intersects(x, y, w, h)
  }

  /**
   * Tests if the interior of this `Polygon` intersects the
   * interior of a specified `Rectangle2D`.
   *
   * @param r a specified `Rectangle2D`
   * @return `true` if this `Polygon` and the
   * interior of the specified `Rectangle2D`
   * intersect each other; `false`
   * otherwise.
   */
  override fun intersects(r: Rectangle2D): Boolean {
    return intersects(r.x, r.y, r.width, r.height)
  }

  /**
   * Tests if the interior of this `Polyline2D` entirely
   * contains the specified set of rectangular coordinates.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  override fun contains(x: Double, y: Double, w: Double, h: Double): Boolean {
    return false
  }

  /**
   * Tests if the interior of this `Polyline2D` entirely
   * contains the specified `Rectangle2D`.
   * This method is required to implement the Shape interface,
   * but in the case of Line2D objects it always returns false since a line
   * contains no area.
   */
  override fun contains(r: Rectangle2D): Boolean {
    return false
  }

  /**
   * Returns an iterator object that iterates along the boundary of this
   * `Polygon` and provides access to the geometry
   * of the outline of this `Polygon`. An optional
   * [AffineTransform] can be specified so that the coordinates
   * returned in the iteration are transformed accordingly.
   *
   * @param at an optional `AffineTransform` to be applied to the
   * coordinates as they are returned in the iteration, or
   * `null` if untransformed coordinates are desired
   * @return a [PathIterator] object that provides access to the
   * geometry of this `Polygon`.
   */
  override fun getPathIterator(at: AffineTransform): PathIterator? {
    return if (path == null) null else path!!.getPathIterator(at)
  }

  val polygon2D: PolygonDouble
    /*
        * get the associated {@link Polygon2D}.
        * This method take care that may be the last point can
        * be equal to the first. In that case it must not be included in the Polygon,
        * as polygons declare their first point only once.
        */
    get() {
      val pol = PolygonDouble()
      for (i in 0 until npoints - 1) {
        pol.addPoint(xpoints[i], ypoints[i])
      }
      val p0 = Point2D.Double(xpoints[0], ypoints[0])
      val p1 = Point2D.Double(xpoints[npoints - 1], ypoints[npoints - 1])
      if (p0.distance(p1) > ASSUME_ZERO) pol.addPoint(xpoints[npoints - 1], ypoints[npoints - 1])
      return pol
    }

  /**
   * Returns an iterator object that iterates along the boundary of
   * the `Shape` and provides access to the geometry of the
   * outline of the `Shape`. Only SEG_MOVETO and SEG_LINETO, point
   * types
   * are returned by the iterator.
   * Since polylines are already flat, the `flatness` parameter
   * is ignored.
   *
   * @param at       an optional `AffineTransform` to be applied to the
   * coordinates as they are returned in the iteration, or
   * `null` if untransformed coordinates are desired
   * @param flatness the maximum amount that the control points
   * for a given curve can vary from colinear before a subdivided
   * curve is replaced by a straight line connecting the
   * endpoints. Since polygons are already flat the
   * `flatness` parameter is ignored.
   * @return a `PathIterator` object that provides access to the
   * `Shape` object's geometry.
   */
  override fun getPathIterator(at: AffineTransform, flatness: Double): PathIterator {
    return path!!.getPathIterator(at)
  }

  companion object {
    private const val ASSUME_ZERO = 0.001
  }
}
