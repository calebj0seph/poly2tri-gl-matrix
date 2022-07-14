/*
 * Poly2Tri Copyright (c) 2009-2018, Poly2Tri Contributors
 * https://github.com/jhasse/poly2tri
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of Poly2Tri nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without specific
 *   prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import { makeNode, Node } from './advancing-front';
import { Edge, Point, pointsEqual } from './edge';
import { SweepContext } from './sweep-context';
import { Triangle } from './triangle';
import {
  PI_3div4,
  PI_div2,
  Epsilon,
  Orientation,
  orient2d,
  inScanArea,
} from './utils';

/**
 * Triangulate simple polygon with holes
 *
 * @param tcx
 */
export function triangulate(tcx: SweepContext) {
  tcx.initTriangulation();
  tcx.createAdvancingFront();
  // Sweep points; build mesh
  sweepPoints(tcx);
  // Clean up
  finalizationPolygon(tcx);
}

/**
 * Start sweeping the Y-sorted point set from bottom to top
 *
 * @param tcx
 */
function sweepPoints(tcx: SweepContext) {
  for (let i = 1; i < tcx.pointCount; i++) {
    const point = tcx.getPoint(i);
    const node = pointEvent(tcx, point);
    for (const t of point.edgeList) {
      edgeEvent(tcx, t, node);
    }
  }
}

/**
 * Find closes node to the left of the new point and
 * create a new triangle. If needed new holes and basins
 * will be filled to.
 *
 * @param tcx
 * @param point
 * @return
 */
function pointEvent(tcx: SweepContext, point: Point): Node {
  const node = tcx.locateNode(point);
  if (
    node === null ||
    node.point === null ||
    node.next === null ||
    node.next.point === null
  ) {
    throw new Error('pointEvent: node is null');
  }

  const newNode = newFrontTriangle(tcx, point, node);

  // Only need to check +epsilon since point never have smaller
  // x value than node due to how we fetch nodes from the front
  if (point.position[0] <= node.point.position[0] + Epsilon) {
    fill(tcx, node);
  }

  //tcx.AddNode(newNode);

  fillAdvancingFront(tcx, newNode);
  return newNode;
}

/**
 * @param tcx
 * @param edge
 * @param node
 */
function edgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  tcx.edgeEvent.constrainedEdge = edge;
  tcx.edgeEvent.right = edge.p.position[0] > edge.q.position[0];

  if (isEdgeSideOfTriangle(node.triangle, edge.p, edge.q)) {
    return;
  }

  // For now we will do all needed filling
  // TODO: integrate with flip process might give some better performance
  //       but for now this avoid the issue with cases that needs both flips and fills
  fillEdgeEvent(tcx, edge, node);

  if (node.triangle === null) {
    throw new Error('edgeEvent: node.triangle is null');
  }

  edgeEventWithTriangle(tcx, edge.p, edge.q, node.triangle, edge.q);
}

function edgeEventWithTriangle(
  tcx: SweepContext,
  ep: Point,
  eq: Point,
  t: Triangle,
  point: Point
) {
  let triangle: Triangle | null = t;
  if (isEdgeSideOfTriangle(triangle, ep, eq)) {
    return;
  }

  const p1 = triangle.pointCCW(point);
  const o1 = orient2d(eq, p1, ep);
  if (o1 === Orientation.Collinear) {
    if (triangle.containsPoints(eq, p1)) {
      triangle.markConstrainedEdge(eq, p1);

      const edge = tcx.edgeEvent.constrainedEdge;
      if (edge === null) {
        throw new Error(
          'edgeEventWithTriangle: tcx.edgeEvent.constrainedEdge is null'
        );
      }

      // We are modifying the constraint maybe it would be better to
      // not change the given constraint and just keep a variable for the new constraint
      edge.q = p1;
      triangle = triangle.neighborAcross(point);

      if (triangle === null) {
        throw new Error('edgeEventWithTriangle: triangle is null');
      }

      edgeEventWithTriangle(tcx, ep, p1, triangle, p1);
    } else {
      throw new Error('edgeEventWithTriangle: collinear points not supported');
    }

    return;
  }

  const p2 = triangle.pointCW(point);
  const o2 = orient2d(eq, p2, ep);
  if (o2 === Orientation.Collinear) {
    if (triangle.containsPoints(eq, p2)) {
      triangle.markConstrainedEdge(eq, p2);

      const edge = tcx.edgeEvent.constrainedEdge;
      if (edge === null) {
        throw new Error(
          'edgeEventWithTriangle: tcx.edgeEvent.constrainedEdge is null'
        );
      }

      // We are modifying the constraint maybe it would be better to
      // not change the given constraint and just keep a variable for the new constraint
      edge.q = p2;
      triangle = triangle.neighborAcross(point);

      if (triangle === null) {
        throw new Error('edgeEventWithTriangle: triangle is null');
      }

      edgeEventWithTriangle(tcx, ep, p2, triangle, p2);
    } else {
      throw new Error('edgeEventWithTriangle: collinear points not supported');
    }

    return;
  }

  if (o1 === o2) {
    // Need to decide if we are rotating CW or CCW to get to a triangle
    // that will cross edge
    triangle =
      o1 === Orientation.CW
        ? triangle.neighborCCW(point)
        : triangle.neighborCW(point);

    if (triangle === null) {
      throw new Error('edgeEventWithTriangle: triangle is null');
    }

    edgeEventWithTriangle(tcx, ep, eq, triangle, point);
  } else {
    // This triangle crosses constraint so lets flippin start!
    if (triangle === null) {
      throw new Error('edgeEventWithTriangle: triangle is null');
    }

    flipEdgeEvent(tcx, ep, eq, triangle, point);
  }
}

/**
 * Creates a new front triangle and legalize it
 *
 * @param tcx
 * @param point
 * @param node
 * @return
 */
function newFrontTriangle(tcx: SweepContext, point: Point, node: Node): Node {
  if (node.next === null)
    throw new Error('newFrontTriangle: node.next is null');
  if (node.triangle === null)
    throw new Error('newFrontTriangle: node.triangle is null');

  const triangle = new Triangle(point, node.point, node.next.point);

  triangle.markNeighbor(node.triangle);
  tcx.addToMap(triangle);

  const newNode = makeNode(point);
  newNode.next = node.next;
  newNode.prev = node;

  node.next.prev = newNode;
  node.next = newNode;

  if (!legalize(tcx, triangle)) {
    tcx.mapTriangleToNodes(triangle);
  }

  return newNode;
}

/**
 * Adds a triangle to the advancing front to fill a hole.
 * @param tcx
 * @param node - middle node, that is the bottom of the hole
 */
function fill(tcx: SweepContext, node: Node) {
  if (node.prev === null) throw new Error('fill: node.prev is null');
  if (node.next === null) throw new Error('fill: node.next is null');
  if (node.triangle === null) throw new Error('fill: node.triangle is null');
  if (node.prev.triangle === null)
    throw new Error('fill: node.prev.triangle is null');

  const triangle = new Triangle(node.prev.point, node.point, node.next.point);

  // TODO: should copy the constrained_edge value from neighbor triangles
  //       for now constrained_edge values are copied during the legalize
  triangle.markNeighbor(node.prev.triangle);
  triangle.markNeighbor(node.triangle);

  tcx.addToMap(triangle);

  // Update the advancing front
  node.prev.next = node.next;
  node.next.prev = node.prev;

  // If it was legalized the triangle has already been mapped
  if (!legalize(tcx, triangle)) {
    tcx.mapTriangleToNodes(triangle);
  }
}

/**
 * Returns true if triangle was legalized
 */
function legalize(tcx: SweepContext, t: Triangle): boolean {
  // To legalize a triangle we start by finding if any of the three edges
  // violate the Delaunay condition
  for (let i = 0; i < 3; i++) {
    if (t.delaunayEdge[i]) {
      continue;
    }

    const ot = t.getNeighbor(i);

    if (ot !== null) {
      const p = t.getPoint(i);
      const op = ot.oppositePoint(t, p);
      const oi = ot.index(op);

      // If this is a Constrained Edge or a Delaunay Edge(only during recursive legalization)
      // then we should not try to legalize
      if (ot.constrainedEdge[oi] || ot.delaunayEdge[oi]) {
        t.constrainedEdge[i] = ot.constrainedEdge[oi];
        continue;
      }

      const inside = incircle(p, t.pointCCW(p), t.pointCW(p), op);

      if (inside) {
        // Lets mark this shared edge as Delaunay
        t.delaunayEdge[i] = true;
        ot.delaunayEdge[oi] = true;

        // Lets rotate shared edge one vertex CW to legalize it
        rotateTrianglePair(t, p, ot, op);

        // We now got one valid Delaunay Edge shared by two triangles
        // This gives us 4 new edges to check for Delaunay

        // Make sure that triangle to node mapping is done only one time for a specific triangle
        let notLegalized = !legalize(tcx, t);
        if (notLegalized) {
          tcx.mapTriangleToNodes(t);
        }

        notLegalized = !legalize(tcx, ot);
        if (notLegalized) {
          tcx.mapTriangleToNodes(ot);
        }

        // Reset the Delaunay edges, since they only are valid Delaunay edges
        // until we add a new triangle or point.
        // XXX: need to think about this. Can these edges be tried after we
        //      return to previous recursive level?
        t.delaunayEdge[i] = false;
        ot.delaunayEdge[oi] = false;

        // If triangle have been legalized no need to check the other edges since
        // the recursive legalization will handles those so we can end here.
        return true;
      }
    }
  }

  return false;
}

/**
 * <b>Requirement</b>
 * :
 * <br />
 * 1. a,b and c form a triangle.
 * <br />
 * 2. a and d is know to be on opposite side of bc
 * <br />
 * <pre>
 *     a
 *     +
 *     / \
 *     /   \
 *     b/     \c
 *     +-------+
 *     /    d    \
 *     /           \
 * </pre>
 * <b>Fact</b>
 * : d has to be in area B to have a chance to be inside the circle formed by
 * a,b and c
 * <br />
 * d is outside B if orient2d(a,b,d) or orient2d(c,a,d) is CW
 * <br />
 * This preknowledge gives us a way to optimize the incircle test
 * @param a - triangle point, opposite d
 * @param b - triangle point
 * @param c - triangle point
 * @param d - point opposite a
 * @return true if d is inside circle, false if on circle edge
 */
function incircle(pa: Point, pb: Point, pc: Point, pd: Point): boolean {
  const adx = pa.position[0] - pd.position[0];
  const ady = pa.position[1] - pd.position[1];
  const bdx = pb.position[0] - pd.position[0];
  const bdy = pb.position[1] - pd.position[1];

  const adxbdy = adx * bdy;
  const bdxady = bdx * ady;
  const oabd = adxbdy - bdxady;

  if (oabd <= 0) {
    return false;
  }

  const cdx = pc.position[0] - pd.position[0];
  const cdy = pc.position[1] - pd.position[1];

  const cdxady = cdx * ady;
  const adxcdy = adx * cdy;
  const ocad = cdxady - adxcdy;

  if (ocad <= 0) {
    return false;
  }

  const bdxcdy = bdx * cdy;
  const cdxbdy = cdx * bdy;

  const alift = adx * adx + ady * ady;
  const blift = bdx * bdx + bdy * bdy;
  const clift = cdx * cdx + cdy * cdy;

  const det = alift * (bdxcdy - cdxbdy) + blift * ocad + clift * oabd;

  return det > 0;
}

/**
 * Rotates a triangle pair one vertex CW
 * <pre>
 *     n2                    n2
 *     P +-----+             P +-----+
 *     | t  /|               |\  t |
 *     |   / |               | \   |
 *     n1|  /  |n3           n1|  \  |n3
 *     | /   |    after CW   |   \ |
 *     |/ oT |               | oT \|
 *     +-----+ oP            +-----+
 *     n4                    n4
 * </pre>
 */
function rotateTrianglePair(t: Triangle, p: Point, ot: Triangle, op: Point) {
  const n1 = t.neighborCCW(p);
  const n2 = t.neighborCW(p);
  const n3 = ot.neighborCCW(op);
  const n4 = ot.neighborCW(op);

  const ce1 = t.getConstrainedEdgeCCW(p);
  const ce2 = t.getConstrainedEdgeCW(p);
  const ce3 = ot.getConstrainedEdgeCCW(op);
  const ce4 = ot.getConstrainedEdgeCW(op);

  const de1 = t.getDelunayEdgeCCW(p);
  const de2 = t.getDelunayEdgeCW(p);
  const de3 = ot.getDelunayEdgeCCW(op);
  const de4 = ot.getDelunayEdgeCW(op);

  t.legalize(p, op);
  ot.legalize(op, p);

  // Remap delaunay_edge
  ot.setDelunayEdgeCCW(p, de1);
  t.setDelunayEdgeCW(p, de2);
  t.setDelunayEdgeCCW(op, de3);
  ot.setDelunayEdgeCW(op, de4);

  // Remap constrained_edge
  ot.setConstrainedEdgeCCW(p, ce1);
  t.setConstrainedEdgeCW(p, ce2);
  t.setConstrainedEdgeCCW(op, ce3);
  ot.setConstrainedEdgeCW(op, ce4);

  // Remap neighbors
  // XXX: might optimize the markNeighbor by keeping track of
  //      what side should be assigned to what neighbor after the
  //      rotation. Now mark neighbor does lots of testing to find
  //      the right side.
  t.clearNeighbors();
  ot.clearNeighbors();
  if (n1 !== null) {
    ot.markNeighbor(n1);
  }

  if (n2 !== null) {
    t.markNeighbor(n2);
  }

  if (n3 !== null) {
    t.markNeighbor(n3);
  }

  if (n4 !== null) {
    ot.markNeighbor(n4);
  }

  t.markNeighbor(ot);
}

/**
 * Fills holes in the Advancing Front
 *
 *
 * @param tcx
 * @param n
 */
function fillAdvancingFront(tcx: SweepContext, n: Node) {
  // Fill right holes
  let node = n.next;

  while (node !== null && node.next !== null) {
    // if HoleAngle exceeds 90 degrees then break.
    if (largeHoleDoNotFill(node)) {
      break;
    }

    fill(tcx, node);
    node = node.next;
  }

  // Fill left holes
  node = n.prev;

  while (node !== null && node.prev !== null) {
    // if HoleAngle exceeds 90 degrees then break.
    if (largeHoleDoNotFill(node)) {
      break;
    }

    fill(tcx, node);
    node = node.prev;
  }

  // Fill right basins
  if (n.next !== null && n.next.next !== null) {
    const angle = basinAngle(n);
    if (angle < PI_3div4) {
      fillBasin(tcx, n);
    }
  }
}

// Decision-making about when to Fill hole. True if HoleAngle exceeds 90 degrees.
// Contributed by ToolmakerSteve2
function largeHoleDoNotFill(node: Node): boolean {
  if (node.prev === null)
    throw new Error('largeHole_DontFill: node.prev is null');
  if (node.next === null)
    throw new Error('largeHole_DontFill: node.next is null');

  const nextNode = node.next;
  const prevNode = node.prev;
  if (!angleExceeds90Degrees(node.point, nextNode.point, prevNode.point)) {
    return false;
  }

  // Check additional points on front.
  const next2Node = nextNode.next;
  // "..Plus.." because only want angles on same side as point being added.
  if (
    next2Node !== null &&
    !angleExceedsPlus90DegreesOrIsNegative(
      node.point,
      next2Node.point,
      prevNode.point
    )
  ) {
    return false;
  }

  const prev2Node = prevNode.prev;
  // "..Plus.." because only want angles on same side as point being added.
  if (
    prev2Node !== null &&
    !angleExceedsPlus90DegreesOrIsNegative(
      node.point,
      nextNode.point,
      prev2Node.point
    )
  ) {
    return false;
  }

  return true;
}

function angleExceeds90Degrees(origin: Point, pa: Point, pb: Point): boolean {
  const a = angle(origin, pa, pb);
  return a > PI_div2 || a < -PI_div2;
}

function angleExceedsPlus90DegreesOrIsNegative(
  origin: Point,
  pa: Point,
  pb: Point
): boolean {
  const a = angle(origin, pa, pb);
  return a > PI_div2 || a < 0;
}

function angle(origin: Point, pa: Point, pb: Point): number {
  /* Complex plane
   * ab = cosA +i*sinA
   * ab = (ax + ay*i)(bx + by*i) = (ax*bx + ay*by) + i(ax*by-ay*bx)
   * atan2(y,x) computes the principal value of the argument function
   * applied to the complex number x+iy
   * Where x = ax*bx + ay*by
   *       y = ax*by - ay*bx
   */
  const px = origin.position[0];
  const py = origin.position[1];
  const ax = pa.position[0] - px;
  const ay = pa.position[1] - py;
  const bx = pb.position[0] - px;
  const by = pb.position[1] - py;
  const x = ax * by - ay * bx;
  const y = ax * bx + ay * by;
  return Math.atan2(x, y);
}

/**
 * The basin angle is decided against the horizontal line [1,0]
 */
function basinAngle(node: Node): number {
  if (node.next === null || node.next.next === null)
    throw new Error('basinAngle: node.next.next is null');

  const ax = node.point.position[0] - node.next.next.point.position[0];
  const ay = node.point.position[1] - node.next.next.point.position[1];
  return Math.atan2(ay, ax);
}

/**
 * Fills a basin that has formed on the Advancing Front to the right
 * of given node.
 * <br />
 * First we decide a left,bottom and right node that forms the
 * boundaries of the basin. Then we do a reqursive fill.
 * @param tcx
 * @param node - starting node, this or next node will be left node
 */
function fillBasin(tcx: SweepContext, node: Node) {
  if (node.next === null || node.next.next === null)
    throw new Error('fillBasin: node.next.next is null');

  tcx.basin.leftNode =
    orient2d(node.point, node.next.point, node.next.next.point) ===
    Orientation.CCW
      ? node.next.next
      : node.next;

  // Find the bottom and right node
  tcx.basin.bottomNode = tcx.basin.leftNode;
  while (
    tcx.basin.bottomNode.next !== null &&
    tcx.basin.bottomNode.point.position[1] >=
      tcx.basin.bottomNode.next.point.position[1]
  ) {
    tcx.basin.bottomNode = tcx.basin.bottomNode.next;
  }

  if (tcx.basin.bottomNode === tcx.basin.leftNode) {
    // No valid basin
    return;
  }

  tcx.basin.rightNode = tcx.basin.bottomNode;
  while (
    tcx.basin.rightNode.next !== null &&
    tcx.basin.rightNode.point.position[1] <
      tcx.basin.rightNode.next.point.position[1]
  ) {
    tcx.basin.rightNode = tcx.basin.rightNode.next;
  }

  if (tcx.basin.rightNode === tcx.basin.bottomNode) {
    // No valid basins
    return;
  }

  tcx.basin.width =
    tcx.basin.rightNode.point.position[0] -
    tcx.basin.leftNode.point.position[0];
  tcx.basin.leftHighest =
    tcx.basin.leftNode.point.position[1] >
    tcx.basin.rightNode.point.position[1];

  fillBasinReq(tcx, tcx.basin.bottomNode);
}

/**
 * Recursive algorithm to fill a Basin with triangles
 *
 * @param tcx
 * @param node - bottom_node
 * @param cnt - counter used to alternate on even and odd numbers
 */
function fillBasinReq(tcx: SweepContext, node: Node) {
  // if shallow stop filling
  if (isShallow(tcx, node)) {
    return;
  }

  fill(tcx, node);

  if (node.prev === tcx.basin.leftNode && node.next === tcx.basin.rightNode) {
    return;
  }

  if (node.prev === tcx.basin.leftNode) {
    if (node.next === null || node.next.next === null)
      throw new Error('fillBasinReq: node.next.next is null');

    const o = orient2d(node.point, node.next.point, node.next.next.point);
    if (o === Orientation.CW) {
      return;
    }

    node = node.next;
  } else if (node.next === tcx.basin.rightNode) {
    if (node.prev === null || node.prev.prev === null)
      throw new Error('fillBasinReq: node.prev.prev is null');

    const o = orient2d(node.point, node.prev.point, node.prev.prev.point);
    if (o === Orientation.CCW) {
      return;
    }

    node = node.prev;
  } else {
    if (node.next === null || node.next.next === null)
      throw new Error('fillBasinReq: node.next.next is null');
    if (node.prev === null || node.prev.prev === null)
      throw new Error('fillBasinReq: node.prev.prev is null');

    // Continue with the neighbor node with lowest Y value
    node =
      node.prev.point.position[1] < node.next.point.position[1]
        ? node.prev
        : node.next;
  }

  fillBasinReq(tcx, node);
}

function isShallow(tcx: SweepContext, node: Node): boolean {
  let height: number;

  if (tcx.basin.leftHighest) {
    if (tcx.basin.leftNode === null)
      throw new Error('isShallow: tcx.basin.leftNode is null');

    height = tcx.basin.leftNode.point.position[1] - node.point.position[1];
  } else {
    if (tcx.basin.rightNode === null)
      throw new Error('isShallow: tcx.basin.rightNode is null');

    height = tcx.basin.rightNode.point.position[1] - node.point.position[1];
  }

  // if shallow stop filling
  if (tcx.basin.width > height) {
    return true;
  }

  return false;
}

function isEdgeSideOfTriangle(
  triangle: Triangle | null,
  ep: Point,
  eq: Point
): boolean {
  if (triangle === null) {
    return false;
  }

  const index = triangle.edgeIndex(ep, eq);

  if (index !== -1) {
    triangle.markConstrainedEdgeByIndex(index);
    const t = triangle.getNeighbor(index);
    if (t !== null) {
      t.markConstrainedEdge(ep, eq);
    }

    return true;
  }

  return false;
}

function fillEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (tcx.edgeEvent.right) {
    fillRightAboveEdgeEvent(tcx, edge, node);
  } else {
    fillLeftAboveEdgeEvent(tcx, edge, node);
  }
}

function fillRightAboveEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  while (
    node.next !== null &&
    node.next.point.position[0] < edge.p.position[0]
  ) {
    // Check if next node is below the edge
    if (orient2d(edge.q, node.next.point, edge.p) === Orientation.CCW) {
      fillRightBelowEdgeEvent(tcx, edge, node);
    } else {
      node = node.next;
    }
  }
}

function fillRightBelowEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (node.next === null || node.next.next === null)
    throw new Error('fillRightBelowEdgeEvent: node.next.next is null');

  if (node.point.position[0] < edge.p.position[0]) {
    if (
      orient2d(node.point, node.next.point, node.next.next.point) ===
      Orientation.CCW
    ) {
      // Concave
      fillRightConcaveEdgeEvent(tcx, edge, node);
    } else {
      // Convex
      fillRightConvexEdgeEvent(tcx, edge, node);
      // Retry this one
      fillRightBelowEdgeEvent(tcx, edge, node);
    }
  }
}

function fillRightConcaveEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (node.next === null)
    throw new Error('fillRightConcaveEdgeEvent: node.next is null');

  fill(tcx, node.next);
  if (!pointsEqual(node.next.point, edge.p)) {
    // Next above or below edge?
    if (orient2d(edge.q, node.next.point, edge.p) === Orientation.CCW) {
      if (node.next.next === null)
        throw new Error('fillRightConcaveEdgeEvent: node.next.next is null');

      // Below
      if (
        orient2d(node.point, node.next.point, node.next.next.point) ===
        Orientation.CCW
      ) {
        // Next is concave
        fillRightConcaveEdgeEvent(tcx, edge, node);
      }
    }
  }
}

function fillRightConvexEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (
    node.next === null ||
    node.next.next === null ||
    node.next.next.next === null
  )
    throw new Error('fillRightConvexEdgeEvent: node.next.next.next is null');

  // Next concave or convex?
  if (
    orient2d(
      node.next.point,
      node.next.next.point,
      node.next.next.next.point
    ) === Orientation.CCW
  ) {
    // Concave
    fillRightConcaveEdgeEvent(tcx, edge, node.next);
  } else {
    // Convex
    // Next above or below edge?
    if (orient2d(edge.q, node.next.next.point, edge.p) === Orientation.CCW) {
      // Below
      fillRightConvexEdgeEvent(tcx, edge, node.next);
    }
  }
}

function fillLeftAboveEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  while (
    node.prev !== null &&
    node.prev.point.position[0] > edge.p.position[0]
  ) {
    // Check if next node is below the edge
    if (orient2d(edge.q, node.prev.point, edge.p) === Orientation.CW) {
      fillLeftBelowEdgeEvent(tcx, edge, node);
    } else {
      node = node.prev;
    }
  }
}

function fillLeftBelowEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (node.point.position[0] > edge.p.position[0]) {
    if (node.prev === null || node.prev.prev === null)
      throw new Error('fillLeftBelowEdgeEvent: node.prev.prev is null');

    if (
      orient2d(node.point, node.prev.point, node.prev.prev.point) ===
      Orientation.CW
    ) {
      // Concave
      fillLeftConcaveEdgeEvent(tcx, edge, node);
    } else {
      // Convex
      fillLeftConvexEdgeEvent(tcx, edge, node);
      // Retry this one
      fillLeftBelowEdgeEvent(tcx, edge, node);
    }
  }
}

function fillLeftConcaveEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (node.prev === null)
    throw new Error('fillLeftConcaveEdgeEvent: node.prev is null');

  fill(tcx, node.prev);
  if (!pointsEqual(node.prev.point, edge.p)) {
    // Next above or below edge?
    if (orient2d(edge.q, node.prev.point, edge.p) === Orientation.CW) {
      if (node.prev.prev === null)
        throw new Error('fillLeftConcaveEdgeEvent: node.prev.prev is null');

      // Below
      if (
        orient2d(node.point, node.prev.point, node.prev.prev.point) ===
        Orientation.CW
      ) {
        // Next is concave
        fillLeftConcaveEdgeEvent(tcx, edge, node);
      }
    }
  }
}

function fillLeftConvexEdgeEvent(tcx: SweepContext, edge: Edge, node: Node) {
  if (
    node.prev === null ||
    node.prev.prev === null ||
    node.prev.prev.prev === null
  )
    throw new Error('fillLeftConvexEdgeEvent: node.prev.prev.prev is null');

  // Next concave or convex?
  if (
    orient2d(
      node.prev.point,
      node.prev.prev.point,
      node.prev.prev.prev.point
    ) === Orientation.CW
  ) {
    // Concave
    fillLeftConcaveEdgeEvent(tcx, edge, node.prev);
  } else {
    // Convex
    // Next above or below edge?
    if (orient2d(edge.q, node.prev.prev.point, edge.p) === Orientation.CW) {
      // Below
      fillLeftConvexEdgeEvent(tcx, edge, node.prev);
    }
  }
}

function flipEdgeEvent(
  tcx: SweepContext,
  ep: Point,
  eq: Point,
  t: Triangle,
  p: Point
) {
  if (t === null) {
    throw new Error('flipEdgeEvent: triangle is null');
  }

  const ot = t.neighborAcross(p);
  if (ot === null) {
    throw new Error('flipEdgeEvent: null neighbor across');
  }

  const op = ot.oppositePoint(t, p);

  if (inScanArea(p, t.pointCCW(p), t.pointCW(p), op)) {
    // Lets rotate shared edge one vertex CW
    rotateTrianglePair(t, p, ot, op);
    tcx.mapTriangleToNodes(t);
    tcx.mapTriangleToNodes(ot);

    if (pointsEqual(p, eq) && pointsEqual(op, ep)) {
      const edge = tcx.edgeEvent.constrainedEdge;
      if (edge === null) {
        throw new Error('flipEdgeEvent: tcx.edgeEvent.constrainedEdge is null');
      }

      if (pointsEqual(eq, edge.q) && pointsEqual(ep, edge.p)) {
        t.markConstrainedEdge(ep, eq);
        ot.markConstrainedEdge(ep, eq);
        legalize(tcx, t);
        legalize(tcx, ot);
      }
    } else {
      const o = orient2d(eq, op, ep);
      t = nextFlipTriangle(tcx, o, t, ot, p, op);
      flipEdgeEvent(tcx, ep, eq, t, p);
    }
  } else {
    const newP = nextFlipPoint(ep, eq, ot, op);
    flipScanEdgeEvent(tcx, ep, eq, t, ot, newP);
    edgeEventWithTriangle(tcx, ep, eq, t, p);
  }
}

/**
 * After a flip we have two triangles and know that only one will still be
 * intersecting the edge. So decide which to contiune with and legalize the other
 *
 * @param tcx
 * @param o - should be the result of an orient2d( eq, op, ep )
 * @param t - triangle 1
 * @param ot - triangle 2
 * @param p - a point shared by both triangles
 * @param op - another point shared by both triangles
 * @return returns the triangle still intersecting the edge
 */
function nextFlipTriangle(
  tcx: SweepContext,
  o: Orientation,
  t: Triangle,
  ot: Triangle,
  p: Point,
  op: Point
): Triangle {
  let edgeIndex: number;
  if (o === Orientation.CCW) {
    // ot is not crossing edge after flip
    edgeIndex = ot.edgeIndex(p, op);
    ot.delaunayEdge[edgeIndex] = true;
    legalize(tcx, ot);
    ot.clearDelunayEdges();
    return t;
  }

  // t is not crossing edge after flip
  edgeIndex = t.edgeIndex(p, op);

  t.delaunayEdge[edgeIndex] = true;
  legalize(tcx, t);
  t.clearDelunayEdges();
  return ot;
}

/**
 * When we need to traverse from one triangle to the next we need
 * the point in current triangle that is the opposite point to the next
 * triangle.
 *
 * @param ep
 * @param eq
 * @param ot
 * @param op
 * @return
 */
function nextFlipPoint(ep: Point, eq: Point, ot: Triangle, op: Point): Point {
  const o2d = orient2d(eq, op, ep);
  if (o2d === Orientation.CW) {
    // Right
    return ot.pointCCW(op);
  }

  if (o2d === Orientation.CCW) {
    // Left
    return ot.pointCW(op);
  }

  throw new Error('nextFlipPoint: opposing point on constrained edge');
}

/**
 * Scan part of the FlipScan algorithm
 * <br />
 * When a triangle pair isn't flippable we will scan for the next
 * point that is inside the flip triangle scan area. When found
 * we generate a new flipEdgeEvent
 * @param tcx
 * @param ep - last point on the edge we are traversing
 * @param eq - first point on the edge we are traversing
 * @param flipTriangle - the current triangle sharing the point eq with edge
 * @param t
 * @param p
 */
function flipScanEdgeEvent(
  tcx: SweepContext,
  ep: Point,
  eq: Point,
  flipTriangle: Triangle,
  t: Triangle,
  p: Point
) {
  const ot = t.neighborAcross(p);
  if (ot === null) {
    throw new Error('flipScanEdgeEvent: null neighbor across');
  }

  const op = ot.oppositePoint(t, p);
  const p1 = flipTriangle.pointCCW(eq);
  const p2 = flipTriangle.pointCW(eq);

  if (inScanArea(eq, p1, p2, op)) {
    // flip with new edge op->eq
    flipEdgeEvent(tcx, eq, op, ot, op);
    // TODO: Actually I just figured out that it should be possible to
    //       improve this by getting the next ot and op before the the above
    //       flip and continue the flipScanEdgeEvent here
    // set new ot and op here and loop back to inScanArea test
    // also need to set a new flip_triangle first
    // Turns out at first glance that this is somewhat complicated
    // so it will have to wait.
  } else {
    const newP = nextFlipPoint(ep, eq, ot, op);
    flipScanEdgeEvent(tcx, ep, eq, flipTriangle, ot, newP);
  }
}

function finalizationPolygon(tcx: SweepContext) {
  // Get an Internal triangle to start with
  const head = tcx.advancingFront.head;
  if (head === null || head.next === null) {
    throw new Error('finalizationPolygon: tcx.front.head.next is null');
  }

  let t = head.next.triangle;
  const p = head.next.point;
  while (t !== null && !t.getConstrainedEdgeCW(p)) {
    t = t.neighborCCW(p);
  }

  // Collect interior triangles constrained by edges
  if (t !== null) {
    tcx.meshClean(t);
  }
}
