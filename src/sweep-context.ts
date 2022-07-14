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

import { vec2 } from 'gl-matrix';
import { AdvancingFront, makeNode, Node } from './advancing-front';
import { Edge, makeEdge, makePoint, Point } from './edge';
import { Triangle } from './triangle';

interface Basin {
  bottomNode: Node | null;
  leftHighest: boolean;
  leftNode: Node | null;
  rightNode: Node | null;
  width: number;
}

interface EdgeEvent {
  constrainedEdge: Edge | null;
  right: boolean;
}

// Inital triangle factor, seed triangle will extend 30% of
// PointSet width to both left and right.
const KAlpha = 0.3;

export class SweepContext {
  private readonly edgeList: Edge[] = [];
  private readonly map: Triangle[] = [];

  private readonly points: Point[];

  private readonly triangles: Triangle[] = [];
  public readonly basin: Basin = {
    bottomNode: null,
    leftHighest: false,
    leftNode: null,
    rightNode: null,
    width: 0,
  };
  public readonly edgeEvent: EdgeEvent = {
    constrainedEdge: null,
    right: false,
  };

  private afHead: Node | null = null;
  private afMiddle: Node | null = null;
  private afTail: Node | null = null;

  // Advancing front
  private front: AdvancingFront | null = null;

  // head point used with advancing front
  private head: Point | null = null;

  // tail point used with advancing front
  private tail: Point | null = null;

  /// Constructor
  public constructor(polyline: Point[]) {
    this.points = polyline;
    this.initEdges(this.points);
  }

  public get pointCount(): number {
    return this.points.length;
  }

  public locateNode(point: Point): Node | null {
    if (this.front === null)
      throw new Error('SweepContext::locateNode: not initialized');

    // TODO implement search tree
    return this.front.locateNode(point.position[0]);
  }

  public createAdvancingFront() {
    if (this.head === null || this.tail === null)
      throw new Error('SweepContext::createAdvancingFront: not initialized');

    // Initial triangle
    const triangle = new Triangle(this.points[0], this.head, this.tail);

    this.map.push(triangle);

    this.afHead = makeNode(triangle.getPoint(1), triangle);
    this.afMiddle = makeNode(triangle.getPoint(0), triangle);
    this.afTail = makeNode(triangle.getPoint(2));
    this.front = new AdvancingFront(this.afHead, this.afTail);

    // TODO: More intuitive if head is middles next and not previous?
    //       so swap head and tail
    this.afHead.next = this.afMiddle;
    this.afMiddle.next = this.afTail;
    this.afMiddle.prev = this.afHead;
    this.afTail.prev = this.afMiddle;
  }

  /// Try to map a node to all sides of this triangle that don't have a neighbor
  public mapTriangleToNodes(t: Triangle) {
    if (this.front === null)
      throw new Error('SweepContext::mapTriangleToNodes: not initialized');

    for (let i = 0; i < 3; i++)
      if (t.getNeighbor(i) === null) {
        const n = this.front.locatePoint(t.pointCW(t.getPoint(i)));
        if (n !== null) n.triangle = t;
      }
  }

  public addToMap(triangle: Triangle) {
    this.map.push(triangle);
  }

  public getPoint(index: number): Point {
    return this.points[index];
  }

  public addHole(polyline: Point[]) {
    this.initEdges(polyline);
    for (const t of polyline) this.points.push(t);
  }

  public addPoint(point: Point) {
    this.points.push(point);
  }

  public get advancingFront(): AdvancingFront {
    if (this.front === null)
      throw new Error('SweepContext::advancingFront: not initialized');

    return this.front;
  }

  public meshClean(triangle: Triangle) {
    const triangles: (Triangle | null)[] = [triangle];

    while (triangles.length > 0) {
      const t = triangles[triangles.length - 1];
      triangles.splice(triangles.length - 1, 1);

      if (t !== null && !t.isInterior) {
        t.isInterior = true;
        this.triangles.push(t);
        for (let i = 0; i < 3; i++)
          if (!t.constrainedEdge[i]) {
            triangles.push(t.getNeighbor(i));
          }
      }
    }
  }

  public getTriangles(): Triangle[] {
    return this.triangles;
  }

  public getMap(): Triangle[] {
    return this.map;
  }

  public initTriangulation() {
    let xmax = this.points[0].position[0],
      xmin = this.points[0].position[0];
    let ymax = this.points[0].position[1],
      ymin = this.points[0].position[1];

    // Calculate bounds.
    for (const p of this.points) {
      if (p.position[0] > xmax) xmax = p.position[0];

      if (p.position[0] < xmin) xmin = p.position[0];

      if (p.position[1] > ymax) ymax = p.position[1];

      if (p.position[1] < ymin) ymin = p.position[1];
    }

    const dx = KAlpha * (xmax - xmin);
    const dy = KAlpha * (ymax - ymin);
    this.head = makePoint(vec2.fromValues(xmin - dx, ymin - dy));
    this.tail = makePoint(vec2.fromValues(xmax + dx, ymin - dy));

    // Sort points along y-axis
    this.points.sort((a, b) => {
      if (a.position[1] < b.position[1]) return -1;

      if (a.position[1] === b.position[1]) {
        // Make sure q is point with greater x value
        if (a.position[0] < b.position[0]) return -1;

        return 0;
      }

      return 1;
    });
  }

  private initEdges(polyline: Point[]) {
    const numPoints = polyline.length;
    for (let i = 0; i < numPoints; i++) {
      const j = i < numPoints - 1 ? i + 1 : 0;
      this.edgeList.push(makeEdge(polyline[i], polyline[j]));
    }
  }
}
